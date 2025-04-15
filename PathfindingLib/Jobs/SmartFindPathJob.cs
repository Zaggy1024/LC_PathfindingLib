using System;
using System.Collections.Generic;

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.AI;

using PathfindingLib.API;
using PathfindingLib.API.SmartPathfinding;
using PathfindingLib.Utilities;
using PathfindingLib.Utilities.Collections;

#if BENCHMARKING
using System.Diagnostics;
using Unity.Profiling;
#endif

#if SMART_PATHFINDING_DEBUG
using System.Text;
#endif

namespace PathfindingLib.Jobs;

internal struct SmartFindPathJob : IJob
{
    internal const int PlaceholderLinkIndex = int.MinValue;

    internal struct PathResult()
    {
        internal int linkIndex = PlaceholderLinkIndex;
        internal int linkDestinationIndex = PlaceholderLinkIndex;
        internal float pathLength = float.PositiveInfinity;
    }

    internal const float MinEdgeCost = 0.0001f;

    private const int MaxPathsToTest = 30;

    // Inputs:
    [ReadOnly, NativeDisableContainerSafetyRestriction] private NativeArray<NavMeshQuery> ThreadQueriesRef;

    [ReadOnly] private int agentTypeID;
    [ReadOnly] private int areaMask;
    [ReadOnly] private Vector3 start;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<Vector3> goals;
    [ReadOnly] private int goalCount;

    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<Vector3> linkOrigins;

    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<IndexAndSize> linkDestinationSlices;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<Vector3> linkDestinations;

    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<int> linkDestinationCostOffsets;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<float> linkDestinationCosts;

    [ReadOnly] private int linkCount;
    [ReadOnly] private int linkDestinationCount;

    [ReadOnly] private int vertexCount;

    [ReadOnly, NativeSetThreadIndex] private int threadIndex;

    // Outputs:
    [WriteOnly] internal NativeArray<PathResult> results;

    internal void Initialize(SmartPathJobDataContainer data)
    {
        ThreadQueriesRef = PathfindingJobSharedResources.GetPerThreadQueriesArray();

        agentTypeID = data.agentTypeID;
        areaMask = data.areaMask;

        start = data.pathStart;
        goals = data.pathGoals;
        goalCount = data.pathGoalCount;
        results = data.pathResults;

        // Shhhh, compiler...
        threadIndex = -1;

        linkOrigins = data.linkOrigins;

        linkDestinationSlices = data.linkDestinationSlices;
        linkDestinations = data.linkDestinations;

        linkDestinationCostOffsets = data.linkDestinationCostOffsets;
        linkDestinationCosts = data.linkDestinationCosts;

        linkCount = data.linkCount;
        linkDestinationCount = data.linkDestinationCount;

        vertexCount = linkCount + linkDestinationCount + 2;
    }

    private void DisposeResizeableArrays()
    {
        if (!goals.IsCreated)
            return;

        goals.Dispose();
        results.Dispose();
    }

#if BENCHMARKING
    private static readonly ProfilerMarker CalculateSinglePathMarker = new("CalculateSinglePath");
#endif

    // The path vertices are made up of:
    // - [linkCount]: Link origins
    // - [linkDestinationCount]: Link destinations
    // - [1]: Start
    // - [1]: Goal
    private readonly int LinkDestinationsOffset => linkCount;
    private readonly int StartVertexIndex => vertexCount - 2;
    private readonly int GoalVertexIndex => vertexCount - 1;

    private readonly float CalculateSinglePath(Vector3 origin, Vector3 destination)
    {
        var query = ThreadQueriesRef[threadIndex];

        using var readLocker = new NavMeshReadLocker();

#if BENCHMARKING
        using var markerAuto = new TogglableProfilerAuto(in CalculateSinglePathMarker);
#endif

        var startLocation = query.MapLocation(origin, SharedJobValues.OriginExtents, agentTypeID, areaMask);

        if (!query.IsValid(startLocation.polygon))
            return float.PositiveInfinity;

        var destinationExtents = SharedJobValues.DestinationExtents;
        var destinationLocation = query.MapLocation(destination, destinationExtents, agentTypeID, areaMask);
        if (!query.IsValid(destinationLocation))
            return float.PositiveInfinity;

        var status = query.BeginFindPath(startLocation, destinationLocation, areaMask);
        if (status.GetResult() == PathQueryStatus.Failure)
            return float.PositiveInfinity;

        while (status.GetResult() == PathQueryStatus.InProgress)
        {
            status = query.UpdateFindPath(NavMeshLock.RecommendedUpdateFindPathIterationCount, out int _);

#if BENCHMARKING
            markerAuto.Pause();
#endif
            readLocker.Yield();
#if BENCHMARKING
            markerAuto.Resume();
#endif
        }

        status = query.EndFindPath(out var pathNodesSize);
        if (status.GetResult() != PathQueryStatus.Success)
            return float.PositiveInfinity;

        var pathNodes = new NativeArray<PolygonId>(pathNodesSize, Allocator.Temp);
        pathNodesSize = query.GetPathResult(pathNodes);

        using var path = new NativeArray<Vector3>(NavMeshQueryUtils.RecommendedCornerCount, Allocator.Temp);
        var straightPathStatus = NavMeshQueryUtils.FindStraightPath(query, origin, destination, pathNodes, pathNodesSize, path, out var pathSize);
        pathNodes.Dispose();

        // ReSharper disable once DisposeOnUsingVariable
        readLocker.Dispose();

        if (straightPathStatus.GetResult() != PathQueryStatus.Success)
            return float.PositiveInfinity;

        // Check if the end of the path is close enough to the target.
        var pathEnd = path[pathSize - 1];
        var endDistance = math.distancesq(destination, pathEnd);
        if (endDistance > SharedJobValues.MaximumEndpointDistanceSquared)
            return float.PositiveInfinity;

        var distance = 0f;
        for (var i = 1; i < pathSize; i++)
            distance += math.distance(path[i - 1], path[i]);

        return distance;
    }

    private readonly void FailPath(int index)
    {
        ref var result = ref results.GetRef(index);
        result.linkIndex = -1;
        result.linkDestinationIndex = -1;
    }

    public void Execute()
    {
#if SMART_PATHFINDING_DEBUG
        if (captureNextVertices)
            InitVerticesCapture(goalCount);
#endif

#if BENCHMARKING
        var jobStopwatch = Stopwatch.StartNew();
        var goalStopwatch = new Stopwatch();
#endif

        var memory = new PathfinderMemory(vertexCount);
        PopulateVertices(memory);

        for (var goalIndex = 0; goalIndex < goalCount; goalIndex++)
        {
#if BENCHMARKING
            goalStopwatch.Restart();
#endif

            SetNewGoal(memory, goalIndex);

            if (!memory.HasSuccessors(StartVertexIndex, in this))
            {
#if SMART_PATHFINDING_DEBUG
                if (captureNextVertices)
                    CaptureVerticesSnapshot(memory, goalIndex);

                PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed, start position is isolated.");
#endif

                FailPath(goalIndex);
                continue;
            }

            if (!memory.HasPredecessors(GoalVertexIndex, in this))
            {
#if SMART_PATHFINDING_DEBUG
                if (captureNextVertices)
                    CaptureVerticesSnapshot(memory, goalIndex);

                PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed, goal position is isolated.");
#endif

                FailPath(goalIndex);
                continue;
            }

            var result = CalculatePath(memory);

#if BENCHMARKING
            goalStopwatch.Stop();
            PathfindingLibPlugin.Instance.Logger.LogInfo($"Path to Goal {goalIndex} took {goalStopwatch.Elapsed.TotalMilliseconds}ms");
#endif

#if SMART_PATHFINDING_DEBUG
            if (captureNextVertices)
                CaptureVerticesSnapshot(memory, goalIndex);

            PrintCurrPath(memory, goalIndex);

            if (result.linkIndex == -1)
                PathfindingLibPlugin.Instance.Logger.LogInfo($"Path for goal {goalIndex} failed.");
            else if (result.linkIndex == linkCount)
                PathfindingLibPlugin.Instance.Logger.LogInfo($"Path for goal {goalIndex} was a direct path with length {result.pathLength}.");
            else
                PathfindingLibPlugin.Instance.Logger.LogInfo($"Path for goal {goalIndex} was an indirect path through link index {result.linkIndex} ({SmartPathJobDataContainer.linkNames[result.linkIndex]}).");
#endif

            results[goalIndex] = result;
        }

        memory.Dispose();

#if BENCHMARKING
        PathfindingLibPlugin.Instance.Logger.LogWarning($"Job took {jobStopwatch.Elapsed.TotalMilliseconds}ms.");
#endif

#if SMART_PATHFINDING_DEBUG
        captureNextVertices = false;
#endif
    }

    private readonly Vector3 GetVertexPosition(int vertexIndex)
    {
        if (vertexIndex > vertexCount)
            throw new IndexOutOfRangeException($"Index {vertexIndex} is out of range.");

        if (vertexIndex == GoalVertexIndex)
            throw new NotImplementedException("Should never get called for the goal");

        if (vertexIndex == StartVertexIndex)
            return start;

        if (vertexIndex >= LinkDestinationsOffset)
            return linkDestinations[vertexIndex - LinkDestinationsOffset];

        if (vertexIndex >= 0)
            return linkOrigins[vertexIndex];

        throw new IndexOutOfRangeException("Index cannot be negative.");
    }

    private void PopulateVertices(PathfinderMemory memory)
    {
        for (var i = 0; i < GoalVertexIndex; i++)
        {
            ref var vertex = ref memory.GetVertex(i);
            var vertexPosition = GetVertexPosition(i);

            if (i >= LinkDestinationsOffset)
            {
                // Connect the start and all link destinations to all links' origins.
                vertex.heuristic = (start - vertexPosition).magnitude;

                for (var j = 0; j < linkCount; j++)
                {
                    ref var edge = ref memory.GetEdge(i, j);

                    edge.isValid = true;
                }
            }
            else if (i >= 0)
            {
                // Connect all link origins to their destinations.
                vertex.heuristic = (start - vertexPosition).magnitude;

                var slice = linkDestinationSlices[i];

                for (var j = 0; j < slice.size; j++)
                {
                    var destIndex = LinkDestinationsOffset + slice.index + j;
                    ref var edge = ref memory.GetEdge(i, destIndex);

                    edge.isValid = true;

                    var traverseCost = linkDestinationCosts[linkDestinationCostOffsets[i] + j];

                    if (traverseCost < MinEdgeCost)
                        traverseCost = MinEdgeCost;

                    edge.cost = traverseCost;
                }
            }
            else
            {
#if SMART_PATHFINDING_DEBUG
                PathfindingLibPlugin.Instance.Logger.LogError($"Index {i} is not mapped in PrepareData");
#endif
            }
        }
    }

    private void SetNewGoal(PathfinderMemory memory, int goalIndex)
    {
        ref var goalVertex = ref memory.GetVertex(GoalVertexIndex);
        var goalPosition = goals[goalIndex];

        for (var i = 0; i < vertexCount; i++)
        {
            ref var vertex = ref memory.GetVertex(i);
            var vertexPosition = (i == GoalVertexIndex) ? goalPosition : GetVertexPosition(i);

            if (i == GoalVertexIndex)
            {
                vertex.heuristic = (start - goalPosition).magnitude;
            }
            else if (i >= LinkDestinationsOffset)
            {
                vertex.heuristic = (start - vertexPosition).magnitude;

                for (var j = 0; j < linkCount; j++)
                {
                    //check if we can have a better estimation for the cost ( use the lowest value heuristic from the origins )
                    ref var edge = ref memory.GetEdge(j, i);

                    if(!edge.isValid)
                        continue;

                    var combinedHeuristic = (start - GetVertexPosition(j)).magnitude + (goalPosition - vertexPosition).magnitude;
                    if (vertex.heuristic > combinedHeuristic)
                        vertex.heuristic = combinedHeuristic;
                }

                ref var goalEdge = ref memory.GetEdge(i, GoalVertexIndex);

                goalEdge.isValid = true;
                //pre-compute costs to goals so we can remove the field
                goalEdge.cost = CalculateSinglePath(vertexPosition, goalPosition);
            }

            vertex.g = float.PositiveInfinity;
            vertex.rhs = float.PositiveInfinity;
        }

        goalVertex.rhs = 0;
        memory.heap.Clear();
        goalVertex.heapIndex = memory.heap.Insert(new HeapElement(GoalVertexIndex, goalVertex.CalculateKey()));
    }

    private static void UpdateOrAddVertex(PathfinderMemory memory, ref PathVertex vertex)
    {
        var heap = memory.heap;
        if (vertex.heapIndex.IsValid)
        {
            heap.Remove(vertex.heapIndex);
            vertex.heapIndex = NativeHeapIndex.Invalid;
        }
        if (!Mathf.Approximately(vertex.g, vertex.rhs))
            vertex.heapIndex = heap.Insert(new HeapElement(vertex.index, vertex.CalculateKey()));
    }

    private readonly PathResult CalculatePath(PathfinderMemory memory)
    {
        var heap = memory.heap;

        var goalVertexIndex = GoalVertexIndex;
        ref var startVertex = ref memory.GetVertex(StartVertexIndex);

        var iterations = 0;
        var extraIterations = 0;

        while (heap.Count > 0)
        {
            iterations += 1;
            var (index, oldKey) = heap.Peek();
            ref var currentVertex = ref memory.GetVertex(index);

            // If we've found a successful path, check more subsequent successful paths, within a limit.
            if (oldKey >= startVertex.CalculateKey() && startVertex.rhs <= startVertex.g && extraIterations++ > MaxPathsToTest)
                break;

            var newKey = currentVertex.CalculateKey();

            if (oldKey < newKey)
            {
                // If the new key is worse than the old one, reinsert into the list, sorting by the new key.
                heap.Pop();
                currentVertex.heapIndex = heap.Insert(new(index, newKey));
            }
            else if (currentVertex.g > currentVertex.rhs)
            {
                // Found a better path towards the current vertex.
                currentVertex.g = currentVertex.rhs;

                heap.Pop();
                currentVertex.heapIndex = NativeHeapIndex.Invalid;

                memory.CalculatePredecessors(index, in this);
                memory.ForEachPredecessor(index, (in PathEdge edge) =>
                {
                    ref var predecessorVertex = ref memory.GetVertex(edge.source);

                    if (edge.source != goalVertexIndex)
                    {
                        var newRhs = edge.cost + memory.GetVertex(index).g;

                        if (newRhs < predecessorVertex.rhs)
                            predecessorVertex.rhs = newRhs;
                    }

                    UpdateOrAddVertex(memory, ref predecessorVertex);
                });
            }
            else
            {
                // The cost has increased, so we have to update the vertex's cost as well as its predecessors'.
                // This appears to be unreachable given that we aren't changing the costs outside of the pathfinder.
                var gOld = currentVertex.g;
                currentVertex.g = float.PositiveInfinity;

                memory.CalculatePredecessors(index, in this);
                memory.ForEachPredecessor(index, (in PathEdge edge) =>
                {
                    RecalculateRHS(edge.source, edge.cost);
                });

                RecalculateRHS(index, 0);

                void RecalculateRHS(int vertex, float successorCost)
                {
                    ref var predVertex = ref memory.GetVertex(vertex);
                    if (vertex != goalVertexIndex)
                    {
                        if (Mathf.Approximately(predVertex.rhs, successorCost + gOld))
                        {
                            var minRhs = float.PositiveInfinity;

                            memory.ForEachSuccessor(vertex, (in PathEdge edge) =>
                            {
                                ref var succVertex = ref memory.GetVertex(edge.destination);

                                var newRhs = edge.cost + succVertex.g;

                                if (newRhs < minRhs)
                                    minRhs = newRhs;
                            });

                            predVertex.rhs = minRhs;
                        }
                    }

                    UpdateOrAddVertex(memory, ref predVertex);
                }
            }
        }

#if SMART_PATHFINDING_DEBUG
        PathfindingLibPlugin.Instance.Logger.LogDebug($"Found path after {iterations - extraIterations} iterations, then searched for an extra {extraIterations} iterations");
#endif

        // No path was found.
        if (float.IsInfinity(startVertex.rhs))
            return new();

        var result = new PathResult();
        result.linkIndex = memory.GetBestSuccessor(StartVertexIndex, out result.pathLength);
        result.linkDestinationIndex = memory.GetBestSuccessor(result.linkIndex, out _);

        if (result.linkDestinationIndex != -1)
            result.linkDestinationIndex -= LinkDestinationsOffset;

        // Return one greater than the possible nodes if we've gotten a direct path to the destination.
        if (result.linkIndex == GoalVertexIndex)
        {
            result.linkIndex = linkCount;
            return result;
        }

        if (result.linkIndex < linkCount)
            return result;

        PathfindingLibPlugin.Instance.Logger.LogFatal($"Smart pathfinding job found a best path that was a link destination. This should not be possible. First index = {result.linkIndex}, second index = {result.linkDestinationIndex}, link count = {linkCount}");
        return new();
    }

    private struct PathfinderMemory : IDisposable
    {
        private bool initialized;

        internal readonly int vertexCount;

        private NativeArray<PathVertex> vertices;
        private NativeArray<PathEdge> edges;

        internal NativeHeap<HeapElement, HeapElementComparer> heap;

        public PathfinderMemory()
        {
            throw new NotImplementedException();
        }

        public PathfinderMemory(int vertexCount)
        {
            this.vertexCount = vertexCount;
            vertices = new NativeArray<PathVertex>(vertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            edges = new NativeArray<PathEdge>(vertexCount * vertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            for (var i = 0; i < vertexCount * vertexCount; i++)
            {
                var destination = i / vertexCount;
                var source = i % vertexCount;
                edges[i] = new PathEdge(source, destination);

                if (i >= vertexCount)
                    continue;

                vertices[i] = new PathVertex(i);
            }

            var heapComparer = new HeapElementComparer();
            heap = new NativeHeap<HeapElement, HeapElementComparer>(Allocator.Temp, vertexCount, heapComparer);
            initialized = true;
        }

        internal delegate void EdgeProcessor(in PathEdge edge);

        internal readonly bool HasPredecessors(int vertexIndex, in SmartFindPathJob job)
        {
            for (var i = 0; i < vertexCount; i++)
            {
                ref var edge = ref GetEdge(i, vertexIndex);
                if (!edge.isValid)
                    continue;

                edge.CalculateCostIfMissing(in job);

                if (float.IsNaN(edge.cost))
                    continue;
                if (float.IsInfinity(edge.cost))
                    continue;

                return true;
            }

            return false;
        }

        internal readonly void CalculatePredecessors(int vertexIndex, in SmartFindPathJob job)
        {
            for (var i = 0; i < vertexCount; i++)
            {
                ref var edge = ref GetEdge(i, vertexIndex);
                if (!edge.isValid)
                    continue;

                edge.CalculateCostIfMissing(in job);
            }
        }

        internal readonly void ForEachPredecessor(int vertexIndex, EdgeProcessor processor)
        {
            for (var i = 0; i < vertexCount; i++)
            {
                ref var edge = ref GetEdge(i, vertexIndex);
                if (!edge.isValid)
                    continue;

                if (float.IsNaN(edge.cost))
                    continue;
                if (float.IsInfinity(edge.cost))
                    continue;

                processor(in edge);
            }
        }

        internal readonly bool HasSuccessors(int vertexIndex, in SmartFindPathJob job)
        {
            for (var i = 0; i < vertexCount; i++)
            {
                ref var edge = ref GetEdge(vertexIndex, i);
                if (!edge.isValid)
                    continue;

                edge.CalculateCostIfMissing(in job);

                if (float.IsNaN(edge.cost))
                    continue;
                if (float.IsInfinity(edge.cost))
                    continue;

                return true;
            }

            return false;
        }

        internal readonly void ForEachSuccessor(int vertexIndex, EdgeProcessor processor)
        {
            for (var i = 0; i < vertexCount; i++)
            {
                ref var edge = ref GetEdge(vertexIndex, i);
                if (!edge.isValid)
                    continue;

                if (float.IsNaN(edge.cost))
                    continue;
                if (float.IsInfinity(edge.cost))
                    continue;

                processor(in edge);
            }
        }

        internal readonly int GetBestSuccessor(int vertexIndex, out float cost)
        {
            var result = -1;
            cost = float.PositiveInfinity;

            for (var i = 0; i < vertexCount; i++)
            {
                ref var edge = ref GetEdge(vertexIndex, i);
                if (!edge.isValid)
                    continue;

                if (float.IsNaN(edge.cost))
                    continue;
                if (float.IsInfinity(edge.cost))
                    continue;

                ref var succVertex = ref GetVertex(edge.destination);
                var newCost = edge.cost + succVertex.g;

                if (newCost >= cost)
                    continue;

                cost = newCost;
                result = edge.destination;
            }

            return result;
        }

        internal readonly ref PathEdge GetEdge(int source, int destination)
        {
            return ref edges.GetRef(destination * vertexCount + source);
        }
        internal readonly ref PathVertex GetVertex(int vertexIndex)
        {
            return ref vertices.GetRef(vertexIndex);
        }

        public void Dispose()
        {
            if (!initialized)
                return;

            initialized = false;

            vertices.Dispose();
            edges.Dispose();
            heap.Dispose();
        }
    }

    private struct PathEdge(int sourceIndex, int destinationIndex)
    {
        internal bool isValid;
        internal readonly int source = sourceIndex;
        internal readonly int destination = destinationIndex;
        internal float cost = float.NaN;

        internal void CalculateCostIfMissing(in SmartFindPathJob job)
        {
            if (!isValid)
                return;
            if (!float.IsNaN(cost))
                return;

            var sourcePosition = job.GetVertexPosition(source);
            var destinationPosition = job.GetVertexPosition(destination);

            var traverseCost = job.CalculateSinglePath(sourcePosition, destinationPosition);

            if (traverseCost < MinEdgeCost)
                traverseCost = MinEdgeCost;

            cost = traverseCost;
        }

#if SMART_PATHFINDING_DEBUG
        public readonly override string ToString()
        {
            if (!isValid)
                return $"{source}->{destination} Invalid";
            return $"{source}->{destination} {nameof(cost)}:{cost:0.###}";
        }
#endif
    }

    private struct PathVertex : IDisposable
    {
        internal int index;
        internal NativeHeapIndex heapIndex;

        internal float heuristic;
        internal float g;
        internal float rhs;

        internal PathVertex(int index)
        {
            this.index = index;
            heapIndex = NativeHeapIndex.Invalid;

            heuristic = float.PositiveInfinity;
            g = float.PositiveInfinity;
            rhs = float.PositiveInfinity;
        }

        public void Dispose()
        {
            heuristic = float.NaN;
            g = float.NaN;
            rhs = float.NaN;
        }

        internal readonly record struct VertexKey(float key1, float key2) : IComparable<VertexKey>
        {
            private readonly float key1 = key1;
            private readonly float key2 = key2;

            public readonly int CompareTo(VertexKey other)
            {
                var key1Comparison = key1.CompareTo(other.key1);
                return key1Comparison != 0 ? key1Comparison : key2.CompareTo(other.key2);
            }

            public static bool operator <(VertexKey left, VertexKey right)
            {
                return left.CompareTo(right) < 0;
            }

            public static bool operator >(VertexKey left, VertexKey right)
            {
                return left.CompareTo(right) > 0;
            }

            public static bool operator <=(VertexKey left, VertexKey right)
            {
                return left.CompareTo(right) <= 0;
            }

            public static bool operator >=(VertexKey left, VertexKey right)
            {
                return left.CompareTo(right) >= 0;
            }

#if SMART_PATHFINDING_DEBUG
            public readonly override string ToString()
            {
                return $"[{key1:0.###}, {key2:0.###}]";
            }
#endif
        }

        public readonly VertexKey CalculateKey()
        {
            var key2 = Mathf.Min(g, rhs);
            var key1 = key2 + heuristic;

            return new VertexKey(key1, key2);
        }

#if SMART_PATHFINDING_DEBUG
        public readonly override string ToString()
        {
            return $"{nameof(index)}: {index}, {nameof(g)}: {g:0.###}, {nameof(rhs)}: {rhs:0.###}";
        }
#endif
    }

    private readonly record struct HeapElement(int index, PathVertex.VertexKey key)
    {
        internal readonly int index = index;
        internal readonly PathVertex.VertexKey key = key;
    }

    private readonly struct HeapElementComparer : IComparer<HeapElement>
    {
        public readonly int Compare(HeapElement x, HeapElement y)
        {
            return x.key.CompareTo(y.key);
        }
    }

#if SMART_PATHFINDING_DEBUG
    private void PrintCurrPath(PathfinderMemory memory, int goalIndex)
    {
        var currIndex = StartVertexIndex;
        var pathCost = 0f;
        var last_g = float.PositiveInfinity;
        var builder = new StringBuilder("Path:\n");

        while (true)
        {
            builder.AppendFormat(" - distance: {0:0.###}", pathCost);
            if (currIndex > GoalVertexIndex)
            {
                builder.AppendFormat(" ??? ({0})\n", currIndex);
                break;
            }
            else if (currIndex == GoalVertexIndex)
            {
                builder.AppendFormat(" Goal {0}\n", goals[goalIndex]);
                break;
            }
            else if (currIndex == StartVertexIndex)
            {
                builder.AppendFormat(" Start {0}\n", start);
            }
            else if (currIndex >= LinkDestinationsOffset)
            {
                builder.AppendFormat(" linkDestination {0}\n", linkDestinations[currIndex - LinkDestinationsOffset]);
            }
            else if (currIndex >= 0)
            {
                var linkName = "Unknown";
                if (currIndex < SmartPathJobDataContainer.linkNames.Count)
                    linkName = SmartPathJobDataContainer.linkNames[currIndex];
                builder.AppendFormat(" {0} {1}\n", linkName, linkOrigins[currIndex]);
            }
            else
            {
                builder.AppendFormat(" ??? ({0})\n", currIndex);
                break;
            }

            var bestIndex = memory.GetBestSuccessor(currIndex, out _);

            if (bestIndex == -1)
                break;

            pathCost += memory.GetEdge(currIndex, bestIndex).cost;
            currIndex = bestIndex;

            ref var vertex = ref memory.GetVertex(currIndex);

            if (vertex.g > last_g)
            {
                builder.AppendFormat(" Loop! ({0})\n", currIndex);
                break;
            }

            last_g = vertex.g;
        }

        PathfindingLibPlugin.Instance.Logger.LogInfo(builder.ToString());
    }

    private static DebugVertex[][] DebugVertices = [];

    private static bool captureNextVertices;

    private void InitVerticesCapture(int count)
    {
        DebugVertices = new DebugVertex[count][];
    }

    private void CaptureVerticesSnapshot(PathfinderMemory memory, int goal)
    {
        DebugVertices[goal] = new DebugVertex[vertexCount];
        for (var i = 0; i < vertexCount; i++)
        {
            var vertexType = "OutOfRange";
            if (i > GoalVertexIndex)
                vertexType = "OutOfRange";
            else if (i == GoalVertexIndex)
                vertexType = "Goal";
            else if (i == StartVertexIndex)
                vertexType = "Start";
            else if (i >= LinkDestinationsOffset)
                vertexType = "LinkDestination";
            else if (i >= 0)
                vertexType = "LinkOrigin";

            DebugVertices[goal][i] = new DebugVertex(memory, i, vertexType);
        }
    }

    private readonly struct DebugVertex
    {
        internal readonly bool valid;

        internal readonly List<PathEdge> predecessors;
        internal readonly List<PathEdge> successors;

        internal readonly int index;
        internal readonly string type;

        internal readonly float heuristic;
        internal readonly float g;
        internal readonly float rhs;

        internal readonly PathVertex.VertexKey key;
        internal readonly PathEdge NextEdge;

        internal DebugVertex(PathfinderMemory memory, int index, string type)
        {
            ref var src = ref memory.GetVertex(index);

            this.index = src.index;

            this.type = type;

            predecessors = [];
            successors = [];

            for (var i = 0; i < memory.vertexCount; i++)
            {
                var predecessorEdge = memory.GetEdge(i, index);
                if (predecessorEdge.isValid)
                    predecessors.Add(predecessorEdge);

                var successorEdge = memory.GetEdge(index, i);
                if (successorEdge.isValid)
                    successors.Add(successorEdge);
            }

            heuristic = src.heuristic;
            g = src.g;
            rhs = src.rhs;

            key = src.CalculateKey();

            var bestIndex = memory.GetBestSuccessor(index, out var cost);

            NextEdge = new PathEdge(index, bestIndex)
            {
                isValid = true,
                cost = cost,
            };
            valid = true;
        }

        public readonly override string ToString()
        {
            if (!valid)
                return "Not Initialized!";
            return $"{type} {nameof(index)}: {index}, {nameof(g)}: {g:0.###}, {nameof(rhs)}: {rhs:0.###}";
        }
    }
#endif

    internal void FreeAllResources()
    {
        DisposeResizeableArrays();
    }
}
