#define SMART_PATHFINDING_DEBUG

using System;
using System.Collections.Generic;
using System.Diagnostics;

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Experimental.AI;

using PathfindingLib.API;
using PathfindingLib.API.SmartPathfinding;
using PathfindingLib.Utilities;
using PathfindingLib.Utilities.Collections;

#if BENCHMARKING
using Unity.Profiling;
#endif

#if SMART_PATHFINDING_DEBUG
using System.Text;
#endif

namespace PathfindingLib.Jobs;

public struct SmartFindPathJob : IJob
{
    private const int MaxPathsToTest = 30;
    private const float MinEdgeCost = 0.0001f;

    [ReadOnly, NativeDisableContainerSafetyRestriction] private NativeArray<NavMeshQuery> ThreadQueriesRef;

    [ReadOnly] private int agentTypeID;
    [ReadOnly] private int areaMask;
    [ReadOnly] private Vector3 start;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<Vector3> goals;
    [ReadOnly] private int goalCount;

    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<Vector3> linkOrigins;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<IndexAndSize> linkDestinationSlices;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<Vector3> linkDestinations;
    [ReadOnly] private int linkCount;
    [ReadOnly] private int linkDestinationCount;

    [ReadOnly] private int vertexCount;

    [ReadOnly, NativeSetThreadIndex] private int threadIndex;

    [WriteOnly] internal NativeArray<int> firstNodeIndices;
    [WriteOnly] internal NativeArray<float> pathLengths;

    private void Initialize(SmartPathJobDataContainer data, Vector3 origin, Vector3[] destinations, int destinationCount, NavMeshAgent agent)
    {
        ThreadQueriesRef = PathfindingJobSharedResources.GetPerThreadQueriesArray();

        agentTypeID = agent.agentTypeID;
        areaMask = agent.areaMask;
        start = origin;

        // Shhhh, compiler...
        threadIndex = -1;

        linkOrigins = data.linkOrigins;
        linkDestinationSlices = data.linkDestinationSlices;
        linkDestinations = data.linkDestinations;

        linkCount = data.linkCount;
        linkDestinationCount = data.linkDestinationCount;

        EnsureDestinationCount(destinationCount);
        goalCount = destinationCount;

        NativeArray<Vector3>.Copy(destinations, goals, destinationCount);
        firstNodeIndices.SetAllElements(-1);
        pathLengths.SetAllElements(float.PositiveInfinity);

        vertexCount = linkCount + destinationCount + linkDestinationCount + 1;
    }

    internal void Initialize(SmartPathJobDataContainer data, Vector3 origin, Vector3[] destinations, NavMeshAgent agent)
    {
        Initialize(data, origin, destinations, destinations.Length, agent);
    }

    internal void Initialize(SmartPathJobDataContainer data, Vector3 origin, List<Vector3> destinations, NavMeshAgent agent)
    {
        Initialize(data, origin, NoAllocHelpers.ExtractArrayFromListT(destinations), destinations.Count, agent);
    }

    private void EnsureDestinationCount(int count)
    {
        if (goals.Length >= count)
            return;

        DisposeResizeableArrays();

        goals = new NativeArray<Vector3>(count, Allocator.Persistent);
        firstNodeIndices = new NativeArray<int>(count, Allocator.Persistent);
        pathLengths = new NativeArray<float>(count, Allocator.Persistent);
    }

    private void DisposeResizeableArrays()
    {
        if (!goals.IsCreated)
            return;

        goals.Dispose();
        firstNodeIndices.Dispose();
        pathLengths.Dispose();
    }

#if BENCHMARKING
    private static readonly ProfilerMarker CalculateSinglePathMarker = new("CalculateSinglePath");
#endif

    // The path vertices are made up of:
    // - [linkCount]: Link origins
    // - [goalCount]: Goals
    // - [linkDestinationCount]: Link destinations
    // - [1]: Start
    private readonly int GoalsOffset => linkCount;
    private readonly int LinkDestinationsOffset => linkCount + goalCount;
    private readonly int StartIndex => vertexCount - 1;

    private float CalculateSinglePath(Vector3 origin, Vector3 destination)
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

    public void Execute()
    {
#if SMART_PATHFINDING_DEBUG
        if (captureNextVertices)
            InitVerticesCapture(goalCount);
#endif

        var jobStopwatch = Stopwatch.StartNew();
        var goalStopwatch = new Stopwatch();

        var memory = new PathfinderMemory(vertexCount);
        PopulateVertices(memory);

        if (!memory.HasSuccessors(StartIndex, in this))
        {
#if SMART_PATHFINDING_DEBUG
            if (captureNextVertices)
            {
                InitVerticesCapture(1);
                CaptureVerticesSnapshot(memory, 0);
            }

            PathfindingLibPlugin.Instance.Logger.LogInfo($"Path failed in {jobStopwatch.Elapsed.TotalMilliseconds}ms, start position is isolated.");
#endif
            memory.Dispose();
            return;
        }

        for (var goalIndex = 0; goalIndex < goalCount; goalIndex++)
        {
            var goalVertexIndex = GoalsOffset + goalIndex;

            if (!memory.HasPredecessors(goalVertexIndex, in this))
            {
#if SMART_PATHFINDING_DEBUG
                if (captureNextVertices)
                    CaptureVerticesSnapshot(memory, goalIndex);

                PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed, goal position is isolated.");
#endif
                continue;
            }

            goalStopwatch.Restart();

            var result = CalculatePath(memory, goalVertexIndex, out var pathLength);

            goalStopwatch.Stop();

            PathfindingLibPlugin.Instance.Logger.LogInfo($"Path to Goal{goalIndex} took {goalStopwatch.Elapsed.TotalMilliseconds}ms");

#if SMART_PATHFINDING_DEBUG
            if (captureNextVertices)
                CaptureVerticesSnapshot(memory, goalIndex);

            PrintCurrPath(memory);

            if (result == -1)
                PathfindingLibPlugin.Instance.Logger.LogInfo($"Path for goal {goalIndex} failed.");
            else if (result == linkCount)
                PathfindingLibPlugin.Instance.Logger.LogInfo($"Path for goal {goalIndex} was a direct path with length {pathLength}.");
            else
                PathfindingLibPlugin.Instance.Logger.LogInfo($"Path for goal {goalIndex} was an indirect path through link index {result} ({SmartPathJobDataContainer.linkNames[result]}).");
#endif

            firstNodeIndices[goalIndex] = result;
            pathLengths[goalIndex] = pathLength;
        }

        memory.Dispose();

        PathfindingLibPlugin.Instance.Logger.LogInfo($"Job took {jobStopwatch.Elapsed.TotalMilliseconds}ms.");

#if SMART_PATHFINDING_DEBUG
        captureNextVertices = false;
#endif
    }

    private Vector3 GetVertexPosition(int vertexIndex)
    {
        if (vertexIndex > StartIndex)
            throw new IndexOutOfRangeException($"Index {vertexIndex} is out of range.");

        if (vertexIndex == StartIndex)
            return start;

        if (vertexIndex >= LinkDestinationsOffset)
            return linkDestinations[vertexIndex - LinkDestinationsOffset];

        if (vertexIndex >= GoalsOffset)
            return goals[vertexIndex - GoalsOffset];

        if (vertexIndex >= 0)
            return linkOrigins[vertexIndex];

        throw new IndexOutOfRangeException("Index cannot be negative.");
    }

    private void PopulateVertices(PathfinderMemory memory)
    {
        for (var i = 0; i < vertexCount; i++)
        {
            ref var vertex = ref memory.GetVertex(i);
            var vertexPosition = GetVertexPosition(i);

            if (i >= LinkDestinationsOffset)
            {
                // Connect the start and all link destinations to all links' origins...
                vertex.heuristic = (vertexPosition - start).magnitude;

                for (var j = 0; j < LinkDestinationsOffset; j++)
                {
                    ref var edge = ref memory.GetEdge(i, j);

                    edge.isValid = true;
                }
            }
            else if (i >= GoalsOffset)
            {
                var goalIndex = i - GoalsOffset;
                vertex.heuristic = (goals[goalIndex] - start).magnitude;
            }
            else if (i >= 0)
            {
                // Connect all link origins to their destinations.
                vertex.heuristic = (vertexPosition - start).magnitude;

                var slice = linkDestinationSlices[i];

                for (var j = slice.index; j < slice.index + slice.size; j++)
                {
                    var destIndex = LinkDestinationsOffset + j;
                    ref var edge = ref memory.GetEdge(i, destIndex);

                    edge.isValid = true;

                    var traverseCost = 0.0001f;

                    if (traverseCost <= MinEdgeCost)
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

    private readonly int CalculatePath(PathfinderMemory memory, int goalIndex, out float totalPathLength)
    {
        var heap = memory.heap;
        for (var i = 0; i < vertexCount; i++)
        {
            ref var vertex = ref memory.GetVertex(i);
            vertex.g = float.PositiveInfinity;
            vertex.rhs = float.PositiveInfinity;
        }

        ref var startVertex = ref memory.GetVertex(StartIndex);
        ref var goalVertex = ref memory.GetVertex(goalIndex);

        goalVertex.rhs = 0;

        heap.Clear();
        goalVertex.heapIndex = heap.Insert(new HeapElement(goalIndex, goalVertex.CalculateKey()));

        var iterations = 0;
        var extraIterations = 0;

        void UpdateOrAddVertex(ref PathVertex vertex)
        {
            if (vertex.heapIndex.IsValid)
            {
                heap.Remove(vertex.heapIndex);
                vertex.heapIndex = NativeHeapIndex.Invalid;
            }
            if (!Mathf.Approximately(vertex.g, vertex.rhs))
                vertex.heapIndex = heap.Insert(new HeapElement(vertex.index, vertex.CalculateKey()));
        }

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

                    if (edge.source != goalIndex)
                    {
                        var newRhs = edge.cost + memory.GetVertex(index).g;

                        if (newRhs < predecessorVertex.rhs)
                            predecessorVertex.rhs = newRhs;
                    }

                    UpdateOrAddVertex(ref predecessorVertex);
                });
            }
            else
            {
                // The cost has increased, so we have to update the vertex's cost as well as its predecessors'.
                // This appears to be unreachable given that we aren't changing the costs outside of the pathfinder.
                var gOld = currentVertex.g;
                currentVertex.g = float.PositiveInfinity;

                memory.ForEachPredecessor(index, (in PathEdge edge) =>
                {
                    RecalculateRHS(edge.source, edge.cost);
                });

                RecalculateRHS(index, 0);

                void RecalculateRHS(int vertex, float successorCost)
                {
                    ref var predVertex = ref memory.GetVertex(vertex);
                    if (vertex != goalIndex)
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

                    UpdateOrAddVertex(ref predVertex);
                }
            }
        }

#if SMART_PATHFINDING_DEBUG
        PathfindingLibPlugin.Instance.Logger.LogDebug($"Found path after {iterations - extraIterations} iterations, then searched for an extra {extraIterations} iterations");
#endif

        totalPathLength = float.PositiveInfinity;

        // No path was found.
        if (float.IsInfinity(startVertex.rhs))
            return -1;

        var bestIndex = memory.GetBestSuccessor(StartIndex, out totalPathLength);

        // Return one greater than the possible nodes if we've gotten a direct path to the destination.
        if (bestIndex == goalIndex)
            return linkCount;

        if (bestIndex < linkCount)
            return bestIndex;

        PathfindingLibPlugin.Instance.Logger.LogFatal($"Smart pathfinding job found a best path that was a link destination. This should not be possible. Index = {bestIndex}, link count = {linkCount}");
        return -1;
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
                var row = i / vertexCount;
                var column = i % vertexCount;
                edges[i] = new PathEdge(row, column);

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

        internal readonly ref PathEdge GetEdge(int start, int destination)
        {
            return ref edges.GetRef(start * vertexCount + destination);
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

            if (traverseCost <= MinEdgeCost)
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
    private void PrintCurrPath(PathfinderMemory memory)
    {
        var currIndex = StartIndex;
        var pathCost = 0f;
        var builder = new StringBuilder("Path:\n");

        while (true)
        {
            ref var currVertex = ref memory.GetVertex(currIndex);

            builder.AppendFormat(" - distance: {0:0.###}", pathCost);
            if (currIndex > StartIndex)
            {
                builder.AppendFormat(" ??? ({0})\n", currIndex);
                break;
            }
            else if (currIndex == StartIndex)
            {
                builder.AppendFormat(" Start {0}\n", start);
            }
            else if (currIndex >= LinkDestinationsOffset)
            {
                builder.AppendFormat(" linkDestination {0}\n", linkDestinations[currIndex - linkCount]);
            }
            else if (currIndex >= GoalsOffset)
            {
                builder.AppendFormat(" Goal {0}\n", start);
                break;
            }
            else if (currIndex >= 0)
            {
                builder.AppendFormat(" {0}\n", SmartPathJobDataContainer.linkNames[currIndex]);
            }
            else
            {
                builder.AppendFormat(" ??? ({0})\n", currIndex);
                break;
            }

            var bestIndex = memory.GetBestSuccessor(currIndex, out var cost);

            if (bestIndex == -1)
                break;

            pathCost += cost;
            currIndex = bestIndex;
        }

        PathfindingLibPlugin.Instance.Logger.LogDebug(builder.ToString());
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
            if (i > StartIndex)
                vertexType = "OutOfRange";
            else if (i == StartIndex)
                vertexType = "Start";
            else if (i >= LinkDestinationsOffset)
                vertexType = "LinkDestination";
            else if (i >= GoalsOffset)
                vertexType = "Goal";
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
