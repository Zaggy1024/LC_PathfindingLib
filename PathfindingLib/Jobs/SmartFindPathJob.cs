#define SMART_PATHFINDING_DEBUG

using System;
using System.Collections.Generic;
using System.Diagnostics;

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
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
        query.GetPathResult(pathNodes);

        using var path = new NativeArray<NavMeshLocation>(NavMeshQueryUtils.RecommendedCornerCount, Allocator.Temp);
        var straightPathStatus = NavMeshQueryUtils.FindStraightPath(query, origin, destination, pathNodes, pathNodesSize, path, out var pathSize);
        pathNodes.Dispose();

        // ReSharper disable once DisposeOnUsingVariable
        readLocker.Dispose();

        if (straightPathStatus.GetResult() != PathQueryStatus.Success)
            return float.PositiveInfinity;

        // Check if the end of the path is close enough to the target.
        var pathEnd = path[pathSize - 1].position;
        var endDistance = (pathEnd - destination).sqrMagnitude;
        if (endDistance > SharedJobValues.MaximumEndpointDistanceSquared)
            return float.PositiveInfinity;

        var distance = 0f;
        for (var i = 1; i < pathSize; i++)
            distance += Vector3.Distance(path[i - 1].position, path[i].position);

        return distance;
    }

    private record struct PathEdge(int sourceIndex, int destIndex, float cost)
    {
        internal int source = sourceIndex;
        internal int dest = destIndex;
        internal float cost = cost;

#if SMART_PATHFINDING_DEBUG
        public readonly override string ToString()
        {
            return $"{source}->{dest} {nameof(cost)}:{cost:0.###}";
        }
#endif
    }

    private struct PathVertex : IDisposable
    {
        internal UnsafeList<PathEdge> pred;
        internal UnsafeList<PathEdge> succ;

        internal int index;
        internal NativeHeapIndex heapIndex;

        internal float heuristic;
        internal float g;
        internal float rhs;

        internal PathVertex(int index, int maxCount)
        {
            pred = new UnsafeList<PathEdge>(maxCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            succ = new UnsafeList<PathEdge>(maxCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            this.index = index;
            heapIndex = NativeHeapIndex.Invalid;

            heuristic = float.PositiveInfinity;
            g = float.PositiveInfinity;
            rhs = float.PositiveInfinity;
        }

        public void Dispose()
        {
            pred.Dispose();
            pred = default;
            succ.Dispose();
            succ = default;

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

    private readonly struct HeapElementComparer : IComparer<HeapElement>
    {
        public readonly int Compare(HeapElement x, HeapElement y)
        {
            return x.key.CompareTo(y.key);
        }
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

    private void PopulateVertices(NativeArray<PathVertex> pathVertices)
    {
        for (var i = 0; i < vertexCount; i++){
            pathVertices[i] = new PathVertex(i, vertexCount);
        }

        for (var i = 0; i < vertexCount; i++)
        {
            ref var vertex = ref pathVertices.GetRef(i);
            var vertexPosition = GetVertexPosition(i);

            if (i >= LinkDestinationsOffset)
            {
                // Connect the start and all link destinations to all links' origins...
                vertex.heuristic = (vertexPosition - start).magnitude;

                for (var j = 0; j < LinkDestinationsOffset; j++)
                {
                    var originPos = GetVertexPosition(j);
                    ref var originVertex = ref pathVertices.GetRef(j);

                    var cost = CalculateSinglePath(vertexPosition, originPos);

                    if (float.IsInfinity(cost))
                        continue;

                    originVertex.pred.Add(new PathEdge(i, j, cost));
                    vertex.succ.Add(new PathEdge(i,j, cost));
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

                    var traverseCost = 0.0001f;

                    ref var destinationVertex = ref pathVertices.GetRef(destIndex);

                    destinationVertex.pred.Add(new PathEdge(i, destIndex, traverseCost));
                    vertex.succ.Add(new PathEdge(i, destIndex, traverseCost));
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

    private readonly record struct HeapElement(int index, PathVertex.VertexKey key)
    {
        internal readonly int index = index;
        internal readonly PathVertex.VertexKey key = key;
    }

    private readonly int CalculatePath(NativeArray<PathVertex> pathVertices, int goalIndex, out float totalPathLength)
    {
        for (var i = 0; i < vertexCount; i++)
        {
            ref var vertex = ref pathVertices.GetRef(i);
            vertex.g = float.PositiveInfinity;
            vertex.rhs = float.PositiveInfinity;
        }

        ref var startVertex = ref pathVertices.GetRef(StartIndex);
        ref var goalVertex = ref pathVertices.GetRef(goalIndex);

        goalVertex.rhs = 0;
        var heapComparer = new HeapElementComparer();

        var heap = new NativeHeap<HeapElement, HeapElementComparer>(Allocator.Temp, vertexCount, heapComparer);
        goalVertex.heapIndex = heap.Insert(new(goalIndex, goalVertex.CalculateKey()));

        var iterations = 0;
        var extraIterations = 0;

        void UpdateOrAddVertex(ref PathVertex vertex)
        {
            if (vertex.heapIndex.IsValid)
            {
                var removed = heap.Remove(vertex.heapIndex);
                vertex.heapIndex = NativeHeapIndex.Invalid;
            }
            if (!Mathf.Approximately(vertex.g, vertex.rhs))
                vertex.heapIndex = heap.Insert(new(vertex.index, vertex.CalculateKey()));
        }

        while (heap.Count > 0)
        {
            iterations += 1;
            var (index, oldKey) = heap.Peek();
            ref var currentVertex = ref pathVertices.GetRef(index);

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

                foreach (var (predIndex, _, cost) in currentVertex.pred)
                {
                    ref var predVertex = ref pathVertices.GetRef(predIndex);

                    if (predIndex != goalIndex)
                    {
                        var newRhs = cost + currentVertex.g;

                        if (newRhs < predVertex.rhs)
                            predVertex.rhs = newRhs;
                    }

                    UpdateOrAddVertex(ref predVertex);
                }
            }
            else
            {
                // The cost has increased, so we have to update the vertex's cost as well as its predecessors'.
                // This appears to be unreachable given that we aren't changing the costs outside of the pathfinder.
                var gOld = currentVertex.g;
                currentVertex.g = float.PositiveInfinity;

                void RecalculateRHS(int predIndex, float predCost)
                {
                    ref var predVertex = ref pathVertices.GetRef(predIndex);
                    if (predIndex != goalIndex)
                    {
                        if (Mathf.Approximately(predVertex.rhs, predCost + gOld))
                        {
                            var minRhs = float.PositiveInfinity;

                            foreach (var (_, succIndex, succCost) in predVertex.succ)
                            {
                                ref var succVertex = ref pathVertices.GetRef(succIndex);

                                var newRhs = succCost + succVertex.g;

                                if (newRhs < minRhs)
                                    minRhs = newRhs;
                            }

                            predVertex.rhs = minRhs;
                        }
                    }

                    UpdateOrAddVertex(ref predVertex);
                }

                foreach (var (predIndex, _, predCost) in currentVertex.pred)
                    RecalculateRHS(predIndex, predCost);

                RecalculateRHS(index, 0);
            }
        }

#if SMART_PATHFINDING_DEBUG
        PathfindingLibPlugin.Instance.Logger.LogDebug($"Found path after {iterations - extraIterations} iterations, then searched for an extra {extraIterations} iterations");
#endif

        totalPathLength = float.PositiveInfinity;

        // No path was found.
        if (float.IsInfinity(startVertex.rhs))
            return -1;

        var bestIndex = -1;

        foreach (var (_, succIndex, cost) in startVertex.succ)
        {
            ref var succVertex = ref pathVertices.GetRef(succIndex);
            var newCost = cost + succVertex.g;

            if (newCost >= totalPathLength)
                continue;

            totalPathLength = newCost;
            bestIndex = succIndex;
        }

        // Return one greater than the possible nodes if we've gotten a direct path to the destination.
        if (bestIndex == goalIndex)
            return linkCount;

        if (bestIndex < linkCount)
            return bestIndex;

        PathfindingLibPlugin.Instance.Logger.LogFatal($"Smart pathfinding job found a best path that was a link destination. This should not be possible. Index = {bestIndex}, link count = {linkCount}");
        return -1;
    }

    private void DisposeVertices(ref NativeArray<PathVertex> vertices)
    {
        if (vertices == default)
            return;

        foreach (var vertex in vertices)
            vertex.Dispose();

        vertices.Dispose();

        vertices = default;
    }

    public void Execute()
    {
#if SMART_PATHFINDING_DEBUG
        if (captureNextVertices)
            InitVerticesCapture();
#endif

        var stopwatch = Stopwatch.StartNew();

        var pathVertices = new NativeArray<PathVertex>(vertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

        PopulateVertices(pathVertices);

        ref var startVertex = ref pathVertices.GetRef(StartIndex);
        if (startVertex.succ.Length == 0)
        {
#if SMART_PATHFINDING_DEBUG
            PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed, start position is isolated.");
#endif
            return;
        }

        for (var goalIndex = 0; goalIndex < goalCount; goalIndex++)
        {
            var goalVertexIndex = GoalsOffset + goalIndex;
            ref var goalVertex = ref pathVertices.GetRef(goalVertexIndex);

            if (goalVertex.pred.Length == 0)
            {
#if SMART_PATHFINDING_DEBUG
                if (captureNextVertices)
                    CaptureVerticesSnapshot(ref pathVertices, goalIndex);

                PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed, goal position is isolated");
#endif
                continue;
            }

            var result = CalculatePath(pathVertices, goalVertexIndex, out var pathLength);

#if SMART_PATHFINDING_DEBUG
            if (captureNextVertices)
                CaptureVerticesSnapshot(ref pathVertices, goalIndex);

            PrintCurrPath(pathVertices);

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

        DisposeVertices(ref pathVertices);

        PathfindingLibPlugin.Instance.Logger.LogInfo($"Path took {stopwatch.Elapsed.TotalMilliseconds}ms");

#if SMART_PATHFINDING_DEBUG
        captureNextVertices = false;
#endif
    }

#if SMART_PATHFINDING_DEBUG
    private void PrintCurrPath(NativeArray<PathVertex> pathVertices)
    {
        var currIndex = StartIndex;
        var builder = new StringBuilder("Path:\n");

        while (true)
        {
            ref var currVertex = ref pathVertices.GetRef(currIndex);

            builder.AppendFormat(" - distance: {0:0.###}", currVertex.g);
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

            var bestIndex = -1;
            var minCost = float.PositiveInfinity;

            foreach (var (_, succIndex, cost) in currVertex.succ)
            {
                ref var succVertex = ref pathVertices.GetRef(succIndex);
                var newCost = cost + succVertex.g;

                if (newCost >= minCost)
                    continue;

                minCost = newCost;
                bestIndex = succIndex;
            }

            if (bestIndex == -1)
                break;

            currIndex = bestIndex;
        }

        PathfindingLibPlugin.Instance.Logger.LogDebug(builder.ToString());
    }

    private static DebugVertex[][] DebugVertices = [];

    private static bool captureNextVertices;

    private void InitVerticesCapture()
    {
        DebugVertices = new DebugVertex[goalCount][];
    }

    private void CaptureVerticesSnapshot(ref NativeArray<PathVertex> vertices, int goal)
    {
        DebugVertices[goal] = new DebugVertex[vertexCount];
        for (var i = 0; i < vertexCount; i++)
        {
            var vertexType = "OutOfRange";
            if (i > StartIndex) {}
            else if (i == StartIndex)
                vertexType = "Start";
            else if (i >= LinkDestinationsOffset)
                vertexType = "LinkDestination";
            else if (i >= GoalsOffset)
                vertexType = "Goal";
            else if (i >= 0)
                vertexType = "LinkOrigin";

            DebugVertices[goal][i] = new DebugVertex(ref vertices, i, vertexType);
        }
    }

    private readonly struct DebugVertex
    {
        internal readonly bool valid;

        internal readonly PathEdge[] pred;
        internal readonly PathEdge[] succ;

        internal readonly int index;
        internal readonly string type;

        internal readonly float heuristic;
        internal readonly float g;
        internal readonly float rhs;

        internal readonly PathVertex.VertexKey key;
        internal readonly PathEdge NextEdge;

        internal DebugVertex(ref NativeArray<PathVertex> vertices, int index, string type)
        {
            ref var src = ref vertices.GetRef(index);

            this.index = src.index;

            this.type = type;

            var oldList = src.pred;
            pred = new PathEdge[oldList.Length];
            for (var i = 0; i < oldList.Length; i++)
            {
                pred[i] = oldList[i];
            }
            oldList = src.succ;
            succ = new PathEdge[oldList.Length];
            for (var i = 0; i < oldList.Length; i++)
            {
                succ[i] = oldList[i];
            }

            heuristic = src.heuristic;
            g = src.g;
            rhs = src.rhs;

            key = src.CalculateKey();

            var bestIndex = -1;
            var minCost = float.PositiveInfinity;

            foreach (var (_, succIndex, cost) in succ)
            {
                ref var succVertex = ref vertices.GetRef(succIndex);
                var newCost = cost + succVertex.g;

                if (newCost >= minCost)
                    continue;

                minCost = newCost;
                bestIndex = succIndex;
            }

            NextEdge = new PathEdge(index, bestIndex, minCost);
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
