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

public struct SmartFindPathJob : IJob, IDisposable
{
    private const int MaxExtraIterations = 30;

    [ReadOnly, NativeDisableContainerSafetyRestriction] private NativeArray<NavMeshQuery> ThreadQueriesRef;

    [ReadOnly] private int agentTypeID;
    [ReadOnly] private int areaMask;
    [ReadOnly] private Vector3 start;
    [ReadOnly] private Vector3 goal;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<Vector3> linkOrigins;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<IndexAndSize> linkDestinationSlices;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<Vector3> linkDestinations;
    [ReadOnly] private int linkCount;
    [ReadOnly] private int destinationCount;

    [ReadOnly] private int vertexCount;
    [ReadOnly] private int startIndex;
    [ReadOnly] private int goalIndex;

    [ReadOnly, NativeSetThreadIndex] private int threadIndex;

    [WriteOnly] internal NativeArray<int> destinationIndex;

    internal void Initialize(SmartPathJobDataContainer data, Vector3 origin, Vector3 destination, NavMeshAgent agent)
    {
        ThreadQueriesRef = PathfindingJobSharedResources.GetPerThreadQueriesArray();

        agentTypeID = agent.agentTypeID;
        areaMask = agent.areaMask;
        start = origin;
        goal = destination;

        // Shhhh, compiler...
        threadIndex = -1;

        linkOrigins = data.linkOrigins;
        linkDestinationSlices = data.linkDestinationSlices;
        linkDestinations = data.linkDestinations;

        linkCount = data.linkCount;
        destinationCount = data.destinationCount;

        vertexCount = linkCount + destinationCount + 2;
        startIndex = vertexCount - 2;
        goalIndex = vertexCount - 1;

        destinationIndex = new NativeArray<int>(1, Allocator.Persistent);
        destinationIndex[0] = -1;
    }

#if BENCHMARKING
    private static readonly ProfilerMarker CalculateSinglePathMarker = new("CalculateSinglePath");
#endif

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

    private record struct PathLink(int index, float cost)
    {
        internal int index = index;
        internal float cost = cost;

#if SMART_PATHFINDING_DEBUG
        public readonly override string ToString()
        {
            return $"{index} {nameof(cost)}:{cost:0.###}";
        }
#endif
    }

    private struct PathVertex : IDisposable
    {
        internal UnsafeList<PathLink> pred;
        internal UnsafeList<PathLink> succ;

        internal readonly int index;

        internal float heuristic;
        internal float g;
        internal float rhs;

        internal PathVertex(int index, int verticesCount)
        {
            pred = new UnsafeList<PathLink>(verticesCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            succ = new UnsafeList<PathLink>(verticesCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            this.index = index;

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
        if (vertexIndex < 0)
            throw new IndexOutOfRangeException("Index cannot be negative.");

        if (vertexIndex < linkCount)
            return linkOrigins[vertexIndex];

        if (vertexIndex < linkCount + destinationCount)
            return linkDestinations[vertexIndex - linkCount];

        if (vertexIndex == startIndex)
            return start;

        if (vertexIndex == goalIndex)
            return goal;

        throw new IndexOutOfRangeException($"Index {vertexIndex} is out of range.");
    }

    private void PopulateVertices(NativeArray<PathVertex> pathVertices)
    {
        for (var i = 0; i < vertexCount; i++)
            pathVertices[i] = new(i, vertexCount);

        ref var goalVertex = ref pathVertices.GetRef(goalIndex);

        for (var i = 0; i < vertexCount; i++)
        {
            ref var vertex = ref pathVertices.GetRef(i);
            var vertexPosition = GetVertexPosition(i);

            if (i < linkCount) //if this index is an origin
            {
                vertex.heuristic = (vertexPosition - start).magnitude;

                //connect all its destinations

                var slice = linkDestinationSlices[i];

                for (var j = slice.index; j < slice.index + slice.size; j++)
                {
                    var destIndex = linkCount + j;

                    //TODO add costs! it's important the cost is never 0 or less
                    var traverseCost = 0.0001f;

                    ref var destinationVertex = ref pathVertices.GetRef(destIndex);

                    destinationVertex.pred.Add(new PathLink(i, traverseCost));
                    vertex.succ.Add(new PathLink(destIndex, traverseCost));
                }

            }
            else if (i < linkCount + destinationCount) //if this is a destination
            {
                vertex.heuristic = (vertexPosition - start).magnitude;

                //try path to all origins

                for (var j = 0; j < linkCount; j++)
                {
                    var originPos = GetVertexPosition(j);
                    ref var originVertex = ref pathVertices.GetRef(j);

                    //try connect
                    var cost = CalculateSinglePath(vertexPosition, originPos);

                    //if there is a valid path
                    if (float.IsInfinity(cost))
                        continue;

                    originVertex.pred.Add(new PathLink(i, cost));
                    vertex.succ.Add(new PathLink(j, cost));
                }

                //try path to our goal
                var goalCost = CalculateSinglePath(vertexPosition, goal);

                //if there is a valid path
                if (float.IsInfinity(goalCost))
                    continue;

                goalVertex.pred.Add(new PathLink(i, goalCost));
                vertex.succ.Add(new PathLink(goalIndex, goalCost));

            }
            else if (i == startIndex) //if this index is our start point
            {
                vertex.heuristic = 0;

                //try path to all origins

                for (var j = 0; j < linkCount; j++)
                {
                    var originPos = GetVertexPosition(j);
                    ref var originVertex = ref pathVertices.GetRef(j);

                    //try connect
                    var cost = CalculateSinglePath(start, originPos);

                    //if there is a valid path
                    if (float.IsInfinity(cost))
                        continue;

                    originVertex.pred.Add(new PathLink(i, cost));
                    vertex.succ.Add(new PathLink(j, cost));
                }

                //try path directly to our goal
                var goalCost = CalculateSinglePath(start, goal);

                //if there is a valid path
                if (float.IsInfinity(goalCost))
                    continue;

                goalVertex.pred.Add(new PathLink(i, goalCost));
                vertex.succ.Add(new PathLink(goalIndex, goalCost));
            }
            else if (i == goalIndex) //if this index is our goal
            {
                vertex.heuristic = (goal - start).magnitude;
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

    private readonly int CalculatePath(NativeArray<PathVertex> pathVertices)
    {
        ref var startVertex = ref pathVertices.GetRef(startIndex);
        ref var goalVertex = ref pathVertices.GetRef(goalIndex);

        goalVertex.rhs = 0;
        var heapComparer = new HeapElementComparer();
        var heap = new UnsafeList<HeapElement>(vertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory)
        {
            new(goalIndex, goalVertex.CalculateKey()),
        };

        var iterations = 0;
        var extraIterations = 0;

        while (heap.Length > 0)
        {
            iterations += 1;
            var (index, oldKey) = heap[0];
            ref var currentVertex = ref pathVertices.GetRef(index);

            //if we have found a path
            if (oldKey >= startVertex.CalculateKey() && startVertex.rhs <= startVertex.g)
                //try to compute a few extra times in hope of finding a better path
                if (extraIterations++ >= MaxExtraIterations)
                    break;

            var newKey = currentVertex.CalculateKey();

            if (oldKey < newKey)
            {
                //update heap
                heap.RemoveAt(0);
                heap.AddOrdered(new(index, newKey), heapComparer);
            }
            else if (currentVertex.g > currentVertex.rhs)
            {
                currentVertex.g = currentVertex.rhs;
                heap.RemoveAt(0);
                foreach (var (predIndex, cost) in currentVertex.pred)
                {
                    ref var predVertex = ref pathVertices.GetRef(predIndex);

                    if (predIndex != goalIndex)
                    {

                        var newRhs = cost + currentVertex.g;

                        if (newRhs < predVertex.rhs)
                            predVertex.rhs = newRhs;
                    }

                    UpdateVertex(ref predVertex);
                }
            }
            else
            {
                // Path is dead end
                var gOld = currentVertex.g;
                currentVertex.g = float.PositiveInfinity;

                void RecalculateRHS(int goalIndex, int predIndex, float predCost)
                {
                    ref var predVertex = ref pathVertices.GetRef(predIndex);
                    if (predIndex != goalIndex)
                    {

                        if (Mathf.Approximately(predVertex.rhs, predCost + gOld))
                        {
                            var minRhs = float.PositiveInfinity;

                            foreach (var (succIndex, succCost) in predVertex.succ)
                            {
                                ref var succVertex = ref pathVertices.GetRef(succIndex);

                                var newRhs = succCost + succVertex.g;

                                if (newRhs < minRhs)
                                    minRhs = newRhs;
                            }

                            predVertex.rhs = minRhs;
                        }
                    }

                    UpdateVertex(ref predVertex);
                }

                foreach (var (predIndex, predCost) in currentVertex.pred)
                    RecalculateRHS(goalIndex, predIndex, predCost);

                RecalculateRHS(goalIndex, index, 0);
            }
        }

#if SMART_PATHFINDING_DEBUG
        PathfindingLibPlugin.Instance.Logger.LogDebug($"Found path after {iterations - extraIterations} iterations, then searched for an extra {extraIterations} iterations");
#endif
        //after computing

        if (float.IsInfinity(startVertex.rhs)) //no path found!
            return -1;

        var bestIndex = -1;
        var minCost = float.PositiveInfinity;

        foreach (var (succIndex, cost) in startVertex.succ)
        {
            ref var succVertex = ref pathVertices.GetRef(succIndex);
            var newCost = cost + succVertex.g;

            if (newCost >= minCost)
                continue;

            minCost = newCost;
            bestIndex = succIndex;
        }

        //if the best solution is a direct path
        if (bestIndex == goalIndex)
            return linkCount;

        if (bestIndex < linkCount)
            return bestIndex;

        //if best solution is a TeleportDestination something went wrong!
#if SMART_PATHFINDING_DEBUG
        PathfindingLibPlugin.Instance.Logger.LogFatal($"Something went wrong. index is {bestIndex}/{linkCount}");
#endif
        return -1;

        void UpdateVertex(ref PathVertex vertex)
        {
            var index = vertex.index;
            var heapIndex = heap.FindIndex(e => e.index == index);

            if (heapIndex != -1)
                heap.RemoveAt(heapIndex);

            if (!Mathf.Approximately(vertex.g, vertex.rhs))
                heap.AddOrdered(new(index, vertex.CalculateKey()), heapComparer);
        }
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
        var stopwatch = Stopwatch.StartNew();

        var pathVertices = new NativeArray<PathVertex>(vertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

        PopulateVertices(pathVertices);

        ref var startVertex = ref pathVertices.GetRef(startIndex);
        ref var goalVertex = ref pathVertices.GetRef(goalIndex);

        if (startVertex.succ.Length == 0)
        {
#if SMART_PATHFINDING_DEBUG
            PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed, start position is isolated");
#endif
            destinationIndex[0] = -1;
        }
        else if (goalVertex.pred.Length == 0)
        {
#if SMART_PATHFINDING_DEBUG
            PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed, goal position is isolated");
#endif
            destinationIndex[0] = -1;
        }
        else
        {
            var result = CalculatePath(pathVertices);

#if SMART_PATHFINDING_DEBUG
            PrintCurrPath(pathVertices);

            if (result == -1)
                PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed");
            else if (result == linkCount)
                PathfindingLibPlugin.Instance.Logger.LogInfo("Completed direct path");
            else
                PathfindingLibPlugin.Instance.Logger.LogInfo($"Completed path with index {result} ({SmartPathJobDataContainer.linkNames[result]})");
#endif

            destinationIndex[0] = result;
        }

        DisposeVertices(ref pathVertices);

        PathfindingLibPlugin.Instance.Logger.LogInfo($"Path took {stopwatch.Elapsed.TotalMilliseconds}ms");
    }

#if SMART_PATHFINDING_DEBUG
    private void PrintCurrPath(NativeArray<PathVertex> pathVertices)
    {
        var currIndex = startIndex;
        var builder = new StringBuilder("Path:\n");

        while (true)
        {
            ref var currVertex = ref pathVertices.GetRef(currIndex);

            builder.AppendFormat(" - distance: {0:0.###}", currVertex.g);
            if (currIndex == startIndex)
                builder.AppendFormat(" Start {0}\n", start);
            else if (currIndex == goalIndex)
                builder.AppendFormat(" Goal {0}\n", start);
            else if (currIndex < linkCount)
                builder.AppendFormat(" {0}\n", SmartPathJobDataContainer.linkNames[currIndex]);
            else if (currIndex >= linkCount && currIndex < linkCount + destinationCount)
                builder.AppendFormat(" linkDestination {0}\n", linkDestinations[currIndex - linkCount]);
            else
                builder.AppendFormat(" ??? ({0})\n", currIndex);

            if (currIndex == goalIndex)
                break;

            var bestIndex = -1;
            var minCost = float.PositiveInfinity;

            foreach (var (succIndex, cost) in currVertex.succ)
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
#endif

    public void Dispose()
    {
        destinationIndex.Dispose();
    }

}
