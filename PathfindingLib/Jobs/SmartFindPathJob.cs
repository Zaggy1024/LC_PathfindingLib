#define SMART_PATHFINDING_DEBUG

using System;
using System.Buffers;
using System.Collections.Generic;

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
using UnityEngine.Rendering;

#if BENCHMARKING
using Unity.Profiling;
#endif

#if SMART_PATHFINDING_DEBUG
using System.Text;
#endif

namespace PathfindingLib.Jobs;

// ReSharper disable MemberCanBeMadeStatic.Local
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
    [ReadOnly] private int nodeCount;

    [ReadOnly, NativeSetThreadIndex] private int threadIndex;

    [WriteOnly] internal NativeArray<int> destinationIndex;

    internal void Initialize(SmartPathJobDataContainer data, Vector3 origin, Vector3 destination, NavMeshAgent agent)
    {
        ThreadQueriesRef = PathfindingJobSharedResources.GetPerThreadQueriesArray();

        agentTypeID = agent.agentTypeID;
        areaMask = agent.areaMask;
        this.start = origin;
        this.goal = destination;

        // Shhhh, compiler...
        threadIndex = -1;

        linkOrigins = data.linkOrigins;
        linkDestinationSlices = data.linkDestinationSlices;
        linkDestinations = data.linkDestinations;
        linkCount = data.linkCount;
        destinationCount = data.destinationCount;
        nodeCount = linkCount + destinationCount + 2;

        destinationIndex = new(1, Allocator.Persistent);
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

    private struct PathNode: IDisposable
    {
        internal List<(int index, float cost)> pred;
        internal List<(int index, float cost)> succ;

        internal float heuristic = float.PositiveInfinity;
        internal float g = float.PositiveInfinity;
        internal float rhs = float.PositiveInfinity;

        public PathNode()
        {
            pred = ListPool<(int index, float cost)>.Get();
            pred.Clear();
            succ = ListPool<(int index, float cost)>.Get();
            succ.Clear();
        }

        public void Dispose()
        {
            ListPool<(int index, float cost)>.Release(pred);
            pred = null;
            ListPool<(int index, float cost)>.Release(succ);
            succ = null;
            heuristic = float.NaN;
            g = float.NaN;
            rhs = float.NaN;
        }

        internal struct NodeKey : IComparable<NodeKey>
        {
            internal readonly float key1;
            internal readonly float key2;

            public NodeKey(float key1, float key2)
            {
                this.key1 = key1;
                this.key2 = key2;
            }

            public int CompareTo(NodeKey other)
            {
                var key1Comparison = key1.CompareTo(other.key1);
                return key1Comparison != 0 ? key1Comparison : key2.CompareTo(other.key2);
            }

            public static bool operator <(NodeKey left, NodeKey right)
            {
                return left.CompareTo(right) < 0;
            }

            public static bool operator >(NodeKey left, NodeKey right)
            {
                return left.CompareTo(right) > 0;
            }

            public static bool operator <=(NodeKey left, NodeKey right)
            {
                return left.CompareTo(right) <= 0;
            }

            public static bool operator >=(NodeKey left, NodeKey right)
            {
                return left.CompareTo(right) >= 0;
            }
        }

        public readonly NodeKey CalculateKey()
        {
            var key2 = Mathf.Min(g, rhs);
            var key1 = key2 + heuristic;

            return new NodeKey(key1, key2);
        }
    }

    private void PrepareData(PathNode[] nodes)
    {
        for (var i = 0; i < nodeCount; i++)
        {
            nodes[i] = new PathNode();
        }

        var startIndex = nodeCount - 2;
        var goalIndex = nodeCount - 1;
        ref var startNode = ref nodes[startIndex];
        ref var goalNode = ref nodes[goalIndex];

        for (var i = 0; i < nodeCount; i++)
        {
            ref var node = ref nodes[i];

            if (i < linkCount) //if this index is an origin
            {
                var nodePosition = linkOrigins[i];
                node.heuristic = (nodePosition - start).magnitude;

                //connect all its destinations

                var slice = linkDestinationSlices[i];

                for (var j = slice.index; j < slice.index + slice.size; j++)
                {
                    var destIndex = linkCount + j;

                    //TODO add costs! it's important the cost is never 0 or less
                    var traverseCost = 0.0001f;

                    ref var destinationNode = ref nodes[destIndex];

                    destinationNode.pred.Add((i, traverseCost));
                    node.succ.Add((destIndex, traverseCost));
                }

            }else if (i < linkCount + destinationCount) //if this is a destination
            {
                var destIndex = i - linkCount;
                var nodePosition = linkDestinations[destIndex];
                node.heuristic = (nodePosition - start).magnitude;

                //try path to all origins

                for (var j = 0; j < linkCount; j++)
                {
                    var originPos = linkOrigins[j];
                    ref var originNode = ref nodes[j];

                    //try connect
                    var cost = CalculateSinglePath(nodePosition, originPos);

                    //if there is a valid path
                    if (float.IsInfinity(cost))
                        continue;

                    originNode.pred.Add((i, cost));
                    node.succ.Add((j, cost));
                }

                //try path to our goal
                var goalCost = CalculateSinglePath(nodePosition, goal);

                //if there is a valid path
                if (float.IsInfinity(goalCost))
                    continue;

                goalNode.pred.Add((i, goalCost));
                node.succ.Add((goalIndex, goalCost));

            }else if (i == startIndex) //if this index is our start point
            {
                node.heuristic = 0;

                //try path to all origins

                for (var j = 0; j < linkCount; j++)
                {
                    var originPos = linkOrigins[j];
                    ref var originNode = ref nodes[j];

                    //try connect
                    var cost = CalculateSinglePath(start, originPos);

                    //if there is a valid path
                    if (float.IsInfinity(cost))
                        continue;

                    originNode.pred.Add((i, cost));
                    node.succ.Add((j, cost));
                }

                //try path directly to our goal
                var goalCost = CalculateSinglePath(start, goal);

                //if there is a valid path
                if (float.IsInfinity(goalCost))
                    continue;

                goalNode.pred.Add((i, goalCost));
                node.succ.Add((goalIndex, goalCost));
            }else if (i == goalIndex) //if this index is our goal
            {
                node.heuristic = (goal - start).magnitude;
            }
            else
            {
#if SMART_PATHFINDING_DEBUG
                PathfindingLibPlugin.Instance.Logger.LogError($"Index {i} is not mapped in PrepareData");
#endif
            }
        }
    }

    private void ReleaseData(PathNode[] nodes)
    {
        for (var i = 0; i < nodeCount ; i++)
        {
            ref var node = ref nodes[i];
            node.Dispose();
        }
    }

    private struct HeapElementComparer : IComparer<(int index, PathNode.NodeKey key)>
    {
        public int Compare((int index, PathNode.NodeKey key) x, (int index, PathNode.NodeKey key) y)
        {
            return x.Item2.CompareTo(y.Item2);
        }
    }

    private int CalculatePath(PathNode[] nodes)
    {
        var startIndex = nodeCount - 2;
        var goalIndex = nodeCount - 1;
        ref var startNode = ref nodes[startIndex];
        ref var goalNode = ref nodes[goalIndex];

        goalNode.rhs = 0;
        var heapComparer = new HeapElementComparer();
        var heap = new List<(int index, PathNode.NodeKey key)>();
        heap.Add((goalIndex, goalNode.CalculateKey()));

        var iterations = 0;
        var extraIterations = 0;

        while (heap.Count > 0)
        {
            iterations += 1;
            var (index, oldKey) = heap[0];
            ref var currentNode = ref nodes[index];

            //if we have found a path
            if (oldKey >= startNode.CalculateKey() && startNode.rhs <= startNode.g)
                //try to compute a few extra times in hope of finding a better path
                if (extraIterations++ >= MaxExtraIterations)
                    break;

            var newKey = currentNode.CalculateKey();

            if (oldKey < newKey)
            {
                //update heap
                heap.RemoveAt(0);
                heap.AddOrdered((index, newKey), heapComparer);
            }else if (currentNode.g > currentNode.rhs)
            {
                currentNode.g = currentNode.rhs;
                heap.RemoveAt(0);
                foreach (var (predIndex, cost) in currentNode.pred)
                {
                    if (predIndex != goalIndex)
                    {
                        ref var predNode = ref nodes[predIndex];

                        var newRhs = cost + currentNode.g;

                        if (newRhs < predNode.rhs)
                            predNode.rhs = newRhs;
                    }

                    UpdateNode(predIndex);
                }
            }
            else
            {
                var gOld = currentNode.g;
                currentNode.g = float.PositiveInfinity;

                List<(int index, float cost)> computable = [..currentNode.pred, (index, 0)];
                foreach (var (predIndex, predCost) in computable)
                {
                    if (predIndex != goalIndex)
                    {
                        ref var predNode = ref nodes[predIndex];

                        if (Mathf.Approximately(predNode.rhs, predCost + gOld))
                        {
                            var minRhs = float.PositiveInfinity;

                            foreach (var (succIndex, succCost) in predNode.succ)
                            {
                                ref var succNode = ref nodes[succIndex];

                                var newRhs = succCost + succNode.g;

                                if (newRhs < minRhs)
                                    minRhs = newRhs;
                            }

                            predNode.rhs = minRhs;
                        }
                    }

                    UpdateNode(predIndex);
                }
            }
        }

#if SMART_PATHFINDING_DEBUG
        PathfindingLibPlugin.Instance.Logger.LogDebug($"Found path after {iterations-extraIterations} iterations, then searched for an extra {extraIterations} iterations");
#endif
        //after computing

        if (float.IsInfinity(startNode.rhs)) //no path found!
            return -1;

        var bestIndex = -1;
        var minCost = float.PositiveInfinity;

        foreach (var (succIndex, cost) in startNode.succ)
        {
            ref var succNode = ref nodes[succIndex];
            var newCost = cost + succNode.g;

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

        void UpdateNode(int index)
        {
            ref var node = ref nodes[index];

            var heapIndex = heap.FindIndex(e => e.index == index);

            if (heapIndex != -1)
            {
                heap.RemoveAt(heapIndex);
            }

            if (!Mathf.Approximately(node.g, node.rhs))
            {
                heap.AddOrdered((index, node.CalculateKey()), heapComparer);
            }
        }
    }

    public void Execute()
    {
#if SMART_PATHFINDING_DEBUG
        PathfindingLibPlugin.Instance.Logger.LogInfo($"Start {start} -> {goal} --------");
#endif
        var nodes = ArrayPool<PathNode>.Shared.Rent(linkCount + destinationCount + 2);

        PrepareData(nodes);

        ref var startNode = ref nodes[linkCount + destinationCount];
        ref var goalNode = ref nodes[linkCount + destinationCount + 1];

        if (startNode.succ.Count == 0)
        {
#if SMART_PATHFINDING_DEBUG
            PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed, start position is isolated");
#endif
            destinationIndex[0] = -1;
        }else if (goalNode.pred.Count == 0)
        {
#if SMART_PATHFINDING_DEBUG
            PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed, goal position is isolated");
#endif
            destinationIndex[0] = -1;
        }
        else
        {
            var result = CalculatePath(nodes);

#if SMART_PATHFINDING_DEBUG

            PrintCurrPath(nodes);

            if (result == -1)
                PathfindingLibPlugin.Instance.Logger.LogInfo("Path failed");
            else if (result == linkCount)
                PathfindingLibPlugin.Instance.Logger.LogInfo("Completed direct path");
            else
                PathfindingLibPlugin.Instance.Logger.LogInfo($"Completed path with index {result} ({SmartPathJobDataContainer.linkNames[result]})");
#endif

            destinationIndex[0] = result;
        }

        ReleaseData(nodes);
        ArrayPool<PathNode>.Shared.Return(nodes);
    }

#if SMART_PATHFINDING_DEBUG
    private void PrintCurrPath(PathNode[] nodes)
    {
        var startIndex = nodeCount - 2;
        var goalIndex = nodeCount - 1;
        ref var startNode = ref nodes[startIndex];

        var currIndex = startIndex;
        var builder = new StringBuilder("Path:\n");

        while (true)
        {
            ref var currNode = ref nodes[currIndex];

            builder.AppendFormat(" - distance: {0:0.###}", currNode.g);
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

            foreach (var (succIndex, cost) in currNode.succ)
            {
                ref var succNode = ref nodes[succIndex];
                var newCost = cost + succNode.g;

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
