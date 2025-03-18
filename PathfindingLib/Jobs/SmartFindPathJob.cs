//#define SMART_PATHFINDING_DEBUG

using System;
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

#if BENCHMARKING
using Unity.Profiling;
#endif

#if SMART_PATHFINDING_DEBUG
using System.Text;
#endif

namespace PathfindingLib.Jobs;

public struct SmartFindPathJob : IJob, IDisposable
{
    [ReadOnly, NativeDisableContainerSafetyRestriction] private NativeArray<NavMeshQuery> ThreadQueriesRef;

    [ReadOnly] private int agentTypeID;
    [ReadOnly] private int areaMask;
    [ReadOnly] private Vector3 origin;
    [ReadOnly] private Vector3 destination;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<Vector3> linkOrigins;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<IndexAndSize> linkDestinationSlices;
    [ReadOnly, NativeDisableParallelForRestriction] private NativeArray<Vector3> linkDestinations;
    [ReadOnly] private int linkCount;

    [ReadOnly, NativeSetThreadIndex] private int threadIndex;

    [WriteOnly] internal NativeArray<int> destinationIndex;

    internal void Initialize(SmartPathJobDataContainer data, Vector3 origin, Vector3 destination, NavMeshAgent agent)
    {
        ThreadQueriesRef = PathfindingJobSharedResources.GetPerThreadQueriesArray();

        agentTypeID = agent.agentTypeID;
        areaMask = agent.areaMask;
        this.origin = origin;
        this.destination = destination;

        // Shhhh, compiler...
        threadIndex = -1;

        linkOrigins = data.linkOrigins;
        linkDestinationSlices = data.linkDestinationSlices;
        linkDestinations = data.linkDestinations;
        linkCount = data.linkCount;

        destinationIndex = new(1, Allocator.Persistent);
        destinationIndex[0] = -1;
    }

#if BENCHMARKING
    private static readonly ProfilerMarker CalculateSinglePathMarker = new("CalculateSinglePath");
#endif

    private float CalculateSinglePath(Vector3 start, Vector3 end)
    {
        var query = ThreadQueriesRef[threadIndex];

        using var readLocker = new NavMeshReadLocker();

#if BENCHMARKING
        using var markerAuto = new TogglableProfilerAuto(in CalculateSinglePathMarker);
#endif

        var startLocation = query.MapLocation(start, SharedJobValues.OriginExtents, agentTypeID, areaMask);

        if (!query.IsValid(startLocation.polygon))
            return float.PositiveInfinity;

        var destinationExtents = SharedJobValues.DestinationExtents;
        var destinationLocation = query.MapLocation(end, destinationExtents, agentTypeID, areaMask);
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
        var straightPathStatus = NavMeshQueryUtils.FindStraightPath(query, start, end, pathNodes, pathNodesSize, path, out var pathSize);
        pathNodes.Dispose();

        readLocker.Dispose();

        if (straightPathStatus.GetResult() != PathQueryStatus.Success)
            return float.PositiveInfinity;

        // Check if the end of the path is close enough to the target.
        var pathEnd = path[pathSize - 1].position;
        var endDistance = (pathEnd - end).sqrMagnitude;
        if (endDistance > SharedJobValues.MaximumEndpointDistanceSquared)
            return float.PositiveInfinity;

        var distance = 0f;
        for (var i = 1; i < pathSize; i++)
            distance += Vector3.Distance(path[i - 1].position, path[i].position);

        return distance;
    }

    private struct PathFrame
    {
        internal int rootNode;

        internal float length;
        internal int segmentCount;
        internal Vector3 origin;
        internal int destination;

#if SMART_PATHFINDING_DEBUG
        internal NativeArray<int> path;
#endif

        internal PathFrame(int rootNode, Vector3 origin)
        {
            this.rootNode = rootNode;
            this.origin = origin;

            destination = rootNode;

#if SMART_PATHFINDING_DEBUG
            path = new NativeArray<int>(1, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            path[0] = rootNode;
#endif
        }

        internal readonly PathFrame Extend(float segmentLength, Vector3 newOrigin, int newDestination)
        {
            var result = new PathFrame()
            {
                rootNode = rootNode,
                origin = newOrigin,
                destination = newDestination,
                length = length + segmentLength,
                segmentCount = segmentCount + 1,
#if SMART_PATHFINDING_DEBUG
                path = new NativeArray<int>(path.Length + 1, Allocator.Temp, NativeArrayOptions.UninitializedMemory),
#endif
            };

#if SMART_PATHFINDING_DEBUG
            NativeArray<int>.Copy(path, result.path, path.Length);
            result.path[^1] = newDestination;
#endif
            return result;
        }
    }

    private struct PathFrameComparer : IComparer<PathFrame>
    {
        public readonly int Compare(PathFrame a, PathFrame b)
        {
            if (Mathf.Approximately(a.length, b.length))
                return Comparer<int>.Default.Compare(a.segmentCount, b.segmentCount);

            return Comparer<float>.Default.Compare(a.length, b.length);
        }
    }

    private Vector3 GetDestinationAtIndex(int index)
    {
        if (index == linkCount)
            return destination;
        return linkOrigins[index];
    }

    private int CalculatePath()
    {
        var shortestLength = float.PositiveInfinity;
        var shortestSegmentCount = int.MaxValue;
        var shortestPath = -1;

        // Fill the heap with the initial destinations:
        //   - The direct destination
        //   - All link origins
        var heap = new NativeHeap<PathFrame, PathFrameComparer>(Allocator.Temp, 16, new PathFrameComparer());
        heap.Insert(new PathFrame(linkCount, origin));

        for (var i = 0; i < linkCount; i++)
            heap.Insert(new PathFrame(i, origin));

        // Pull the next shortest path from the queue until it is empty.
        while (heap.TryPop(out PathFrame currentFrame))
        {
            // Skip paths that cannot be shorter than the current shortest path.
            if (Mathf.Approximately(currentFrame.length, shortestLength) && currentFrame.segmentCount >= shortestSegmentCount)
                continue;
            if (currentFrame.length >= shortestLength)
                continue;
            if (currentFrame.segmentCount > linkCount)
                continue;

#if SMART_PATHFINDING_DEBUG
            PathfindingLibPlugin.Instance.Logger.LogInfo($"Current frame (depth {currentFrame.segmentCount}, length {currentFrame.length}) destination: {currentFrame.destination} ({(currentFrame.destination < linkCount ? SmartPathJobDataContainer.linkNames[currentFrame.destination] : "destination")})");
#endif

            var destinationPosition = GetDestinationAtIndex(currentFrame.destination);

            // Get the length of the path to the current link or the final destination.
            var segmentLength = CalculateSinglePath(currentFrame.origin, destinationPosition);
            if (float.IsPositiveInfinity(segmentLength))
            {
#if SMART_PATHFINDING_DEBUG
                PathfindingLibPlugin.Instance.Logger.LogInfo($"  Path from {currentFrame.origin} to {destinationPosition} failed");
#endif
                continue;
            }

#if SMART_PATHFINDING_DEBUG
            PathfindingLibPlugin.Instance.Logger.LogInfo($"  Path from {currentFrame.origin} to {destinationPosition} has length {segmentLength}");
#endif

            // If we found a direct path that is shorter than the current shortest, replace it with the current one.
            var newLength = currentFrame.length + segmentLength;
            var newSegmentCount = currentFrame.segmentCount + 1;

            if (currentFrame.destination == linkCount)
            {
                if (Mathf.Approximately(newLength, shortestLength) && newSegmentCount >= shortestSegmentCount)
                    continue;
                if (newLength >= shortestLength)
                    continue;
                shortestLength = newLength;
                shortestSegmentCount = newSegmentCount;
                shortestPath = currentFrame.rootNode;

#if SMART_PATHFINDING_DEBUG
                var debugStr = new StringBuilder(256);
                debugStr.AppendFormat("  Found path of length {0:.###} ({1} segments)\n", newLength, currentFrame.segmentCount);
                foreach (var linkID in currentFrame.path)
                {
                    debugStr.Append(" - ");
                    if (linkID >= linkCount)
                        debugStr.Append("Destination");
                    else
                        debugStr.Append(SmartPathJobDataContainer.linkNames[linkID]);
                    debugStr.Append('\n');
                }
                PathfindingLibPlugin.Instance.Logger.LogInfo(debugStr);
#endif
                continue;
            }

            // Again, insert the direct destination and all links to recurse.
            var linkDestinationSlice = linkDestinationSlices[currentFrame.destination];

            for (var linkDestinationSliceIndex = 0; linkDestinationSliceIndex < linkDestinationSlice.size; linkDestinationSliceIndex++)
            {
                var linkDestinationIndex = linkDestinationSlice.index + linkDestinationSliceIndex;
                var linkDestination = linkDestinations[linkDestinationIndex];
                var directPathFrame = currentFrame.Extend(segmentLength, linkDestination, linkCount);
                heap.Insert(directPathFrame);

#if SMART_PATHFINDING_DEBUG
                PathfindingLibPlugin.Instance.Logger.LogInfo($"    Link leads to {linkDestinationIndex} ({linkDestination})");
                PathfindingLibPlugin.Instance.Logger.LogInfo($"    - Added direct path frame with length {directPathFrame.length}");
#endif

                for (var i = 0; i < linkCount; i++)
                {
                    var linkPathFrame = currentFrame.Extend(segmentLength, linkDestination, i);
                    heap.Insert(linkPathFrame);

#if SMART_PATHFINDING_DEBUG
                    PathfindingLibPlugin.Instance.Logger.LogInfo($"    - Added link path frame to {i} ({SmartPathJobDataContainer.linkNames[i]}) with length {linkPathFrame.length}");
#endif
                }
            }
        }

        return shortestPath;
    }

    public void Execute()
    {
#if SMART_PATHFINDING_DEBUG
        PathfindingLibPlugin.Instance.Logger.LogInfo($"Start {origin} -> {destination} --------");
#endif

        var result = CalculatePath();

#if SMART_PATHFINDING_DEBUG
        if (result == -1)
            PathfindingLibPlugin.Instance.Logger.LogInfo($"Path failed");
        else if (result == linkCount)
            PathfindingLibPlugin.Instance.Logger.LogInfo($"Completed direct path");
        else
            PathfindingLibPlugin.Instance.Logger.LogInfo($"Completed path with index {result} ({SmartPathJobDataContainer.linkNames[result]})");
#endif

        destinationIndex[0] = result;
    }

    public void Dispose()
    {
        destinationIndex.Dispose();
    }
}