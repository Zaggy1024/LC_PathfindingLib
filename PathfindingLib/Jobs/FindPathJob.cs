using System;

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Experimental.AI;

using PathfindingLib.Utilities;
using PathfindingLib.API;

#if BENCHMARKING
using Unity.Profiling;
#endif

namespace PathfindingLib.Jobs;

public struct FindPathJob : IJob, IDisposable
{
    [ReadOnly, NativeDisableContainerSafetyRestriction] private NativeArray<NavMeshQuery> ThreadQueriesRef;

    [ReadOnly] private int AgentTypeID;
    [ReadOnly] private int AreaMask;
    [ReadOnly] private Vector3 QueryExtents;
    [ReadOnly] private Vector3 Origin;
    [ReadOnly] private Vector3 Destination;

    [ReadOnly, NativeSetThreadIndex] private int ThreadIndex;

    [WriteOnly] private NativeArray<PathQueryStatus> Status;
    [WriteOnly] private NativeArray<float> PathLength;

    /// <summary>
    /// Initializes the job to search for a valid path for an agent from an origin to a destination.
    /// 
    /// <para>Positions do not need to be already on the navmesh, as the closest position on the navmesh
    /// will be sampled in the job code off the main thread.</para>
    /// 
    /// <para>After the job has been initialized, call <see cref="IJobExtensions.ScheduleByRef"/> to schedule
    /// the job to run whenever a job thread is available, passing a <see cref="JobHandle"/> from a previously
    /// scheduled job to ensure that it only runs after that has completed.</para>
    /// 
    /// <para>It is recommended that your jobs have a dependency on any previous related jobs, i.e. if you are
    /// checking multiple paths to choose one for your AI to use. Without such a dependency, the jobs may saturate
    /// all the available job threads, starve out shorter jobs that Unity synchronously waits for on main thread,
    /// and cause stutters. By passing a dependency, you will ensure that your jobs only run on one thread at a time,
    /// so that all other job threads will be free to do more urgent work.</para>
    /// </summary>
    /// <param name="origin">The start of the desired path.</param>
    /// <param name="destination">The end of the desired path.</param>
    /// <param name="agent">The agent that will be used to determine walkable areas.</param>
    public void Initialize(Vector3 origin, Vector3 destination, NavMeshAgent agent)
    {
        ThreadQueriesRef = PathfindingJobSharedResources.GetPerThreadQueriesArray();

        AgentTypeID = agent.agentTypeID;
        AreaMask = agent.areaMask;
        QueryExtents = NavMeshQueryUtils.GetQueryExtents(AgentTypeID);
        Origin = origin;
        Destination = destination;

        // Shhhh, compiler...
        ThreadIndex = -1;

        CreateArrays();

        Status[0] = PathQueryStatus.InProgress;
        PathLength[0] = float.MaxValue;
    }

    private void CreateArrays()
    {
        if (Status.Length == 1)
            return;

        Status = new NativeArray<PathQueryStatus>(1, Allocator.Persistent);
        PathLength = new NativeArray<float>(1, Allocator.Persistent);
    }

    private void DisposeArrays()
    {
        Status.Dispose();
        PathLength.Dispose();
    }

#if BENCHMARKING
    private static readonly ProfilerMarker FindPathMarker = new("FindPath");
    private static readonly ProfilerMarker FinalizeJobMarker = new("FinalizeJob");
#endif

    public void Execute()
    {
        var query = ThreadQueriesRef[ThreadIndex];

        using var readLocker = new NavMeshReadLocker();

#if BENCHMARKING
        using var markerAuto = new TogglableProfilerAuto(in FindPathMarker);
#endif

        var origin = query.MapLocation(Origin, QueryExtents, AgentTypeID, AreaMask);

        if (!query.IsValid(origin.polygon))
        {
            Status[0] = PathQueryStatus.Failure;
            return;
        }

        var destinationLocation = query.MapLocation(Destination, QueryExtents, AgentTypeID, AreaMask);
        if (!query.IsValid(destinationLocation))
        {
            Status[0] = PathQueryStatus.Failure;
            return;
        }

        var status = query.BeginFindPath(origin, destinationLocation, AreaMask);
        if (status.GetResult() == PathQueryStatus.Failure)
        {
            Status[0] = PathQueryStatus.Failure;
            return;
        }

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
        {
            Status[0] = PathQueryStatus.Failure;
            return;
        }

        var pathNodes = new NativeArray<PolygonId>(pathNodesSize, Allocator.Temp);
        pathNodesSize = query.GetPathResult(pathNodes);

        using var path = new NativeArray<Vector3>(NavMeshQueryUtils.RequiredCornerCount(pathNodesSize), Allocator.Temp);
        var straightPathStatus = query.FindStraightPath(Origin, Destination, pathNodes, pathNodesSize, path, out var pathSize);
        pathNodes.Dispose();

        readLocker.Dispose();

#if BENCHMARKING
        markerAuto.Pause();
        using var finalizeMarkerAuto = FinalizeJobMarker.Auto();
#endif

        if (straightPathStatus.GetResult() != PathQueryStatus.Success)
        {
            Status[0] = straightPathStatus;
            return;
        }

        // Check if the end of the path is close enough to the target.
        var endPosition = path[pathSize - 1];
        var endDistance = math.distancesq(Destination, endPosition);
        if (endDistance > SharedJobValues.MaximumEndpointDistanceSquared)
        {
            Status[0] = PathQueryStatus.Failure;
            return;
        }

        var distance = 0f;
        for (var i = 1; i < pathSize; i++)
            distance += math.distance(path[i - 1], path[i]);
        PathLength[0] = distance;

        Status[0] = PathQueryStatus.Success;
    }

    /// <summary>
    /// Gets the status of the job.
    /// 
    /// <para>Note that <see cref="PathQueryStatus"/> is a flags enum, and the returned value may
    /// include some "detail" flags. To only check if the path is in progress, succeeded, or failed,
    /// call the extension method <see cref="NavMeshQueryUtils.GetResult"/> on the result of this
    /// method.</para>
    /// </summary>
    /// <returns>The current status of the job.</returns>
    public PathQueryStatus GetStatus()
    {
        return Status[0];
    }

    /// <summary>
    /// Gets the length of the calculated path, if the job completed and the path reached the destination.
    /// 
    /// <para>If the job or path is incomplete, the returned value is undefined.</para>
    /// </summary>
    /// <returns>The length of the path.</returns>
    public float GetPathLength()
    {
        return PathLength[0];
    }

    public void Dispose()
    {
        DisposeArrays();
    }
}