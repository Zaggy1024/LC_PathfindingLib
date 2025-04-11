﻿using System;
using System.Collections.Generic;

using Unity.Jobs;
using UnityEngine.AI;
using UnityEngine;

using PathfindingLib.Jobs;
using PathfindingLib.Utilities.Collections;

using SmartPathLinkOriginType = PathfindingLib.API.SmartPathfinding.SmartPathJobDataContainer.SmartPathLinkOriginType;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public class SmartPathTask : IDisposable
{
    private static readonly Vector3[] singleDestination = [Vector3.zero];

    private SmartPathJobDataContainer? jobData;
    private SmartFindPathJob job;
    private JobHandle jobHandle;

    internal void StartPathTask(NavMeshAgent agent, Vector3 origin, Vector3 destination)
    {
        if (jobData != null && !IsComplete)
            return;

        SmartPathJobDataContainer.ReleaseJobData(ref jobData);
        singleDestination[0] = destination;
        jobData = SmartPathJobDataContainer.GetJobData(agent, origin, singleDestination);

        StartJob();
    }

    internal void StartPathTask(NavMeshAgent agent, Vector3 origin, Vector3[] destinations)
    {
        if (jobData != null && !IsComplete)
            return;

        SmartPathJobDataContainer.ReleaseJobData(ref jobData);
        jobData = SmartPathJobDataContainer.GetJobData(agent, origin, destinations);

        StartJob();
    }

    internal void StartPathTask(NavMeshAgent agent, Vector3 origin, List<Vector3> destinations)
    {
        if (jobData != null && !IsComplete)
            return;

        SmartPathJobDataContainer.ReleaseJobData(ref jobData);
        jobData = SmartPathJobDataContainer.GetJobData(agent, origin, destinations);

        StartJob();
    }

    private void StartJob()
    {
        job.Initialize(jobData);
        jobHandle = job.ScheduleByRef();
    }

    public Vector3 Origin => jobData!.pathStart;

    public bool IsComplete => jobHandle.IsCompleted;

    public bool IsResultReady(int index)
    {
        if (index < 0)
            throw new IndexOutOfRangeException("Index cannot be negative.");
        if (index >= jobData!.pathGoalCount)
            throw new IndexOutOfRangeException($"Index {index} is larger than destination count {jobData!.pathGoalCount}.");

        return job.results[index].linkIndex != SmartFindPathJob.PlaceholderLinkIndex;
    }

    public float GetPathLength(int index)
    {
        if (index < 0)
            throw new IndexOutOfRangeException("Index cannot be negative.");
        if (index >= jobData!.pathGoalCount)
            throw new IndexOutOfRangeException($"Index {index} is larger than destination count {jobData!.pathGoalCount}.");

        ref var result = ref job.results.GetRef(index);

        if (result.linkIndex == SmartFindPathJob.PlaceholderLinkIndex)
            throw new InvalidOperationException("Result is not ready.");

        return result.pathLength;
    }

    public SmartPathDestination? GetResult(int index)
    {
        if (index < 0)
            throw new IndexOutOfRangeException("Index cannot be negative.");
        if (index >= jobData!.pathGoalCount)
            throw new IndexOutOfRangeException($"Index {index} is larger than destination count {jobData!.pathGoalCount}.");

        ref var result = ref job.results.GetRef(index);

        if (result.linkIndex == SmartFindPathJob.PlaceholderLinkIndex)
            throw new InvalidOperationException("Result is not ready.");

        if (result.linkIndex == -1)
            return null;

        if (result.linkIndex >= jobData.linkCount)
            return SmartPathDestination.DirectDestination(jobData.pathGoals[index]);

        var destination = jobData.linkOriginNodes[result.linkIndex];

        if (destination.type == SmartPathLinkOriginType.EntranceTeleport)
            return SmartPathDestination.EntranceTeleportDestination(destination.entrance);

        if (destination.type == SmartPathLinkOriginType.CallElevator)
            return SmartPathDestination.CallElevatorDestination(destination.elevatorFloor);

        if (destination.type == SmartPathLinkOriginType.RideElevator)
        {
            var targetFloorNode = jobData.linkDestinationNodes[result.linkDestinationIndex];
            return SmartPathDestination.RideElevatorDestination(targetFloorNode.elevatorFloor);
        }

        throw new Exception("Invalid destination type, this should be unreachable.");
    }

    public void Dispose()
    {
        SmartPathJobDataContainer.ReleaseJobData(ref jobData);
        job.FreeAllResources();
    }
}
