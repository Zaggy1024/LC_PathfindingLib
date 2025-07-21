using System;
using System.Collections.Generic;

using Unity.Jobs;
using UnityEngine.AI;
using UnityEngine;

using PathfindingLib.Components;
using PathfindingLib.Jobs;
using PathfindingLib.Utilities.Collections;

using SmartPathLinkOriginType = PathfindingLib.API.SmartPathfinding.SmartPathJobDataContainer.SmartPathLinkOriginType;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public sealed class SmartPathTask : IDisposable
{
    private static readonly Vector3[] singleDestination = [Vector3.zero];

    private SmartPathJobDataContainer? jobData;
    private SmartPathfindingJob job;
    private JobHandle jobHandle;

    public void StartPathTask(NavMeshAgent agent, Vector3 origin, Vector3 destination, SmartPathfindingLinkFlags allowedLinks)
    {
        if (jobData != null && !IsComplete)
            return;

        SmartPathJobDataContainer.ReleaseJobData(ref jobData);
        singleDestination[0] = destination;
        jobData = SmartPathJobDataContainer.GetJobData(agent, origin, singleDestination, allowedLinks);

        StartJob();
    }

    public void StartPathTask(NavMeshAgent agent, Vector3 origin, Vector3[] destinations, SmartPathfindingLinkFlags allowedLinks)
    {
        if (jobData != null && !IsComplete)
            return;
        if (destinations.Length == 0)
            return;

        SmartPathJobDataContainer.ReleaseJobData(ref jobData);
        jobData = SmartPathJobDataContainer.GetJobData(agent, origin, destinations, allowedLinks);

        StartJob();
    }

    public void StartPathTask(NavMeshAgent agent, Vector3 origin, List<Vector3> destinations, SmartPathfindingLinkFlags allowedLinks)
    {
        if (jobData != null && !IsComplete)
            return;
        if (destinations.Count == 0)
            return;

        SmartPathJobDataContainer.ReleaseJobData(ref jobData);
        jobData = SmartPathJobDataContainer.GetJobData(agent, origin, destinations, allowedLinks);

        StartJob();
    }

    private void StartJob()
    {
        job.Initialize(jobData);
        jobHandle = job.ScheduleByRef();
    }

    public Vector3 Origin => jobData!.pathStart;

    public bool IsResultReady(int index)
    {
        if (jobData == null)
            return false;
        if (index < 0)
            throw new IndexOutOfRangeException("Index cannot be negative.");
        if (index >= jobData!.pathGoalCount)
            throw new IndexOutOfRangeException($"Index {index} is larger than destination count {jobData!.pathGoalCount}.");

        return job.results[index].linkIndex != SmartPathfindingJob.PlaceholderLinkIndex;
    }

    public bool IsComplete => IsResultReady(jobData!.pathGoalCount - 1) && jobHandle.IsCompleted;

    public bool PathSucceeded(int index)
    {
        if (jobData == null)
            throw new InvalidOperationException("Job has not been started.");
        if (index < 0)
            throw new IndexOutOfRangeException("Index cannot be negative.");
        if (index >= jobData!.pathGoalCount)
            throw new IndexOutOfRangeException($"Index {index} is larger than destination count {jobData!.pathGoalCount}.");

        ref var result = ref job.results.GetRef(index);

        if (result.linkIndex == SmartPathfindingJob.PlaceholderLinkIndex)
            throw new InvalidOperationException("Result is not ready.");
        if (result.linkIndex == -1)
            return false;

        return true;
    }

    public float GetPathLength(int index)
    {
        if (jobData == null)
            throw new InvalidOperationException("Job has not been started.");
        if (index < 0)
            throw new IndexOutOfRangeException("Index cannot be negative.");
        if (index >= jobData!.pathGoalCount)
            throw new IndexOutOfRangeException($"Index {index} is larger than destination count {jobData!.pathGoalCount}.");

        ref var result = ref job.results.GetRef(index);

        if (result.linkIndex == SmartPathfindingJob.PlaceholderLinkIndex)
            throw new InvalidOperationException("Result is not ready.");
        if (result.linkIndex == -1)
            throw new InvalidOperationException("Path was not found.");

        return result.pathLength;
    }

    public SmartPathDestination? GetResult(int index)
    {
        if (jobData == null)
            throw new InvalidOperationException("Job has not been started.");
        if (index < 0)
            throw new IndexOutOfRangeException("Index cannot be negative.");
        if (index >= jobData!.pathGoalCount)
            throw new IndexOutOfRangeException($"Index {index} is larger than destination count {jobData!.pathGoalCount}.");

        ref var result = ref job.results.GetRef(index);

        if (result.linkIndex == SmartPathfindingJob.PlaceholderLinkIndex)
            throw new InvalidOperationException("Result is not ready.");

        if (result.linkIndex == -1)
            return null;

        if (result.linkIndex >= jobData.linkOrigins.Count)
            return SmartPathDestination.DirectDestination(jobData.pathGoals[index]);

        if (result.linkIndex < 0 || result.linkIndex > jobData.linkOriginNodes.Count)
            PathfindingLibPlugin.Instance.Logger.LogError($"Attempting to get link origin {result.linkIndex} with {jobData.linkOriginNodes.Count} links available.");

        var destination = jobData.linkOriginNodes[result.linkIndex];

        if (destination.type == SmartPathLinkOriginType.InternalTeleport)
            return SmartPathDestination.InternalTeleportDestination(destination.internalTeleport);

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
    }

    ~SmartPathTask()
    {
        SmartPathTaskDisposer.Enqueue(this);
    }
}
