using System;

using Unity.Jobs;
using UnityEngine.AI;
using UnityEngine;

using PathfindingLib.Jobs;

using SmartPathLinkOriginType = PathfindingLib.API.SmartPathfinding.SmartPathJobDataContainer.SmartPathLinkOriginType;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public class SmartPathTask : IDisposable
{
    private Vector3 origin;
    private Vector3 destination;

    private SmartPathJobDataContainer jobData;
    private SmartFindPathJob job;
    private JobHandle jobHandle;

    private SmartPathTask(Vector3 origin, Vector3 destination, NavMeshAgent agent)
    {
        this.origin = origin;
        this.destination = destination;
        jobData = SmartPathJobDataContainer.GetJobData(agent);
    }

    internal static SmartPathTask StartPathTask(Vector3 origin, Vector3 destination, NavMeshAgent agent)
    {
        var task = new SmartPathTask(origin, destination, agent);
        task.job.Initialize(task.jobData, origin, new Vector3[] { destination }, agent);
        task.jobHandle = task.job.ScheduleByRef();
        return task;
    }

    public Vector3 Origin => origin;
    public Vector3 Destination => destination;

    public bool IsComplete => jobHandle.IsCompleted;

    public SmartPathDestination? Result
    {
        get
        {
            if (!IsComplete)
                throw new InvalidOperationException("Job is not complete.");

            var result = job.results[0];

            if (result.linkIndex == -1)
                return null;

            if (result.linkIndex >= jobData.linkCount)
                return SmartPathDestination.DirectDestination(Destination);

            var destination = jobData.linkOriginNodes[result.linkIndex];

            if (destination.type == SmartPathLinkOriginType.EntranceTeleport)
            {
                return SmartPathDestination.EntranceTeleportDestination(destination.entrance);
            }
            else if (destination.type == SmartPathLinkOriginType.CallElevator)
            {
                return SmartPathDestination.CallElevatorDestination(destination.elevatorFloor);
            }
            else if (destination.type == SmartPathLinkOriginType.RideElevator)
            {
                var targetFloorNode = jobData.linkDestinationNodes[result.linkDestinationIndex];
                return SmartPathDestination.RideElevatorDestination(targetFloorNode.elevatorFloor);
            }

            return null;
        }
    }

    public void Dispose()
    {
        SmartPathJobDataContainer.ReleaseJobData(ref jobData);
        job.FreeAllResources();
    }
}
