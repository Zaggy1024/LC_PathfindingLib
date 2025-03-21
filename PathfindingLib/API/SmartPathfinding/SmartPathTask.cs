using System;

using Unity.Jobs;
using UnityEngine.AI;
using UnityEngine;

using PathfindingLib.API.Smart;
using PathfindingLib.Jobs;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public class SmartPathTask : IDisposable
{
    private Vector3 origin;
    private Vector3 destination;

    private SmartPathJobDataContainer jobData;
    private SmartFindPathJob job;
    private JobHandle jobHandle;

    private SmartPathTask(Vector3 origin, Vector3 destination)
    {
        this.origin = origin;
        this.destination = destination;
        jobData = SmartPathJobDataContainer.GetJobData();
    }

    internal static SmartPathTask StartPathTask(Vector3 origin, Vector3 destination, NavMeshAgent agent)
    {
        var task = new SmartPathTask(origin, destination);
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

            var index = job.firstNodeIndices[0];

            if (index == -1)
                return null;

            if (index >= jobData.linkCount)
                return SmartPathDestination.DirectDestination(destination);

            return jobData.linkOriginDestinations[index];
        }
    }

    public void Dispose()
    {
        SmartPathJobDataContainer.ReleaseJobData(ref jobData);
        job.FreeAllResources();
    }
}
