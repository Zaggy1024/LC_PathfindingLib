using System;
using System.Collections.Generic;

using Unity.Collections;
using UnityEngine;
using UnityEngine.Pool;

using PathfindingLib.Jobs;
using PathfindingLib.Data;

namespace PathfindingLib.API.SmartPathfinding;

internal class SmartPathJobDataContainer : IDisposable
{
    internal enum SmartPathLinkOriginType
    {
        EntranceTeleport,
        CallElevator,
        RideElevator,
    }

    internal struct SmartPathLinkNode
    {
        internal SmartPathLinkOriginType type;
        internal EntranceTeleport entrance;
        internal ElevatorFloor elevatorFloor;

        internal SmartPathLinkNode(EntranceTeleport entrance)
        {
            type = SmartPathLinkOriginType.EntranceTeleport;
            this.entrance = entrance;
        }

        internal SmartPathLinkNode(bool ride, ElevatorFloor elevatorFloor)
        {
            type = ride ? SmartPathLinkOriginType.RideElevator : SmartPathLinkOriginType.CallElevator;
            this.elevatorFloor = elevatorFloor;
        }
    }

    private static readonly ObjectPool<SmartPathJobDataContainer> pool = new(() => new(), actionOnDestroy: v => v.Dispose());

    internal static readonly List<string> linkNames = [];

    internal readonly List<SmartPathLinkNode> linkOriginNodes = [];
    internal readonly List<SmartPathLinkNode> linkDestinationNodes = [];
    internal readonly List<Vector3> linkDestinationsManaged = [];
    internal NativeArray<Vector3> linkOrigins;
    internal NativeArray<IndexAndSize> linkDestinationSlices;
    internal NativeArray<Vector3> linkDestinations;
    internal int linkCount;
    internal int linkDestinationCount;

    internal static SmartPathJobDataContainer GetJobData()
    {
        var jobData = pool.Get();
        jobData.FillJobData();
        return jobData;
    }

    internal static void ReleaseJobData(ref SmartPathJobDataContainer jobData)
    {
        jobData.Clear();
        pool.Release(jobData);
        jobData = null;
    }

    internal void FillJobData()
    {
        var elevatorLinkCount = 0;

        foreach (var elevatorFloors in SmartPathLinks.elevators.Values)
            elevatorLinkCount += elevatorFloors.Count + 1;

        linkCount = SmartPathLinks.entranceTeleports.Count + elevatorLinkCount;
        if (linkDestinationsManaged.Capacity < linkCount)
        {
            linkDestinationNodes.Capacity = linkCount;
            linkDestinationsManaged.Capacity = linkCount;
        }
        if (linkCount > linkOrigins.Length)
        {
            linkOrigins.Dispose();
            linkOrigins = new NativeArray<Vector3>(linkCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            linkDestinationSlices.Dispose();
            linkDestinationSlices = new NativeArray<IndexAndSize>(linkCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        }

        var fillNames = linkNames.Count == 0;
        var i = 0;

        foreach (var entranceLink in SmartPathLinks.entranceTeleports.Values)
        {
            var node = new SmartPathLinkNode(entranceLink.teleport);
            linkOriginNodes.Add(node);
            linkOrigins[i] = entranceLink.teleport.entrancePoint.position;

            linkDestinationNodes.Add(node);
            linkDestinationSlices[i] = new IndexAndSize(linkDestinationsManaged.Count, 1);
            linkDestinationsManaged.Add(entranceLink.exit.position);

            if (fillNames)
                linkNames.Add(entranceLink.teleport.ToString());

            i++;
        }

        foreach (var (elevator, floors) in SmartPathLinks.elevators)
        {
            var insideButtonPosition = elevator.InsideButtonNavMeshNode.position;

            // Add links from the call buttons to the inside of the elevator.
            var insideSlice = new IndexAndSize(linkDestinationsManaged.Count, 1);
            linkDestinationNodes.Add(new(ride: false, null));
            linkDestinationsManaged.Add(insideButtonPosition);

            foreach (var floor in floors)
            {
                linkOriginNodes.Add(new(ride: false, floor));
                linkOrigins[i] = floor.CallButtonNavMeshNode.position;

                linkDestinationSlices[i] = insideSlice;

                if (fillNames)
                    linkNames.Add(floor.ToString());

                i++;
            }

            // Add a link from the inside of the elevator to all call buttons.
            linkOriginNodes.Add(new(ride: true, null));
            linkOrigins[i] = insideButtonPosition;
            linkDestinationSlices[i] = new IndexAndSize(linkDestinationsManaged.Count, floors.Count);

            if (fillNames)
                linkNames.Add(elevator.ToString());

            foreach (var floor in floors)
            {
                linkDestinationNodes.Add(new(ride: true, floor));
                linkDestinationsManaged.Add(floor.CallButtonNavMeshNode.position);
            }

            i++;
        }

        if (linkDestinationsManaged.Count > linkDestinations.Length)
        {
            linkDestinations.Dispose();
            linkDestinations = new(linkDestinationsManaged.Count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        }

        NativeArray<Vector3>.Copy(NoAllocHelpers.ExtractArrayFromListT(linkDestinationsManaged), linkDestinations, linkDestinationsManaged.Count);
        linkDestinationCount = linkDestinationsManaged.Count;
    }

    private void Clear()
    {
        linkDestinationNodes.Clear();
        linkDestinationsManaged.Clear();
    }

    public void Dispose()
    {
        linkOrigins.Dispose();
        linkDestinationSlices.Dispose();
        linkDestinations.Dispose();

        linkOrigins = default;
        linkDestinationSlices = default;
        linkDestinations = default;
    }
}
