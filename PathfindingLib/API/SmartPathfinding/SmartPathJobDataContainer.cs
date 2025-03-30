using System;
using System.Collections.Generic;

using Unity.Collections;
using UnityEngine;
using UnityEngine.AI;
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

        internal static SmartPathLinkNode EntranceTeleport(EntranceTeleport teleport)
        {
            return new SmartPathLinkNode()
            {
                type = SmartPathLinkOriginType.EntranceTeleport,
                entrance = teleport,
            };
        }

        internal static SmartPathLinkNode CallElevator(ElevatorFloor elevatorFloor)
        {
            return new SmartPathLinkNode()
            {
                type = SmartPathLinkOriginType.CallElevator,
                elevatorFloor = elevatorFloor,
            };
        }

        internal static SmartPathLinkNode RideElevator()
        {
            return new SmartPathLinkNode()
            {
                type = SmartPathLinkOriginType.RideElevator,
            };
        }
    }

    private static readonly ObjectPool<SmartPathJobDataContainer> pool = new(() => new(), actionOnDestroy: v => v.Dispose());

    internal static readonly List<string> linkNames = [];

    internal readonly List<SmartPathLinkNode> linkOriginNodes = [];
    internal readonly List<SmartPathLinkNode> linkDestinationNodes = [];
    internal NativeArray<Vector3> linkOrigins;

    internal readonly List<Vector3> linkDestinationsManaged = [];
    internal NativeArray<IndexAndSize> linkDestinationSlices;
    internal NativeArray<Vector3> linkDestinations;

    internal readonly List<float> linkDestinationCostsManaged = [];
    internal NativeArray<int> linkDestinationCostOffsets;
    internal NativeArray<float> linkDestinationCosts;

    internal int linkCount;
    internal int linkDestinationCount;

    internal static SmartPathJobDataContainer GetJobData(NavMeshAgent agent)
    {
        var jobData = pool.Get();
        jobData.FillJobData(agent);
        return jobData;
    }

    internal static void ReleaseJobData(ref SmartPathJobDataContainer jobData)
    {
        jobData.Clear();
        pool.Release(jobData);
        jobData = null;
    }

    internal void FillJobData(NavMeshAgent agent)
    {
        var elevatorLinkCount = 0;

        foreach (var elevatorFloors in SmartPathLinks.elevators.Values)
            elevatorLinkCount += elevatorFloors.Count + 1;

        linkCount = SmartPathLinks.entranceTeleports.Count + elevatorLinkCount;
        if (linkCount > linkOrigins.Length)
        {
            linkOrigins.Dispose();
            linkOrigins = new NativeArray<Vector3>(linkCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

            linkDestinationSlices.Dispose();
            linkDestinationSlices = new NativeArray<IndexAndSize>(linkCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

            linkDestinationCostOffsets.Dispose();
            linkDestinationCostOffsets = new NativeArray<int>(linkCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        }

        var fillNames = linkNames.Count == 0;
        var i = 0;

        foreach (var entranceLink in SmartPathLinks.entranceTeleports.Values)
        {
            var node = SmartPathLinkNode.EntranceTeleport(entranceLink.teleport);
            linkOriginNodes.Add(node);
            linkOrigins[i] = entranceLink.teleport.entrancePoint.position;

            linkDestinationNodes.Add(node);
            linkDestinationSlices[i] = new IndexAndSize(linkDestinationsManaged.Count, 1);
            linkDestinationsManaged.Add(entranceLink.exit.position);

            linkDestinationCostOffsets[i] = linkDestinationCostsManaged.Count;
            linkDestinationCostsManaged.Add(SmartFindPathJob.MinEdgeCost);

            if (fillNames)
                linkNames.Add(entranceLink.teleport.ToString());

            i++;
        }

        foreach (var (elevator, floors) in SmartPathLinks.elevators)
        {
            // Add links from each floor to each other floor, using the traversal cost from the elevator interface.
            var floorsSlice = new IndexAndSize(linkDestinationsManaged.Count, floors.Count);

            var currentFloor = elevator.CurrentFloor;
            var targetFloor = elevator.TargetFloor;

            foreach (var floor in floors)
            {
                var node = SmartPathLinkNode.CallElevator(floor);
                linkOriginNodes.Add(node);
                linkOrigins[i] = floor.CallButtonNavMeshNode.position;

                linkDestinationNodes.Add(node);
                linkDestinationSlices[i] = floorsSlice;
                linkDestinationsManaged.Add(floor.CallButtonNavMeshNode.position);

                linkDestinationCostOffsets[i] = linkDestinationCostsManaged.Count;

                foreach (var destinationFloor in floors)
                {
                    var cost = float.PositiveInfinity;

                    // Disallow using the elevator button to go back to the closest floor, to avoid getting stuck.
                    if (floor != destinationFloor && floor != currentFloor)
                    {
                        cost = 0;
                        if (!elevator.DoorsAreOpen)
                            cost += elevator.TimeToCompleteCurrentMovement();
                        if (floor != targetFloor)
                            cost += elevator.TimeFromFloorToFloor(targetFloor, floor);
                        cost += elevator.TimeFromFloorToFloor(floor, destinationFloor);
                        cost *= agent.speed;
                    }

                    linkDestinationCostsManaged.Add(cost);
                }

                if (fillNames)
                    linkNames.Add(floor.ToString());

                i++;
            }

            // Add a link from the inside of the elevator to all floors.
            linkOriginNodes.Add(SmartPathLinkNode.RideElevator());
            linkOrigins[i] = elevator.InsideButtonNavMeshNode.position;

            linkDestinationSlices[i] = floorsSlice;

            linkDestinationCostOffsets[i] = linkDestinationCostsManaged.Count;

            foreach (var floor in floors)
            {
                var cost = float.PositiveInfinity;

                // Link from inside the elevator to all other floors if doors are open.
                // If the doors are closed, only link to the elevator's destination floor, to avoid
                // AI choosing to send the elevator to another floor while they are inside it.
                if (floor != currentFloor && (elevator.DoorsAreOpen || targetFloor == floor))
                    cost = elevator.TimeFromFloorToFloor(elevator.ClosestFloor, floor) * agent.speed;

                linkDestinationCostsManaged.Add(cost);
            }

            if (fillNames)
                linkNames.Add(elevator.ToString());

            i++;
        }

        if (linkDestinationsManaged.Count > linkDestinations.Length)
        {
            linkDestinations.Dispose();
            linkDestinations = new(linkDestinationsManaged.Count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        }

        NativeArray<Vector3>.Copy(NoAllocHelpers.ExtractArrayFromListT(linkDestinationsManaged), linkDestinations, linkDestinationsManaged.Count);
        linkDestinationCount = linkDestinationsManaged.Count;

        if (linkDestinationCostsManaged.Count > linkDestinationCosts.Length)
        {
            linkDestinationCosts.Dispose();
            linkDestinationCosts = new(linkDestinationCostsManaged.Count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        }

        NativeArray<float>.Copy(NoAllocHelpers.ExtractArrayFromListT(linkDestinationCostsManaged), linkDestinationCosts, linkDestinationCostsManaged.Count);
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
