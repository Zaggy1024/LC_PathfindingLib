using System;
using System.Collections.Generic;

using Unity.Collections;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Pool;

using PathfindingLib.Jobs;
using PathfindingLib.Data;
using PathfindingLib.Utilities.Collections;

namespace PathfindingLib.API.SmartPathfinding;

internal sealed class SmartPathJobDataContainer : IDisposable
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

    private static readonly ObjectPool<SmartPathJobDataContainer> pool = new(() => new(), actionOnRelease: v => v.Clear(), actionOnDestroy: v => v.Dispose());

    internal static readonly List<string> linkNames = [];

    internal int agentTypeID;
    internal int areaMask;

    internal Vector3 pathStart;
    internal NativeArray<Vector3> pathGoals;
    internal NativeArray<SmartFindPathJob.PathResult> pathResults;
    internal int pathGoalCount;

    internal readonly List<SmartPathLinkNode> linkOriginNodes = [];
    internal readonly List<SmartPathLinkNode> linkDestinationNodes = [];

    internal NativeArrayBuilder<Vector3> linkOrigins;

    internal NativeArrayBuilder<IndexAndSize> linkDestinationSlices;
    internal NativeArrayBuilder<Vector3> linkDestinations;

    internal NativeArrayBuilder<int> linkDestinationCostOffsets;
    internal NativeArrayBuilder<float> linkDestinationCosts;

    internal static SmartPathJobDataContainer GetJobData(NavMeshAgent agent, Vector3 origin, Vector3[] destinations, int destinationCount, SmartPathfindingLinkFlags allowedLinks)
    {
        var jobData = pool.Get();
        jobData.FillJobData(agent, origin, destinations, destinationCount, allowedLinks);
        return jobData;
    }

    internal static SmartPathJobDataContainer GetJobData(NavMeshAgent agent, Vector3 origin, Vector3[] destinations, SmartPathfindingLinkFlags allowedLinks)
    {
        return GetJobData(agent, origin, destinations, destinations.Length, allowedLinks);
    }

    internal static SmartPathJobDataContainer GetJobData(NavMeshAgent agent, Vector3 origin, List<Vector3> destinations, SmartPathfindingLinkFlags allowedLinks)
    {
        return GetJobData(agent, origin, NoAllocHelpers.ExtractArrayFromListT(destinations), destinations.Count, allowedLinks);
    }

    internal static void ReleaseJobData(ref SmartPathJobDataContainer jobData)
    {
        if (jobData == null)
            return;
        pool.Release(jobData);
        jobData = null;
    }

    internal void FillJobData(NavMeshAgent agent, Vector3 origin, Vector3[] destinations, int destinationCount, SmartPathfindingLinkFlags allowedLinks)
    {
        agentTypeID = agent.agentTypeID;
        areaMask = agent.areaMask;

        pathStart = origin;

        // Populate all the goals into a native array.
        if (destinationCount > pathGoals.Length)
        {
            pathGoals.Dispose();
            pathGoals = new NativeArray<Vector3>(destinationCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            pathResults = new NativeArray<SmartFindPathJob.PathResult>(destinationCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        }
        for (var goalIndex = 0; goalIndex < destinationCount; goalIndex++)
        {
            pathGoals[goalIndex] = destinations[goalIndex];
            pathResults[goalIndex] = new();
        }
        pathGoalCount = destinationCount;

        var fillNames = linkNames.Count == 0;

        // Populate the entrance teleports' links.
        foreach (var entranceLink in SmartPathLinks.entranceTeleports.Values)
        {
            if (entranceLink.teleport == null || !entranceLink.teleport.isActiveAndEnabled)
                continue;

            var linkFlag = entranceLink.teleport.entranceId == 0 ? SmartPathfindingLinkFlags.MainEntrance : SmartPathfindingLinkFlags.FireExits;
            if ((allowedLinks & linkFlag) == 0)
                continue;

            var node = SmartPathLinkNode.EntranceTeleport(entranceLink.teleport);
            linkOriginNodes.Add(node);
            linkOrigins.Add(entranceLink.teleport.entrancePoint.position);

            linkDestinationNodes.Add(node);
            linkDestinationSlices.Add(new IndexAndSize(linkDestinations.Count, 1));
            linkDestinations.Add(entranceLink.exit.position);

            linkDestinationCostOffsets.Add(linkDestinationCosts.Count);
            linkDestinationCosts.Add(SmartFindPathJob.MinEdgeCost);

            if (fillNames)
                linkNames.Add(entranceLink.teleport.ToString());
        }

        // Populate the elevators' floors' links.
        if ((allowedLinks & SmartPathfindingLinkFlags.Elevators) != 0)
        {
            foreach (var (elevator, floors) in SmartPathLinks.elevators)
            {
                // Add links from each floor to each other floor, using the traversal cost from the elevator interface.
                var floorsSlice = new IndexAndSize(linkDestinations.Count, floors.Count);

                var currentFloor = elevator.CurrentFloor;
                var targetFloor = elevator.TargetFloor;

                foreach (var floor in floors)
                {
                    var node = SmartPathLinkNode.CallElevator(floor);
                    linkOriginNodes.Add(node);
                    linkOrigins.Add(floor.CallButtonNavMeshNode.position);

                    linkDestinationNodes.Add(node);
                    linkDestinationSlices.Add(floorsSlice);
                    linkDestinations.Add(floor.CallButtonNavMeshNode.position);

                    linkDestinationCostOffsets.Add(linkDestinationCosts.Count);

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

                        linkDestinationCosts.Add(cost);
                    }

                    if (fillNames)
                        linkNames.Add(floor.ToString());
                }

                // Add a link from the inside of the elevator to all floors.
                linkOriginNodes.Add(SmartPathLinkNode.RideElevator());
                linkOrigins.Add(elevator.InsideButtonNavMeshNode.position);

                linkDestinationSlices.Add(floorsSlice);

                linkDestinationCostOffsets.Add(linkDestinationCosts.Count);

                foreach (var floor in floors)
                {
                    var cost = float.PositiveInfinity;

                    // Link from inside the elevator to all other floors if doors are open.
                    // If the doors are closed, only link to the elevator's destination floor, to avoid
                    // AI choosing to send the elevator to another floor while they are inside it.
                    if (floor != currentFloor && (elevator.DoorsAreOpen || targetFloor == floor))
                        cost = elevator.TimeFromFloorToFloor(elevator.ClosestFloor, floor) * agent.speed;

                    linkDestinationCosts.Add(cost);
                }

                if (fillNames)
                    linkNames.Add(elevator.ToString());
            }
        }
    }

    private void Clear()
    {
        linkOriginNodes.Clear();
        linkDestinationNodes.Clear();

        linkOrigins.Clear();
        linkDestinationSlices.Clear();
        linkDestinations.Clear();
        linkDestinationCostOffsets.Clear();
        linkDestinationCosts.Clear();
    }

    public void Dispose()
    {
        linkOrigins.Dispose();
        linkDestinationSlices.Dispose();
        linkDestinations.Dispose();
        linkDestinationCostOffsets.Dispose();
        linkDestinationCosts.Dispose();
    }
}
