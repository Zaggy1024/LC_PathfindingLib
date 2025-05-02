using System;
using UnityEngine;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public enum SmartDestinationType
{
    DirectToDestination,
    InternalTeleport,
    EntranceTeleport,
    Elevator,
}

public struct SmartPathDestination
{
    public SmartDestinationType Type { get; private set; }
    public Vector3 Position { get; private set; }
    public EntranceTeleport EntranceTeleport
    {
        readonly get
        {
            if (Type != SmartDestinationType.EntranceTeleport)
                throw new InvalidOperationException("Type is not EntranceTeleport.");
            return field;
        }
        private set;
    }
    public ElevatorFloor ElevatorFloor
    {
        readonly get
        {
            if (Type != SmartDestinationType.Elevator)
                throw new InvalidOperationException("Type is not Elevator.");
            return field;
        }
        private set;
    }
    public IInternalTeleport InternalTeleport
    {
        readonly get
        {
            if (Type != SmartDestinationType.InternalTeleport)
                throw new InvalidOperationException("Type is not InternalTeleport.");
            return field;
        }
        private set;
    }

    public static SmartPathDestination DirectDestination(Vector3 position)
    {
        return new SmartPathDestination()
        {
            Type = SmartDestinationType.DirectToDestination,
            Position = position,
        };
    }

    public static SmartPathDestination InternalTeleportDestination(IInternalTeleport teleport)
    {
        return new SmartPathDestination()
        {
            Type = SmartDestinationType.InternalTeleport,
            Position = teleport.Origin.position,
            InternalTeleport = teleport,
        };
    }

    public static SmartPathDestination EntranceTeleportDestination(EntranceTeleport entrance)
    {
        return new SmartPathDestination() {
            Type = SmartDestinationType.EntranceTeleport,
            Position = entrance.entrancePoint.position,
            EntranceTeleport = entrance,
        };
    }

    public static SmartPathDestination CallElevatorDestination(ElevatorFloor floor)
    {
        return new SmartPathDestination()
        {
            Type = SmartDestinationType.Elevator,
            Position = floor.CallButtonNavMeshNode.position,
            ElevatorFloor = floor,
        };
    }

    public static SmartPathDestination RideElevatorDestination(ElevatorFloor floor)
    {
        return new SmartPathDestination()
        {
            Type = SmartDestinationType.Elevator,
            Position = floor.Elevator.InsideButtonNavMeshNode.position,
            ElevatorFloor = floor,
        };
    }

    public readonly override string ToString()
    {
        return Type switch
        {
            SmartDestinationType.DirectToDestination => $"Direct: {Position}",
            SmartDestinationType.EntranceTeleport => $"Entrance Teleport: {EntranceTeleport} @ {Position}",
            SmartDestinationType.Elevator => $"Elevator Floor: {ElevatorFloor!.Elevator} @ {Position}",
            SmartDestinationType.InternalTeleport => $"Internal Teleport: {InternalTeleport!.Name} @ {Position}",
            _ => "Unknown",
        };
    }
}
