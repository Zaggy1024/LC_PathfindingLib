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
    public SmartDestinationType Type { get; internal set; }
    public Vector3 Position { get; internal set; }
    public EntranceTeleport? EntranceTeleport { get; internal set; }
    public ElevatorFloor? ElevatorFloor { get; internal set; }
    public IInternalTeleport? InternalTeleport { get; internal set; }

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
