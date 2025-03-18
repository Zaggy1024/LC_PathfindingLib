using UnityEngine;

using PathfindingLib.API.SmartPathfinding;

namespace PathfindingLib.API.Smart;

#nullable enable

public enum SmartDestinationType
{
    DirectToDestination,
    Elevator,
    EntranceTeleport,
}

public struct SmartPathDestination
{
    public SmartDestinationType Type { get; internal set; }
    public Vector3 Position { get; internal set; }
    public EntranceTeleport? EntranceTeleport { get; internal set; }
    public ElevatorFloor? ElevatorFloor { get; internal set; }

    public static SmartPathDestination DirectDestination(Vector3 position)
    {
        return new SmartPathDestination()
        {
            Type = SmartDestinationType.DirectToDestination,
            Position = position,
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
            Position = floor.ButtonNavMeshNode.position,
            ElevatorFloor = floor,
        };
    }

    public static SmartPathDestination RideElevatorDestination(ElevatorFloor floor)
    {
        return new SmartPathDestination()
        {
            Type = SmartDestinationType.Elevator,
            Position = floor.Elevator.CenterNavMeshNode.position,
            ElevatorFloor = floor,
        };
    }

    public override string ToString()
    {
        switch (Type)
        {
            case SmartDestinationType.DirectToDestination:
                return $"Direct: {Position}";
            case SmartDestinationType.EntranceTeleport:
                return $"Entrance Teleport: {EntranceTeleport} @ {Position}";
            case SmartDestinationType.Elevator:
                return $"Elevator Floor: {ElevatorFloor!.Elevator} @ {Position}";
        }

        return "uuh";
    }
}
