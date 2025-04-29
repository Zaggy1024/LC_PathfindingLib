using System.Collections.Generic;

using UnityEngine;

using PathfindingLib.API.SmartPathfinding;

namespace PathfindingLib.Data;

internal static class SmartPathLinks
{
    internal struct EntranceTeleportLink(EntranceTeleport teleport, Transform exit)
    {
        internal EntranceTeleport teleport = teleport;
        internal Transform exit = exit;
    }

    internal static readonly Dictionary<EntranceTeleport, EntranceTeleportLink> entranceTeleports = [];
    internal static readonly Dictionary<IElevator, HashSet<ElevatorFloor>> elevators = [];
    internal static readonly HashSet<IInternalTeleport> internalTeleports = [];

    internal static void RegisterEntranceTeleport(EntranceTeleport teleport, Transform exit)
    {
        entranceTeleports[teleport] = new EntranceTeleportLink(teleport, exit);
    }

    internal static void UnregisterEntranceTeleport(EntranceTeleport teleport)
    {
        if (!entranceTeleports.Remove(teleport))
        {
            PathfindingLibPlugin.Instance.Logger.LogWarning($"Attempted to remove {teleport} from smart pathfinding data while it was not present.");
            return;
        }
    }

    internal static HashSet<ElevatorFloor> GetFloors(IElevator elevator)
    {
        if (!elevators.TryGetValue(elevator, out var floors))
        {
            floors = [];
            elevators[elevator] = floors;
        }

        return floors;
    }

    internal static void RegisterElevatorFloor(ElevatorFloor floor)
    {
        var floors = GetFloors(floor.Elevator);
        floors.Add(floor);
    }

    internal static void UnregisterElevatorFloor(ElevatorFloor floor)
    {
        var elevator = floor.Elevator;
        var floors = GetFloors(elevator);
        floors.Remove(floor);
        if (floors.Count == 0)
            elevators.Remove(elevator);
    }

    internal static void RegisterInternalTeleport(IInternalTeleport teleport)
    {
        internalTeleports.Add(teleport);
    }

    internal static void UnregisterInternalTeleport(IInternalTeleport teleport)
    {
        internalTeleports.Remove(teleport);
    }
}
