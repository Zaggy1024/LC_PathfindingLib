using System;

using PathfindingLib.Data;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public static class SmartPathfinding
{
    public static void RegisterElevatorFloor(ElevatorFloor floor)
    {
        if (floor == null)
            throw new ArgumentNullException(nameof(floor));
        SmartPathLinks.RegisterElevatorFloor(floor);
    }

    public static void UnregisterElevatorFloor(ElevatorFloor floor)
    {
        if (floor == null)
            throw new ArgumentNullException(nameof(floor));
        SmartPathLinks.UnregisterElevatorFloor(floor);
    }

    public static void RegisterInternalTeleport(IInternalTeleport teleport)
    {
        if (teleport == null)
            throw new ArgumentNullException(nameof(teleport));
        SmartPathLinks.RegisterInternalTeleport(teleport);
    }

    public static void UnregisterInternalTeleport(IInternalTeleport teleport)
    {
        if (teleport == null)
            throw new ArgumentNullException(nameof(teleport));
        SmartPathLinks.UnregisterInternalTeleport(teleport);
    }
}
