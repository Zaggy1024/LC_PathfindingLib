using PathfindingLib.Data;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public static class SmartPathfinding
{
    public static void RegisterElevatorFloor(ElevatorFloor floor)
    {
        SmartPathLinks.RegisterElevatorFloor(floor);
    }

    public static void UnregisterElevatorFloor(ElevatorFloor floor)
    {
        SmartPathLinks.UnregisterElevatorFloor(floor);
    }
}
