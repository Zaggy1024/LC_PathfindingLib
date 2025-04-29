using UnityEngine;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public sealed class ElevatorFloor(IElevator elevator, Transform buttonPosition)
{
    public IElevator Elevator { get; } = elevator;
    public Transform CallButtonNavMeshNode { get; } = buttonPosition;
    public void CallElevator()
    {
        Elevator.GoToFloor(this);
    }
    public override string ToString()
    {
        return $"{Elevator} Floor ({Elevator.GetFloorName(this)})";
    }
}
