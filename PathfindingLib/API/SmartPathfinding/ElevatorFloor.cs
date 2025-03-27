﻿using UnityEngine;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public class ElevatorFloor(IElevator elevator, Transform buttonPosition)
{
    public IElevator Elevator { get; } = elevator;
    public Transform CallButtonNavMeshNode { get; } = buttonPosition;
    public bool IsElevatorAccessible()
    {
        return Elevator.IsAccessibleFromFloor(this);
    }
    public void CallElevator()
    {
        Elevator.GoToFloor(this);
    }
    public override string ToString()
    {
        return $"{Elevator} Floor @ {CallButtonNavMeshNode}";
    }
}
