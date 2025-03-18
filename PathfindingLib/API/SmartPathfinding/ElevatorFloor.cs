using PathfindingLib.API.Interfaces;
using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public class ElevatorFloor(IElevator elevator, Transform buttonPosition)
{
    public IElevator Elevator { get; } = elevator;
    public Transform ButtonNavMeshNode { get; } = buttonPosition;
    public bool IsElevatorAccessible()
    {
        return Elevator.IsAccessibleFromFloor(this);
    }
    public void CallElevator()
    {
        Elevator.GoToFloor(this);
    }
}
