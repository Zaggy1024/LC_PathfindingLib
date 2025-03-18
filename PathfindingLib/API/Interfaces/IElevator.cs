using PathfindingLib.API.SmartPathfinding;
using UnityEngine;

namespace PathfindingLib.API.Interfaces;

#nullable enable

public interface IElevator
{
    public Transform CenterNavMeshNode { get; }
    public Bounds CurrentInsideBounds { get; }

    public bool IsMoving { get; }

    public ElevatorFloor ClosestFloor { get; }
    public bool IsAccessibleOnClosestFloor { get; }
    public bool IsAccessibleFromFloor(ElevatorFloor floor)
    {
        return ClosestFloor == floor && IsAccessibleOnClosestFloor;
    }

    public float TraversalCost(ElevatorFloor a, ElevatorFloor b);
    public void GoToFloor(ElevatorFloor floor);
}
