using UnityEngine;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public interface IElevator
{
    public Transform InsideButtonNavMeshNode { get; }
    public bool PointIsInside(Vector3 point);

    public ElevatorFloor ClosestFloor { get; }
    public bool DoorsAreOpen { get; }
    public bool IsAccessibleFromFloor(ElevatorFloor floor)
    {
        return ClosestFloor == floor && DoorsAreOpen;
    }

    public float TraversalCost(ElevatorFloor a, ElevatorFloor b);
    public void GoToFloor(ElevatorFloor floor);
}
