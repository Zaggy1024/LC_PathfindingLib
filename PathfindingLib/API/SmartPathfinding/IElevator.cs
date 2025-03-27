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

    public ElevatorFloor? CurrentFloor => DoorsAreOpen ? ClosestFloor : null;

    public float CostToRideElevatorFromCurrentFloor(ElevatorFloor floor);
    public float CostToTraverseElevator(ElevatorFloor a, ElevatorFloor b);
    public void GoToFloor(ElevatorFloor floor);
}
