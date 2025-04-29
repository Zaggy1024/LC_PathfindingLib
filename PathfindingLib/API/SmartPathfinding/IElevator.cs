using UnityEngine;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public interface IElevator
{
    public Transform InsideButtonNavMeshNode { get; }

    public ElevatorFloor ClosestFloor { get; }
    public bool DoorsAreOpen { get; }

    public ElevatorFloor? CurrentFloor => DoorsAreOpen ? ClosestFloor : null;
    public ElevatorFloor TargetFloor { get; }

    public float TimeToCompleteCurrentMovement();
    public float TimeFromFloorToFloor(ElevatorFloor a, ElevatorFloor b);
    public void GoToFloor(ElevatorFloor floor);

    public virtual string GetFloorName(ElevatorFloor floor)
    {
        return floor.CallButtonNavMeshNode.name;
    }
}
