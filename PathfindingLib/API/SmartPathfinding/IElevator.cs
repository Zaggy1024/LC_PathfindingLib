using UnityEngine;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public interface IElevator
{
    /// <summary>
    /// The transform that agents should path to in order to ride the elevator to another floor.
    /// </summary>
    public Transform InsideButtonNavMeshNode { get; }

    /// <summary>
    /// Points to the floor that is closest to the inside of the elevator in the current frame.
    /// </summary>
    public ElevatorFloor ClosestFloor { get; }
    /// <summary>
    /// Whether an agent can path to the inside of the elevator without being interrupted due to
    /// the elevator completing its motion to arrive at the target floor.
    /// </summary>
    public bool DoorsAreOpen { get; }

    /// <summary>
    /// The floor at which agents can path to the inside of the elevator, based on the requirements
    /// of <see cref="DoorsAreOpen"/>.
    /// </summary>
    public ElevatorFloor? CurrentFloor => DoorsAreOpen ? ClosestFloor : null;
    /// <summary>
    /// The floor that the elevator is currently traveling to.
    /// </summary>
    public ElevatorFloor TargetFloor { get; }

    /// <summary>
    /// Gets the time that it will take the elevator to arrive at its target floor.
    /// </summary>
    /// <returns>The time in seconds.</returns>
    public float TimeToCompleteCurrentMovement();
    /// <summary>
    /// Gets the time that it will take the elevator to move from floor to floor.
    /// </summary>
    /// <param name="a">The floor that the elevator is moving from.</param>
    /// <param name="b">The floor that the elevator is moving to.</param>
    /// <returns>The time in seconds.</returns>
    public float TimeFromFloorToFloor(ElevatorFloor a, ElevatorFloor b);
    /// <summary>
    /// Sends the elevator to the specified floor. This method will be called repeatedly when
    /// an agent is using smart pathfinding to navigate the elevator.
    /// </summary>
    /// <param name="floor">The floor to send the elevator to.</param>
    public void GoToFloor(ElevatorFloor floor);

    /// <summary>
    /// Gets the display name for the provided floor in this elevator for the purposes of
    /// debugging. By default, this will be the name of the call button at that floor.
    /// </summary>
    /// <param name="floor">The floor to get the display name of.</param>
    /// <returns>The display name of the floor.</returns>
    public virtual string GetFloorName(ElevatorFloor floor)
    {
        return floor.CallButtonNavMeshNode.name;
    }
}
