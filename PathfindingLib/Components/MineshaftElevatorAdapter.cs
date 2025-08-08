using UnityEngine;

using PathfindingLib.API.SmartPathfinding;

namespace PathfindingLib.Components;

public sealed class MineshaftElevatorAdapter : MonoBehaviour, IElevator
{
    public MineshaftElevatorController controller;

    private Collider insideCollider;

    private Transform insideButton;

    private Transform topCallButton;
    private Transform bottomCallButton;

    private ElevatorFloor topFloor;
    private ElevatorFloor bottomFloor;

    private void Start()
    {
        var physicsRegion = controller.GetComponentInChildren<PlayerPhysicsRegion>();
        if (physicsRegion != null)
            insideCollider = physicsRegion.physicsCollider;

        insideButton = controller.elevatorInsidePoint;

        topCallButton = controller.elevatorTopPoint;
        bottomCallButton = controller.elevatorBottomPoint;

        topFloor = new(this, topCallButton);
        bottomFloor = new(this, bottomCallButton);

        if (isActiveAndEnabled)
            OnEnable();
    }

    private void OnEnable()
    {
        if (topFloor == null || bottomFloor == null)
            return;
        PathfindingLibPlugin.Instance.Logger.LogInfo($"Registering {this}");
        SmartPathfinding.RegisterElevatorFloor(topFloor);
        SmartPathfinding.RegisterElevatorFloor(bottomFloor);
    }

    private void OnDisable()
    {
        PathfindingLibPlugin.Instance.Logger.LogInfo($"Unregistering {this}");
        SmartPathfinding.UnregisterElevatorFloor(topFloor);
        SmartPathfinding.UnregisterElevatorFloor(bottomFloor);
    }

    public Transform InsideButtonNavMeshNode => insideButton;

    public ElevatorFloor ClosestFloor
    {
        get
        {
            var position = controller.elevatorPoint.position;
            var topDistance = (position - topCallButton.position).sqrMagnitude;
            var bottomDistance = (position - bottomCallButton.position).sqrMagnitude;
            if (topDistance < bottomDistance)
                return topFloor;
            return bottomFloor;
        }
    }

    public bool DoorsAreOpen => controller.elevatorFinishedMoving && controller.elevatorDoorOpen;
    public ElevatorFloor TargetFloor => controller.elevatorMovingDown ? bottomFloor : topFloor;

    public void GoToFloor(ElevatorFloor floor)
    {
        bool callDown;
        if (floor == bottomFloor)
            callDown = true;
        else if (floor == topFloor)
            callDown = false;
        else
            return;

        if (controller.elevatorCalled || controller.elevatorMovingDown == callDown)
            return;

        controller.CallElevator(callDown);
    }

    public bool IsInsideElevator(Vector3 point)
    {
        if (insideCollider == null)
            return (insideButton.position - point).sqrMagnitude <= 1f;

        return insideCollider.ClosestPoint(point) == point;
    }

    public float TimeToCompleteCurrentMovement()
    {
        if (DoorsAreOpen)
            return 0f;
        return Vector3.Distance(insideButton.position, TargetFloor.CallButtonNavMeshNode.position) / 4f;
    }

    public float TimeFromFloorToFloor(ElevatorFloor a, ElevatorFloor b)
    {
        if (a == b)
            return 0f;
        const float buttonDelay = 4f;
        const float goDownTime = 8.67f;
        const float goUpTime = 11.92f;
        return buttonDelay + (controller.elevatorMovingDown ? goDownTime : goUpTime);
    }
}
