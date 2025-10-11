## Version 2.4.1
- Fixed a crash that could occur, usually during ship landing, with `PatchOffMeshConnectionStutterStepping` enabled.

## Version 2.4.0
- Updated native hooks to support Unity 2022.3.62f2 (Lethal Company v73). This breaks compatibility with previous versions.

## Version 2.3.2
- Prevented an exception in `SmartRoaming` if the agent is not on the navmesh.

## Version 2.3.1
- Made SmartRoaming use `AgentExtensions.GetPathOrigin()` to determine the start point of its paths.
- Fixed an exception that could occur in `SmartRoaming` if a path from an agent succeeds, while a path from the search start point fails.
- Set the non-smart agent navmesh area type name so that `NavMesh.GetAreaFromName("NonSmartAgent")` returns 25.

## Version 2.3.0
- Changed `AgentExtensions.GetPathOrigin()` to get the path origin for `NavMeshAgent`s more accurately. This will avoid issues with pathfinding on agents with a non-zero `baseOffset`, such as the [Soul Devourer](https://thunderstore.io/c/lethal-company/p/bcs4313/Soul_Devourer_Enemy/) enemy.
- Added the `AgentExtensions.GetQueryFilter()` method to retrieve an agent's type ID, area mask, and cost overrides in one call. Previously, the cost overrides were largely unusable due to requiring 32 individual calls to `NavMeshAgent.GetAreaCost()`.

## Version 2.2.2
- Fixed an exception that would occur when `SmartRoaming` is sending an enemy to a search node.
- Added `SmartPathTask.IsStarted` to determine whether `StartPathTask()` has been called.
- Ensured that `SmartRoaming` will not throw an exception if it runs out of nodes to search.

## Version 2.2.1
- Made `SmartPathTask.StartPathTask()` throw an exception if it is called with zero destinations. All path results will also throw exceptions if the task has not been started. This should help to avoid situations where `IsResultReady()` could always return false, blocking further navigation logic from ever running.

## Version 2.2.0
- Added patches to prevent auto-updating OffMeshLinks and NavMeshLinks from causing stutter stepping when they move.
- Made waiting for/in elevators not count towards the time spent walking towards a node in the smart search routine.

## Version 2.1.3
- Corrected the search area when mapping path origins and destinations onto the navmesh to match `NavMesh.CalculatePath()`.

## Version 2.1.2
- Fixed a crash with off-mesh links that are missing one or both endpoint transforms. This was causing a crash upon landing on Black Mesa.

## Version 2.1.1
- Re-release of 2.1.0 without debug functionality.

## Version 2.1.0
- Fixed a crash on Bozoros caused by moving NavMeshLinks.

## Version 2.0.0
- Added the `IElevator.IsInsideElevator(Vector3)` to determine if a point is inside the physical bounds of the elevator, which is required to be implemented for all registered elevators.
- Added the `SmartPathDestination.CanActivateDestination(Vector3)` method which determines whether an agent at that position can activate/use the destination. This is intended to prevent agents from pressing the button to send an elevator to a floor without being inside first, preventing AI from getting stuck repeatedly hitting elevator buttons.
- Allowed users of the `SmartRoaming` API to customize the distance that an agent is allowed to spend navigating to a node.

## Version 1.0.1
- Fixed errors that could occur if a `SmartPathTask` was allowed to be garbage collected instead of being `Dispose()`d.

## Version 1.0.0
- New APIs:
    - `SmartPathfinding`: A static class with methods to register smart agents, elevators, and teleporters for the smart pathfinding system.
    - `IElevator`: An interface that can be implemented to allow enemies to navigate through them if they use `SmartPathTask`.
    - `SmartPathTask`: A class to calculate paths to multiple destinations either directly or through entrance teleports and elevators.
    - `SmartRoaming`: A static class that reimplements the vanilla AI search routine using `SmartPathTask` to allow enemies that choose to use it to roam through elevators.
- Wiki pages will be added to the GitHub for usage instructions later!

## Version 0.1.1
- Fixed an issue that would cause partial paths passed to `NavMeshQueryUtils.FindStraightPath` to result in a final corner at the destination.
- Changed the new overloads of `NavMeshQueryUtils.FindStraightPath` to use `Vector3` instead of `float3`, as the Unity runtime assumes the memory layout matches `Vector3` internally.

## Version 0.1.0
- Made `NavMeshQueryUtils.FindStraightPath` call through to the Unity runtime instead of reimplementing the algorithm. The results should now match the output from `NavMeshPath.corners` perfectly.
- Deprecated the original overload of `NavMeshQueryUtils.FindStraightPath` in favor of one that requires less allocations to match the native Unity function's signature.

## Version 0.0.14
- Fixed a small leak of single-element arrays in `FindPathJob`.

## Version 0.0.13
- Reduced allocations by deprecating the reference-type `NavMeshReadLocker` in favor of a new value-type version in the `Utilities` namespace.

## Version 0.0.12
- Added some more flexible safeties to `NavMeshLock` to help avoid deadlocks.

## Version 0.0.11
- Reverted unreliable `NavMeshLock` safeties that were causing exceptions when apparently taking read locks on the main thread.

## Version 0.0.10
- Fixed pathfinding jobs not functioning properly in release builds.

## Version 0.0.9
- Added some extra checks to help ensure `NavMeshLock` that is used safely.
- Made `TogglableProfilerAuto` methods public.

## Version 0.0.8
- Fixed an issue where `FindPathJob` was not taking the read lock at the start of the job, but would later take the lock without releasing it, which could result in deadlocks.

## Version 0.0.7
- Reduced blocking of the main thread by hooking into the Unity runtime to detect when carving obstacles will make changes to the navmesh.
- Changed documentation to recommend using `NavMeshQuery.UpdateFindPath()` with an iteration limit, and unlocking the navmesh read between calls.

## Version 0.0.6
- Prevented API users releasing null `PooledFindPathJob` back to the pool to avoid null error spam.

## Version 0.0.5
- Reverted an unintentional change to the plugin's GUID string.

## Version 0.0.4
- Made the plugin GUID public for convenient hard dependency setup.

## Version 0.0.3
- Renamed the Plugin class to PathfindingLibPlugin.

## Version 0.0.2
- Replaced the icon with a new placeholder that will totally not stay indefinitely...

## Version 0.0.1
Initial version. Public-facing API includes:
- `FindPathJob`: A simple job to find a valid path for an agent to traverse between a start and end position.
- `JobPools`: A static class providing pooled `FindPathJob` instances that can be reused by any API users.
- `NavMeshLock`: Provides methods to prevent crashes when running pathfinding off the main thread.
- `PathfindingJobSharedResources`: A static class that provides a `NativeArray<NavMeshQuery>` that can be passed to a job to access a thread-specific instance of `NavMeshQuery`.
- `AgentExtensions.GetAgentPathOrigin(this NavMeshAgent)`: Gets the position that paths originating from an agent should start from. This avoids pathing failure when crossing links.
- `Pathfinding.FindStraightPath(...)`: Gets a straight path from the result of a NavMeshQuery.
