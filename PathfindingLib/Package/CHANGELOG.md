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
