using System;

namespace PathfindingLib.API.SmartPathfinding;

[Flags]
public enum SmartPathfindingLinkFlags
{
    InternalTeleports = 1 << 0,
    Elevators = 1 << 1,
    MainEntrance = 1 << 2,
    FireExits = 1 << 3,
}
