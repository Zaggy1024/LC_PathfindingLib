using UnityEngine;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public interface IInternalTeleport
{
    public Transform Origin { get; }
    public Transform Destination { get; }
    public string Name { get; }
}
