using UnityEngine;

namespace PathfindingLib.Jobs;

internal static class SharedJobValues
{
    // Should these be exposed? Hopefully not necessary.
    internal const float MaximumOriginDistance = 5;
    internal static readonly Vector3 OriginExtents = new(MaximumOriginDistance, MaximumOriginDistance, MaximumOriginDistance);

    // Hardcoded values derived from vanilla Lethal Company enemies.
    internal const float MaximumEndpointDistance = 1.5f;
    internal const float MaximumEndpointDistanceSquared = MaximumEndpointDistance * MaximumEndpointDistance;
    internal static readonly Vector3 DestinationExtents = new(MaximumEndpointDistance, MaximumEndpointDistance, MaximumEndpointDistance);
}
