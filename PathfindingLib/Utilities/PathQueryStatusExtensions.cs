using UnityEngine.Experimental.AI;

namespace PathfindingLib.Utilities;

public static class PathQueryStatusExtensions
{
    /// <summary>
    /// Gets the part of the status not including the detail flags of the status. This can be used to check
    /// if the status indicates that it is in progress, failed or completed.
    /// </summary>
    /// <param name="status">The <see cref="PathQueryStatus"/> to retrieve the result from.</param>
    /// <returns>The part of the status not including the detail flags.</returns>
    public static PathQueryStatus GetResult(this PathQueryStatus status)
    {
        return status & ~PathQueryStatus.StatusDetailMask;
    }

    /// <summary>
    /// Gets the detail part of the status. This can sometimes be used to determine the reason for a pathing
    /// failure.
    /// </summary>
    /// <param name="status">The <see cref="PathQueryStatus"/> to retrieve the detail flags from.</param>
    /// <returns>The detail flags of the status.</returns>
    public static PathQueryStatus GetDetail(this PathQueryStatus status)
    {
        return status & PathQueryStatus.StatusDetailMask;
    }
}
