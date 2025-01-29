using System;

using PathfindingLib.API;

namespace PathfindingLib.Utilities;

public struct NavMeshReadLocker : IDisposable
{
    private bool locked = false;

    /// <summary>
    /// Creates an instance that locks the navmesh for reading until it is disposed.
    /// 
    /// <para>Intended to be disposed automatically through a <c>using</c> statement. The lock
    /// will only be released one time, even if the instance is disposed early.</para>
    /// </summary>
    public NavMeshReadLocker()
    {
        NavMeshLock.BeginRead();
        locked = true;
    }

    /// <summary>
    /// Yields execution to any waiting writers. See <see cref="NavMeshLock.YieldRead"/>.
    /// </summary>
    public void Yield()
    {
        NavMeshLock.YieldRead();
    }

    /// <summary>
    /// Releases the read lock, and prevents future calls to <see cref="Dispose"/> from having
    /// any effect.
    /// </summary>
    public void Dispose()
    {
        if (!locked)
            return;
        locked = false;
        NavMeshLock.EndRead();
    }
}
