using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Experimental.AI;

using PathfindingLib.Utilities.Internal;

namespace PathfindingLib.API;

public static class NavMeshLock
{
    private const int recommendedSliceCount = 128;

    /// <summary>
    /// The recommended number of iterations that should be run per call to NavMeshQueryUpdateFindPath()
    /// to avoid delaying the main thread when it tries to take a write lock.
    /// </summary>
    public static int RecommendedUpdateFindPathIterationCount => recommendedSliceCount;

    /// <summary>
    /// This should be called before any sequence of calls to methods that modify the navmesh.
    /// 
    /// <para>This must be followed by a call to <see cref="EndWrite"/> or the
    /// job threads trying to read from the navmesh will deadlock.</para>
    /// 
    /// <para>Most if not all cases of the navmesh being modified are already hooked to call this to ensure
    /// safety when doing threaded pathfinding, but this can be called recursively to avoid extra work
    /// off taking and releasing the lock repeatedly.</para>
    /// </summary>
    public static void BeginWrite()
    {
        Assert.IsTrue(Object.CurrentThreadIsMainThread());
        if (BlockingLockDepth++ == 0)
            BlockingLock.BeginWrite();
    }

    /// <summary>
    /// This should be called after any sequence of calls to methods that modify the navmesh.
    /// 
    /// <para>See <see cref="BeginWrite"/> for more information.</para>
    /// </summary>
    public static void EndWrite()
    {
        Assert.IsTrue(Object.CurrentThreadIsMainThread());
        if (--BlockingLockDepth == 0)
            BlockingLock.EndWrite();
    }

    /// <summary>
    /// Call this before any usage of NavMeshQuery off the main thread to avoid crashes when the
    /// navmesh is modified during queries.
    /// 
    /// <para>This must be followed by a call to <see cref="EndRead"/>,
    /// or the main thread will deadlock and the game will be frozen permanently. It may be called
    /// between calls to <see cref="NavMeshQuery.UpdateFindPath"/></para>
    /// 
    /// <para>This method will block at the start of every frame in anticipation of the navmesh being
    /// updated during the AIUpdate subsystem. The navmesh is not guaranteed to be modified during
    /// that part of the player loop, but there is no way to detect when holes carved by obstacles
    /// get updated in the navmesh, so it is defensively blocked until that subsystem finishes.</para>
    /// </summary>
    public static void BeginRead()
    {
        BlockingLock.BeginRead();
    }

    /// <summary>
    /// Call this after any usage of NavMeshQuery off the main thread to avoid crashes when the
    /// navmesh is modified during queries.
    /// 
    /// <para>See <see cref="BeginRead">BeginNavMeshRead</see> for more information.</para>
    /// </summary>
    public static void EndRead()
    {
        BlockingLock.EndRead();
    }

    /// <summary>
    /// Call this to check if the main thread is waiting to write to the navmesh, and pause execution
    /// until the write finishes. Must be called with a read lock held.
    /// </summary>
    public static void YieldRead()
    {
        EndRead();
        BeginRead();
    }

    internal static ReadersWriterLock BlockingLock = new();
    internal static int BlockingLockDepth = 0;
}
