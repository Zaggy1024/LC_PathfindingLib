using System;
using System.Threading;

using UnityEngine.Experimental.AI;

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
        ref var state = ref threadState;

        // If this thread has started a read, we disallow starting a write.
        if (state < 0)
            throw CreateAndPrintInvalidOperationException("Cannot begin a navmesh write while a read is ongoing.");

        // If this thread was already writing, count the recursion and skip the write lock.
        if (state++ > 0)
            return;

        lock (conditionVariable)
        {
            writersWaiting++;
            while (readersActive > 0 || writerActive)
                Monitor.Wait(conditionVariable);
            writersWaiting--;
            writerActive = true;
        }
    }

    /// <summary>
    /// This should be called after any sequence of calls to methods that modify the navmesh.
    /// 
    /// <para>See <see cref="BeginWrite"/> for more information.</para>
    /// </summary>
    public static void EndWrite()
    {
        ref var state = ref threadState;

        // If this thread is not writing, we cannot release the write lock, so throw an exception.
        if (state <= 0)
            throw CreateAndPrintInvalidOperationException("A navmesh write has not been started.");

        // If this thread is still writing after removing from the recursion, keep the write lock.
        if (--state > 0)
            return;

        lock (conditionVariable)
        {
            if (!writerActive)
                throw CreateAndPrintInvalidOperationException($"{nameof(EndWrite)}() was called without first calling {nameof(BeginWrite)}.");
            writerActive = false;
            Monitor.PulseAll(conditionVariable);
        }
    }

    /// <summary>
    /// Call this before any usage of NavMeshQuery off the main thread to avoid crashes when the
    /// navmesh is modified during queries.
    /// 
    /// <para>This must be followed by a call to <see cref="EndRead"/>,
    /// or the main thread will deadlock and the game will be frozen permanently. It may be called
    /// between calls to <see cref="NavMeshQuery.UpdateFindPath"/></para>
    /// </summary>
    public static void BeginRead()
    {
        ref var state = ref threadState;

        // If this thread is writing, don't allow starting a read within it.
        if (state > 0)
            throw CreateAndPrintInvalidOperationException("Cannot begin a navmesh read while a write is ongoing.");

        // If this thread was already reading, count the recursion and skip the read lock.
        if (state-- < 0)
            return;

        lock (conditionVariable)
        {
            while (writersWaiting > 0 || writerActive)
                Monitor.Wait(conditionVariable);
            readersActive++;
        }
    }

    /// <summary>
    /// Call this after any usage of NavMeshQuery off the main thread to avoid crashes when the
    /// navmesh is modified during queries.
    /// 
    /// <para>See <see cref="BeginRead">BeginNavMeshRead</see> for more information.</para>
    /// </summary>
    public static void EndRead()
    {
        ref var state = ref threadState;

        // If this thread is not reading, we cannot release the read lock, so throw an exception.
        if (state >= 0)
            throw CreateAndPrintInvalidOperationException("A navmesh read has not been started.");

        // If this thread is still reading after removing from the recursion, keep the read lock.
        if (++state < 0)
            return;

        lock (conditionVariable)
        {
            readersActive--;
            if (readersActive < 0)
                throw CreateAndPrintInvalidOperationException($"{nameof(EndRead)}() was called more times than {nameof(BeginRead)}.");
            else if (readersActive == 0)
                Monitor.PulseAll(conditionVariable);
        }
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

    [ThreadStatic]
    private static int threadState = 0;

    private static readonly object conditionVariable = new();

    private static int readersActive = 0;
    private static int writersWaiting = 0;
    private static bool writerActive = false;

    private static InvalidOperationException CreateAndPrintInvalidOperationException(string message)
    {
        PathfindingLibPlugin.Instance.Logger.LogError(message);
        PathfindingLibPlugin.Instance.Logger.LogError("Check Unity's Player.log to see the exception's stack trace.");
        return new InvalidOperationException(message);
    }
}
