using System.Threading;

using HarmonyLib;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Experimental.AI;
using UnityEngine.LowLevel;
using UnityEngine.PlayerLoop;

using PathfindingLib.Utilities.Internal;

namespace PathfindingLib.API;

public static class NavMeshLock
{
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
        JobRunCondition.WaitOne();
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

    internal static ReadersWriterLock BlockingLock = new();
    internal static int BlockingLockDepth = 0;
    internal static ManualResetEvent JobRunCondition = new(true);

    private class LockJobs { }

    private class AIUpdateGroup { }

    private class BeforeAIUpdate { }

    private class AfterAIUpdate { }

    internal static bool Initialize(Harmony harmony)
    {
        var loop = PlayerLoop.GetCurrentPlayerLoop();

        if (!SearchAndInjectAIUpdatePrefixAndPostfix(ref loop))
            return false;

        var lockJobs = new PlayerLoopSystem()
        {
            type = typeof(LockJobs),
            updateDelegate = PauseJobsImpl,
        };
        loop.subSystemList = [lockJobs, .. loop.subSystemList];
        PlayerLoop.SetPlayerLoop(loop);

        harmony.PatchAll(typeof(NavMeshLock));

        return true;
    }

    private static bool SearchAndInjectAIUpdatePrefixAndPostfix(ref PlayerLoopSystem currentSubSystem)
    {
        if (currentSubSystem.subSystemList == null)
            return false;

        var index = -1;
        for (var i = 0; i < currentSubSystem.subSystemList.Length; i++)
        {
            if (currentSubSystem.subSystemList[i].type == typeof(PreUpdate.AIUpdate))
            {
                index = i;
                break;
            }
        }

        if (index == -1)
        {
            for (var i = 0; i < currentSubSystem.subSystemList.Length; i++)
            {
                if (SearchAndInjectAIUpdatePrefixAndPostfix(ref currentSubSystem.subSystemList[i]))
                    return true;
            }

            return false;
        }

        ref var subSystems = ref currentSubSystem.subSystemList;

        var prefixSubSystem = new PlayerLoopSystem()
        {
            type = typeof(BeforeAIUpdate),
            updateDelegate = BeforeAIUpdateImpl,
        };
        var postfixSubSystem = new PlayerLoopSystem()
        {
            type = typeof(AfterAIUpdate),
            updateDelegate = AfterAIUpdateImpl,
        };
        var nestedSystem = new PlayerLoopSystem()
        {
            type = typeof(AIUpdateGroup),
            subSystemList = [prefixSubSystem, subSystems[index], postfixSubSystem],
        };

        subSystems[index] = nestedSystem;
        return true;
    }

    private static void PauseJobsImpl()
    {
        JobRunCondition.Reset();
    }

    private static void BeforeAIUpdateImpl()
    {
        BeginWrite();
    }

    private static void AfterAIUpdateImpl()
    {
        EndWrite();
        JobRunCondition.Set();
    }
}
