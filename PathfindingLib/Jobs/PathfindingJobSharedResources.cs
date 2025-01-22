using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Jobs.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Experimental.AI;

namespace PathfindingLib.Jobs;

public static class PathfindingJobSharedResources
{
    /// <summary>
    /// The maximum number of path polygons that can result from path queries calculated using the
    /// per-job-thread <see cref="NavMeshQuery"/> instances.
    /// 
    /// <para>This value is intended to match the <see cref="NavMesh.CalculatePath"/> path size
    /// limitation to ensure that agents are able to navigate the resulting paths.</para>
    /// </summary>
    public const int MaximumPathPolygonCount = 4096;

    [NativeDisableContainerSafetyRestriction] private static NativeArray<NavMeshQuery> StaticThreadQueries;

    /// <summary>
    /// Creates and returns an array containing a unique <see cref="NavMeshQuery"/> instance for each thread
    /// index that a job may be invoked on. Provide this to your jobs and index it using the job thread
    /// index to ensure that no other thread is using the query at the same time.
    /// 
    /// <para>These queries will be initialized with the capacity for <see cref="MaximumPathPolygonCount"/>
    /// polygons per path.</para>
    /// </summary>
    /// <returns>The array of <see cref="NavMeshQuery"/> to provide to a job.</returns>
    public static ref readonly NativeArray<NavMeshQuery> GetPerThreadQueriesArray()
    {
        var threadCount = JobsUtility.ThreadIndexCount;
        if (StaticThreadQueries.Length >= threadCount)
            return ref StaticThreadQueries;

        Application.quitting -= DisposeQueries;

        var newQueries = new NativeArray<NavMeshQuery>(threadCount, Allocator.Persistent);
        for (var i = 0; i < StaticThreadQueries.Length; i++)
            newQueries[i] = StaticThreadQueries[i];
        for (var i = StaticThreadQueries.Length; i < threadCount; i++)
            newQueries[i] = new NavMeshQuery(NavMeshWorld.GetDefaultWorld(), Allocator.Persistent, MaximumPathPolygonCount);
        StaticThreadQueries.Dispose();
        StaticThreadQueries = newQueries;

        Application.quitting += DisposeQueries;

        return ref StaticThreadQueries;
    }

    private static void DisposeQueries()
    {
        foreach (var query in StaticThreadQueries)
            query.Dispose();

        StaticThreadQueries.Dispose();

        Application.quitting -= DisposeQueries;
    }
}
