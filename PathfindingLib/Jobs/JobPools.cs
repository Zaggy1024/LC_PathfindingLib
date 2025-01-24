using System.Diagnostics;

using UnityEngine.Pool;

namespace PathfindingLib.Jobs;

public class PooledFindPathJob
{
    private FindPathJob job = new();

    public ref FindPathJob Job => ref job;
}

public static class JobPools
{
    private static readonly ObjectPool<PooledFindPathJob> findPathPool = new(() => new(), actionOnDestroy: j => j.Job.Dispose());

    /// <summary>
    /// Gets a pooled instance of <see cref="FindPathJob"/> stored in a reference type to allow it to be freed later.
    /// </summary>
    /// <returns>A wrapper containing a <see cref="FindPathJob"/>.</returns>
    public static PooledFindPathJob GetFindPathJob()
    {
        return findPathPool.Get();
    }

    /// <summary>
    /// Returns a pooled instance of <see cref="FindPathJob"/> back to the pool.
    /// </summary>
    /// <param name="job">A pooled instance of <see cref="FindPathJob"/> previously provided by <see cref="GetFindPathJob"/>.</param>
    public static void ReleaseFindPathJob(PooledFindPathJob job)
    {
        if (job == null)
        {
            PathfindingLibPlugin.Instance.Logger.LogError($"Attempted to free a null job\n{new StackTrace()}");
            return;
        }

        findPathPool.Release(job);
    }
}
