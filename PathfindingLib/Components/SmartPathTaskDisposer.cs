using System.Collections.Concurrent;

using UnityEngine;

using PathfindingLib.API.SmartPathfinding;

namespace PathfindingLib.Components;

internal class SmartPathTaskDisposer : MonoBehaviour
{
    private static readonly ConcurrentQueue<SmartPathTask> disposeQueue = [];

    internal static void Enqueue(SmartPathTask task)
    {
        disposeQueue.Enqueue(task);
    }

    private void Update()
    {
        while (disposeQueue.TryDequeue(out var task))
            task.Dispose();
    }
}
