using System;
using System.Threading;

namespace PathfindingLib.Utilities.Internal;

internal class ReadersWriterLock
{
    private readonly object conditionVariable = new();

    private int readersActive = 0;
    private int writersWaiting = 0;
    private bool writerActive = false;

    public void BeginRead()
    {
        lock (conditionVariable)
        {
            while (writersWaiting > 0 || writerActive)
                Monitor.Wait(conditionVariable);
            readersActive++;
        }
    }

    public void EndRead()
    {
        lock (conditionVariable)
        {
            readersActive--;
            if (readersActive < 0)
                throw new InvalidOperationException($"{nameof(EndRead)}() was called more times than {nameof(BeginRead)}.");
            else if (readersActive == 0)
                Monitor.PulseAll(conditionVariable);
        }
    }

    public void BeginWrite()
    {
        lock (conditionVariable)
        {
            writersWaiting++;
            while (readersActive > 0 || writerActive)
                Monitor.Wait(conditionVariable);
            writersWaiting--;
            writerActive = true;
        }
    }

    public void EndWrite()
    {
        lock (conditionVariable)
        {
            if (!writerActive)
                throw new InvalidOperationException($"{nameof(EndWrite)}() was called without first calling {nameof(BeginWrite)}.");
            writerActive = false;
            Monitor.PulseAll(conditionVariable);
        }
    }
}
