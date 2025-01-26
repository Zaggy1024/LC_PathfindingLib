using System;
using System.Runtime.CompilerServices;

using Unity.Collections.LowLevel.Unsafe;
using Unity.Profiling.LowLevel.Unsafe;
using Unity.Profiling;
using UnityEngine.Scripting;

namespace PathfindingLib.Utilities;

[IgnoredByDeepProfiler]
[UsedByNativeCode]
public struct TogglableProfilerAuto : IDisposable
{
    [NativeDisableUnsafePtrRestriction]
    internal readonly IntPtr ptr;
    internal bool on;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public TogglableProfilerAuto(in ProfilerMarker marker)
    {
        ptr = marker.m_Ptr;
        on = true;
        ProfilerUnsafeUtility.BeginSample(ptr);
    }

    public void Pause()
    {
        if (on)
        {
            ProfilerUnsafeUtility.EndSample(ptr);
            on = false;
        }
    }

    public void Resume()
    {
        if (!on)
        {
            on = true;
            ProfilerUnsafeUtility.BeginSample(ptr);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Dispose()
    {
        if (on)
        {
            ProfilerUnsafeUtility.EndSample(ptr);
            on = false;
        }
    }
}
