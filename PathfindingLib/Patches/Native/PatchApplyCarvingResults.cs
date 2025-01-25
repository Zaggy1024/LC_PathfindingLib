using System;
using System.Runtime.InteropServices;

using MonoMod.RuntimeDetour;

using PathfindingLib.API;

namespace PathfindingLib.Patches.Native;

internal static class PatchApplyCarvingResults
{
    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    private delegate void ApplyCarvingResultsDelegate(IntPtr thisNavMeshCarving);

    private readonly static bool debug = UnityEngine.Debug.isDebugBuild;

    private static NativeDetour detour;

    private static ApplyCarvingResultsDelegate original;

    internal static void Apply(IntPtr baseAddress)
    {
        // This should be set to the offset of NavMeshCarving::ApplyCarvingResults(). When loading the
        // UnityPlayer dll, you may see the address including an offset like 0x180000000. If so, subtract
        // the base address (the address of the header) from the address of the function.
        var functionOffset = 0xA64E80UL;
        if (debug)
            functionOffset = 0x12C1890UL;
        var functionAddress = (IntPtr)((ulong)baseAddress + functionOffset);

        var hookPtr = Marshal.GetFunctionPointerForDelegate<ApplyCarvingResultsDelegate>(ApplyCarvingResultsDetour);

        detour = new NativeDetour(functionAddress, hookPtr);
        original = detour.GenerateTrampoline<ApplyCarvingResultsDelegate>();
    }

    private unsafe static void ApplyCarvingResultsDetour(IntPtr thisNavMeshCarving)
    {
        // Within NavMeshCarving::ApplyCarvingResults(), these offsets are derived from a statement
        // that decompiles similarly to the following in a debug build:
        //   auto var = (*(longlong*)(this + 0x40) - *(longlong*)(this + 0x38)) / 0x50;
        // Note that the division by 0x50 may be decompiled instead as a bit shift right followed by an
        // integer multiplication. This is a compiler optimization that doesn't get fully reversed, likely
        // due to the compiler assuming that those pointers are aligned to a multiple of the divisor.
        //
        // In a more human-readable form, this would be equivalent to:
        //   auto jobCount = this->m_JobDataVector - this->m_JobDataVectorEnd;
        // The function call when resizing m_JobDataVector appears to indicate it to be the beginning of
        // an std::vector<> with an element type of CarveData. Therefore, the statement would really be:
        //   auto jobCount = this->m_JobData.size();
        //
        // In order to determine whether there are any jobs in flight, we first extract the pointer to
        // the start and end of the valid data, then check if the end is greater than the start.
        var jobData = 0UL;
        var jobDataEnd = 0UL;
        if (debug)
        {
            jobData = *(ulong*)((ulong)thisNavMeshCarving + 0x38);
            jobDataEnd = *(ulong*)((ulong)thisNavMeshCarving + 0x40);
        }
        else
        {
            jobData = *(ulong*)((ulong)thisNavMeshCarving + 0x28);
            jobDataEnd = *(ulong*)((ulong)thisNavMeshCarving + 0x30);
        }

        var hasJobDataToApply = jobDataEnd > jobData;

        if (hasJobDataToApply)
            NavMeshLock.BeginWrite();

        original(thisNavMeshCarving);

        if (hasJobDataToApply)
            NavMeshLock.EndWrite();
    }
}
