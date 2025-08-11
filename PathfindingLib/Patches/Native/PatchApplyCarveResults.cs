using System;
using System.Runtime.InteropServices;

using MonoMod.RuntimeDetour;

using PathfindingLib.API;
using PathfindingLib.Utilities.Native;

namespace PathfindingLib.Patches.Native;

internal static class PatchApplyCarveResults
{
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private delegate void ApplyCarveResultsDelegate(IntPtr thisNavMeshCarving);

    private static NativeDetour detour;

    private static ApplyCarveResultsDelegate original;

    internal static void Apply(IntPtr baseAddress)
    {
        // This should be set to the offset of NavMeshCarving::ApplyCarveResults(). When loading the
        // UnityPlayer dll, you may see the address including an offset like 0x180000000. If so, subtract
        // the base address (the address of the header) from the address of the function.
        var functionOffset = 0xA64E80UL;
        if (NativeFunctions.IsDebugBuild)
            functionOffset = 0x12C1890UL;
        var functionAddress = (IntPtr)((ulong)baseAddress + functionOffset);

        var hookPtr = Marshal.GetFunctionPointerForDelegate<ApplyCarveResultsDelegate>(ApplyCarveResultsDetour);

        detour = new NativeDetour(functionAddress, hookPtr);
        original = detour.GenerateTrampoline<ApplyCarveResultsDelegate>();
    }

    private static unsafe void ApplyCarveResultsDetour(IntPtr navMeshCarving)
    {
        var hasJobDataToApply = NativeNavMeshUtils.CarvingHasDataToApply(navMeshCarving);

        if (hasJobDataToApply)
            NavMeshLock.BeginWrite();

        original(navMeshCarving);

        if (hasJobDataToApply)
            NavMeshLock.EndWrite();
    }
}
