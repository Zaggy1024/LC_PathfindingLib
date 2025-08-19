using System;
using System.Runtime.InteropServices;

using MonoMod.RuntimeDetour;
using Unity.AI.Navigation;

using PathfindingLib.API;
using PathfindingLib.Utilities.Native;

namespace PathfindingLib.Patches.Native;

internal static class PatchNavMeshManagerUpdate
{
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private delegate void UpdateDelegate(IntPtr navMeshManager);

    private static NativeDetour detour;

    private static UpdateDelegate original;

    internal static void Apply()
    {
        var functionOffset = 0xA29B00;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x1284E80;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        var hookPtr = Marshal.GetFunctionPointerForDelegate<UpdateDelegate>(UpdateDetour);

        detour = new NativeDetour(functionAddress, hookPtr);
        original = detour.GenerateTrampoline<UpdateDelegate>();
    }

    private static unsafe IntPtr GetNavMeshCarving(IntPtr navMeshManager)
    {
        if (NativeHelpers.IsDebugBuild)
            return *(IntPtr*)(navMeshManager + 0xB0);
        return *(IntPtr*)(navMeshManager + 0x98);
    }

    private static bool WillNavMeshBeModifiedThisUpdate(IntPtr navMeshManager)
    {
        if (NativeNavMeshUtils.CarvingHasDataToApply(GetNavMeshCarving(navMeshManager)))
            return true;

        foreach (var surface in NavMeshSurface.s_NavMeshSurfaces)
        {
            if (surface.HasTransformChanged())
                return true;
        }

        foreach (var link in NativeNavMeshUtils.GetOffMeshLinks(navMeshManager))
        {
            if (NativeNavMeshUtils.OffMeshLinkWillUpdate(link))
                return true;
        }

        foreach (var link in NavMeshLink.s_Tracked)
        {
            if (link.HasTransformChanged())
                return true;
        }

        return false;
    }

    private static unsafe void UpdateDetour(IntPtr navMeshManager)
    {
        var shouldLock = WillNavMeshBeModifiedThisUpdate(navMeshManager);

        if (shouldLock)
            NavMeshLock.BeginWrite();

        original(navMeshManager);

        if (shouldLock)
            NavMeshLock.EndWrite();
    }
}
