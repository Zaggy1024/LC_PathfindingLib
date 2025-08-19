using System;
using System.Diagnostics;

namespace PathfindingLib.Utilities.Native;

internal static class NativeHelpers
{
    internal readonly static IntPtr BaseAddress = GetUnityPlayerModule().BaseAddress;
    internal readonly static bool IsDebugBuild = UnityEngine.Debug.isDebugBuild;

    private static ProcessModule GetUnityPlayerModule()
    {
        var modules = Process.GetCurrentProcess().Modules;
        for (var i = 0; i < modules.Count; i++)
        {
            var module = modules[i];
            if (module.ModuleName.Contains("UnityPlayer"))
                return module;
        }

        return null;
    }

    // Returned by GetNavMeshManager(), which is called by OffMeshLink::AddConnection().
    internal static unsafe IntPtr GetNavMeshManager()
    {
        var offset = 0x1CEA688;
        if (IsDebugBuild)
            offset = 0x2E085E0;
        return *(IntPtr*)(BaseAddress + offset);
    }

    internal static unsafe IntPtr GetNavMesh()
    {
        var manager = GetNavMeshManager();
        if (IsDebugBuild)
            return *(IntPtr*)(manager + 0xC0);
        return *(IntPtr*)(manager + 0xA8);
    }
}
