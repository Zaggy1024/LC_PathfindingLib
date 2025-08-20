using System;
using System.Diagnostics;

using UnityEngine.AI;

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

    internal static unsafe IntPtr GetOffMeshConnectionFreeList()
    {
        var navMesh = GetNavMesh();

        if (navMesh == IntPtr.Zero)
            return IntPtr.Zero;

        if (IsDebugBuild)
            return navMesh + 0x48;
        return navMesh + 0x40;
    }

    internal static unsafe ref OffMeshConnection GetOffMeshConnection(IntPtr list, uint index)
    {
        ref var asList = ref *(FreeList<OffMeshConnection>*)list;
        if (index >= asList.Capacity)
            throw new IndexOutOfRangeException($"{index} is beyond the capacity of the off-mesh connection freelist {asList.Capacity}.");

        return ref asList.Elements[index];
    }

    internal static unsafe int GetInstanceID(IntPtr obj)
    {
        ref var offset = ref UnityEngine.Object.OffsetOfInstanceIDInCPlusPlusObject;
        if (offset == -1)
            offset = UnityEngine.Object.GetOffsetOfInstanceIDInCPlusPlusObject();
        return *(int*)(obj + offset);
    }

    internal static OffMeshLink GetOffMeshLinkWrapper(int instanceID)
    {
        return OffMeshLinkData.GetOffMeshLinkInternal(instanceID);
    }
}
