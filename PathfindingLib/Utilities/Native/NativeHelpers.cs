using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

using Unity.Collections.LowLevel.Unsafe;

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
        var offset = 0x1D35F18;
        if (IsDebugBuild)
            offset = 0x352D6E0;
        return *(IntPtr*)(BaseAddress + offset);
    }

    internal static unsafe IntPtr GetNavMesh()
    {
        var manager = GetNavMeshManager();
        if (IsDebugBuild)
            return *(IntPtr*)(manager + 0xC0);
        return *(IntPtr*)(manager + 0xA8);
    }

    internal static unsafe IntPtr GetCrowdManager()
    {
        var manager = GetNavMeshManager();
        if (IsDebugBuild)
            return *(IntPtr*)(manager + 0xA8);
        return *(IntPtr*)(manager + 0x90);
    }

    internal static unsafe IntPtr GetNavMeshProjectSettings()
    {
        var offset = 0x1CCBA68;
        if (IsDebugBuild)
            offset = 0x2E08908;
        return *(IntPtr*)(BaseAddress + offset);
    }

    internal static void GetNavMeshAreaNamesArray(out IntPtr address, out int stride)
    {
        var projectSettings = GetNavMeshProjectSettings();
        if (IsDebugBuild)
        {
            address = projectSettings + 0x48;
            stride = 0x38;
            return;
        }

        address = projectSettings + 0x30;
        stride = 0x30;
    }

    internal static unsafe ref FreeList<OffMeshConnection> GetOffMeshConnectionFreeList()
    {
        var offset = 0x80;
        if (IsDebugBuild)
            offset = 0x88;

        return ref *(FreeList<OffMeshConnection>*)(GetNavMesh() + offset);
    }

    internal static unsafe int GetInstanceID(IntPtr obj)
    {
        ref var offset = ref UnityEngine.Object.OffsetOfInstanceIDInCPlusPlusObject;
        if (offset == -1)
            offset = UnityEngine.Object.GetOffsetOfInstanceIDInCPlusPlusObject();
        return *(int*)(obj + offset);
    }

    internal static unsafe ulong GetAgentID(IntPtr agent)
    {
        var offset = 0x60;
        if (IsDebugBuild)
            offset = 0x78;
        return *(ulong*)(agent + offset);
    }

    internal static unsafe string GetBasicString(IntPtr ptr)
    {
        ref var basicString = ref *(BasicStringFields*)ptr;
        byte* data;
        ulong length;
        if (basicString.IsShortString)
        {
            data = (byte*)UnsafeUtility.AddressOf(ref basicString);
            length = (ulong)(0x18 - basicString.ShortStringUnusedBytes);
        }
        else
        {
            data = basicString.Data;
            length = basicString.Length;
        }
        return Encoding.UTF8.GetString(data, (int)Math.Min(length, int.MaxValue));
    }

    internal static IEnumerable<string> GetAreaNames()
    {
        GetNavMeshAreaNamesArray(out var nameAddress, out var nameStride);

        for (var i = 0; i < 32; i++)
        {
            yield return GetBasicString(nameAddress);
            nameAddress += nameStride;
        }
    }

    internal static unsafe void SetBasicString(IntPtr ptr, string str)
    {
        ref var basicString = ref *(BasicStringFields*)ptr;
        var allocUTF8StrLen = Encoding.UTF8.GetByteCount(str);
        var buffer = NativeFunctions.GrowString(ptr, (ulong)allocUTF8StrLen);
        var actualUTF8StrLen = Encoding.UTF8.GetBytes(str.AsSpan(), new Span<byte>(buffer, allocUTF8StrLen));
        if (basicString.IsShortString)
            basicString.ShortStringUnusedBytes = (byte)(0x18 - actualUTF8StrLen);
        else
            basicString.Length = (ulong)actualUTF8StrLen;
    }

    internal static void SetAreaName(int index, string name)
    {
        if (index < 0 || index >= 32)
            throw new IndexOutOfRangeException($"Area index out of range: {index}");
        GetNavMeshAreaNamesArray(out var address, out var stride);
        var nameAddress = address + stride * index;
        SetBasicString(nameAddress, name);
    }
}
