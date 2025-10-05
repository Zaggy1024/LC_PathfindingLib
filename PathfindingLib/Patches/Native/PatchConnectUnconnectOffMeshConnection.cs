using System;
using System.Runtime.InteropServices;

using MonoMod.RuntimeDetour;

using PathfindingLib.API;
using PathfindingLib.Utilities.Native;

namespace PathfindingLib.Patches.Native;

internal static class PatchConnectUnconnectOffMeshConnection
{
    internal static void Apply()
    {
        var baseAddress = NativeHelpers.BaseAddress;
        SetUpConnectOffMeshConnectionDetour();
        SetUpUnconnectOffMeshConnectionDetour();
    }

    // NavMesh::ConnectOffMeshConnection()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private delegate void ConnectOffMeshConnectionDelegate(IntPtr navMesh, uint index, float rangeX, float rangeY);

    private static NativeDetour connectOffMeshConnectionDetour;

    private static ConnectOffMeshConnectionDelegate connectOffMeshConnectionOriginal;

    private static void SetUpConnectOffMeshConnectionDetour()
    {
        var functionOffset = 0xA78840;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x12F8B60;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        var hookPtr = Marshal.GetFunctionPointerForDelegate<ConnectOffMeshConnectionDelegate>(ConnectOffMeshConnectionDetour);

        connectOffMeshConnectionDetour = new NativeDetour(functionAddress, hookPtr);
        connectOffMeshConnectionOriginal = connectOffMeshConnectionDetour.GenerateTrampoline<ConnectOffMeshConnectionDelegate>();
    }

    private static void ConnectOffMeshConnectionDetour(IntPtr navMesh, uint index, float rangeX, float rangeY)
    {
        NavMeshLock.BeginWrite();
        connectOffMeshConnectionOriginal(navMesh, index, rangeX, rangeY);
        NavMeshLock.EndWrite();
    }

    internal static void ConnectOffMeshConnection(IntPtr navMesh, uint index, float rangeX, float rangeY)
    {
        connectOffMeshConnectionOriginal(navMesh, index, rangeX, rangeY);
    }

    // NavMesh::UnconnectOffMeshConnection()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private delegate void UnconnectOffMeshConnectionDelegate(IntPtr navMesh, uint index);

    private static NativeDetour unconnectOffMeshConnectionDetour;

    private static UnconnectOffMeshConnectionDelegate unconnectOffMeshConnectionOriginal;

    private static void SetUpUnconnectOffMeshConnectionDetour()
    {
        var functionOffset = 0xA799B0;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x1305070;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        var hookPtr = Marshal.GetFunctionPointerForDelegate<UnconnectOffMeshConnectionDelegate>(UnconnectOffMeshConnectionDetour);

        unconnectOffMeshConnectionDetour = new NativeDetour(functionAddress, hookPtr);
        unconnectOffMeshConnectionOriginal = unconnectOffMeshConnectionDetour.GenerateTrampoline<UnconnectOffMeshConnectionDelegate>();
    }

    private static void UnconnectOffMeshConnectionDetour(IntPtr navMesh, uint index)
    {
        NavMeshLock.BeginWrite();
        unconnectOffMeshConnectionOriginal(navMesh, index);
        NavMeshLock.EndWrite();
    }

    internal static void UnconnectOffMeshConnection(IntPtr navMesh, uint index)
    {
        unconnectOffMeshConnectionOriginal(navMesh, index);
    }
}
