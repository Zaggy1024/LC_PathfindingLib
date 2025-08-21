using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace PathfindingLib.Utilities.Native;

[StructLayout(LayoutKind.Sequential)]
internal struct PPtr<T> where T : unmanaged
{
    public int Ptr;
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct FreeList<T> where T : unmanaged
{
    public uint NextFree;
    public uint Capacity;
    public T* Elements;
}

[StructLayout(LayoutKind.Explicit)]
internal struct NativeTransform
{
}

[StructLayout(LayoutKind.Sequential)]
internal struct NavMeshLinkRegistryEntry
{
    internal int UseCount;
    internal uint Next;
    internal ulong ConnectionID;
}

internal static class Methods
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static unsafe IntPtr GetPtr<T>(this ref PPtr<T> ptr) where T : unmanaged
    {
        return NativeFunctions.DerefPPtr((IntPtr)UnsafeUtility.AddressOf(ref ptr));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static unsafe T* Get<T>(this ref PPtr<T> ptr) where T : unmanaged
    {
        return (T*)GetPtr(ref ptr);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static unsafe Vector3 GetPosition(this ref NativeTransform transform)
    {
        return NativeFunctions.GetPosition((IntPtr)UnsafeUtility.AddressOf(ref transform));
    }
}