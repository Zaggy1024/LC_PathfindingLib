using System;
using System.Runtime.InteropServices;

using Unity.Collections;
using Unity.Mathematics;
using UnityEngine.Experimental.AI;

using PathfindingLib.Patches.Native;
using Unity.Collections.LowLevel.Unsafe;

namespace PathfindingLib.Utilities;

public static class NavMeshQueryUtils
{
    /// <summary>
    /// The recommended number of path corners to allocate when calculating a straight path using
    /// <see cref="FindStraightPath"/>.
    /// </summary>
    public const int RecommendedCornerCount = 128;

    public static unsafe PathQueryStatus FindStraightPath(this NavMeshQuery query,
        in float3 startPos, in float3 endPos,
        in NativeSlice<PolygonId> path, int pathSize,
        in NativeArray<float3> straightPath,
        in NativeArray<StraightPathFlags> straightPathFlags,
        in NativeArray<PolygonId> straightPathRefs,
        out int straightPathCount)
    {
        if (path.Stride != UnsafeUtility.SizeOf<PolygonId>())
            throw new ArgumentException("Path slice must have a stride equal to the size of PolygonId");
        straightPathCount = 0;
        return findStraightPathMethod(query, in startPos, in endPos, path.GetPtr(), pathSize, straightPath.GetPtr(), straightPathFlags.GetPtr(), straightPathRefs.GetPtr(), ref straightPathCount, straightPath.Length);
    }

    public static PathQueryStatus FindStraightPath(this NavMeshQuery query,
        in float3 startPos, in float3 endPos,
        in NativeSlice<PolygonId> path, int pathSize,
        in NativeArray<float3> straightPath,
        out int straightPathCount)
    {
        using var straightPathFlags = new NativeArray<StraightPathFlags>(straightPath.Length, Allocator.Temp);
        using var straightPathRefs = new NativeArray<PolygonId>(straightPath.Length, Allocator.Temp);
        return FindStraightPath(query, in startPos, in endPos, path, pathSize, straightPath, straightPathFlags, straightPathRefs, out straightPathCount);
    }

    public static PathQueryStatus FindStraightPath(this NavMeshQuery query,
        float3 startPos, float3 endPos,
        NativeSlice<PolygonId> path, int pathSize,
        NativeArray<NavMeshLocation> straightPath,
        out int straightPathCount)
    {
        using var straightPathPositions = new NativeArray<float3>(straightPath.Length, Allocator.Temp);
        using var straightPathFlags = new NativeArray<StraightPathFlags>(straightPath.Length, Allocator.Temp);
        using var straightPathRefs = new NativeArray<PolygonId>(straightPath.Length, Allocator.Temp);
        var result = FindStraightPath(query, in startPos, in endPos, path, pathSize, straightPathPositions, straightPathFlags, straightPathRefs, out straightPathCount);
        for (var i = 0; i < straightPathCount; i++)
            straightPath[i] = new(straightPathPositions[i], straightPathRefs[i]);
        return result;
    }

    [Flags]
    public enum StraightPathFlags
    {
        Start = 0x01,
        End = 0x02,
        OffMeshConnection = 0x04,
    }

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private unsafe delegate PathQueryStatus FindStraightPathDelegate(NavMeshQuery self,
        in float3 startPos, in float3 endPos,
        PolygonId* path, int pathSize,
        float3* straightPath,
        StraightPathFlags* straightPathFlags,
        PolygonId* straightPathRefs,
        ref int straightPathCount,
        int maxStraightPath);

    private static FindStraightPathDelegate findStraightPathMethod;

    internal static void SetUpNativeMethodPointers(IntPtr baseAddress)
    {
        var functionOffset = 0xA59270UL;
        if (NativeHooksCommon.IsDebugBuild)
            functionOffset = 0x12B0040UL;
        var functionAddress = (IntPtr)((ulong)baseAddress + functionOffset);

        findStraightPathMethod = Marshal.GetDelegateForFunctionPointer<FindStraightPathDelegate>(functionAddress);
    }

    private static unsafe T* GetPtr<T>(this in NativeArray<T> array) where T : unmanaged
    {
        return (T*)array.m_Buffer;
    }

    private static unsafe T* GetPtr<T>(this in NativeSlice<T> slice) where T : unmanaged
    {
        return (T*)slice.m_Buffer;
    }
}
