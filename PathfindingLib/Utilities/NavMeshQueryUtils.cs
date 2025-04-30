using System;
using System.Runtime.InteropServices;

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.AI;

namespace PathfindingLib.Utilities;

[Flags]
public enum StraightPathFlags : byte
{
    Start = 0x01,
    End = 0x02,
    OffMeshConnection = 0x04,
}

public static class NavMeshQueryUtils
{
    /// <summary>
    /// The recommended number of path corners to allocate when calculating a straight path using
    /// <see cref="FindStraightPath"/>.
    /// </summary>
    public const int RecommendedCornerCount = 128;

    /// <summary>
    /// The number of path corners required to be allocated when calculating a straight path using
    /// <see cref="FindStraightPath"/>.
    /// </summary>
    /// <param name="pathSize">The number of polygons in the path.</param>
    /// <returns>The number of elements to allocate and pass to <see cref="FindStraightPath"/>.</returns>
    public static int RequiredCornerCount(int pathPolygonCount)
    {
        return pathPolygonCount + 2;
    }

    /// <summary>
    /// Calculate a straight path from the data produced by finding a path with NavMeshQuery.
    /// 
    /// <para>If this is called off the main thread, it should be called with the navmesh locked
    /// for reading via <see cref="NavMeshLock"/>.</para>
    /// </summary>
    /// <param name="query">The query to request navmesh geometry from.</param>
    /// <param name="startPos">The position which should be the start of the path.</param>
    /// <param name="endPos">The position which should be the end of the path.</param>
    /// <param name="path">The polygons provided by <see cref="NavMeshQuery.GetPathResult"/>.</param>
    /// <param name="pathSize">The number of polygons that make up the path, returned by <see cref="NavMeshQuery.GetPathResult"/>.</param>
    /// <param name="straightPath">An array that will be filled with the resulting straight path corners' positions.</param>
    /// <param name="straightPathFlags">An array that will be filled with flags indicating the type of each corner.</param>
    /// <param name="straightPathRefs">An array that will be filled with the polygons that each of the corners resides on.</param>
    /// <param name="straightPathCount">The number of corners in the resulting straight path.</param>
    /// <returns>The completion status of the calculation, with any relevant detail flags.</returns>
    public static unsafe PathQueryStatus FindStraightPath(this NavMeshQuery query,
        in Vector3 startPos, in Vector3 endPos,
        in NativeSlice<PolygonId> path, int pathSize,
        in NativeArray<Vector3> straightPath,
        in NativeArray<StraightPathFlags> straightPathFlags,
        in NativeArray<PolygonId> straightPathRefs,
        out int straightPathCount)
    {
        if (path.Stride != UnsafeUtility.SizeOf<PolygonId>())
            throw new ArgumentException("Path slice must have a stride equal to the size of PolygonId.");
        if (straightPathFlags.Length < straightPath.Length)
            throw new ArgumentException("Straight path flags buffer is too small.");
        if (straightPathRefs.Length < straightPath.Length)
            throw new ArgumentException("Straight path refs buffer is too small.");

        straightPathCount = 0;

        // NavMeshPath::m_TargetPosition gets moved to the closest point on the last polygon to the destination.
        // That is then passed to FindStraightPath when NavMeshPath::CalculateCorners() is called.
        // We must mirror that, or partial paths will always reach their destination.
        var endPosStatus = NavMeshQuery.GetClosestPointOnPoly(query.m_NavMeshQuery, path[pathSize - 1], endPos, out var endPosOnPath);
        if (endPosStatus < 0)
            return endPosStatus;

        return findStraightPathMethod(query, in startPos, in endPosOnPath, path.GetPtr(), pathSize, straightPath.GetPtr(), straightPathFlags.GetPtr(), straightPathRefs.GetPtr(), ref straightPathCount, straightPath.Length);
    }

    /// <summary>
    /// Calculate a straight path from the data produced by finding a path with NavMeshQuery.
    /// 
    /// <para>If this is called off the main thread, it should be called with the navmesh locked
    /// for reading via <see cref="NavMeshLock"/>.</para>
    /// </summary>
    /// <param name="query">The query to request navmesh geometry from.</param>
    /// <param name="startPos">The position which should be the start of the path.</param>
    /// <param name="endPos">The position which should be the end of the path.</param>
    /// <param name="path">The polygons provided by <see cref="NavMeshQuery.GetPathResult"/>.</param>
    /// <param name="pathSize">The number of polygons that make up the path, returned by <see cref="NavMeshQuery.GetPathResult"/>.</param>
    /// <param name="straightPath">An array that will be filled with the resulting straight path corners' positions.</param>
    /// <param name="straightPathCount">The number of corners in the resulting straight path.</param>
    /// <returns>The completion status of the calculation, with any relevant detail flags.</returns>
    public static PathQueryStatus FindStraightPath(this NavMeshQuery query,
        in Vector3 startPos, in Vector3 endPos,
        in NativeSlice<PolygonId> path, int pathSize,
        in NativeArray<Vector3> straightPath,
        out int straightPathCount)
    {
        using var straightPathFlags = new NativeArray<StraightPathFlags>(straightPath.Length, Allocator.Temp);
        using var straightPathRefs = new NativeArray<PolygonId>(straightPath.Length, Allocator.Temp);
        return FindStraightPath(query, in startPos, in endPos, path, pathSize, straightPath, straightPathFlags, straightPathRefs, out straightPathCount);
    }

    /// <summary>
    /// Calculate a straight path from the data produced by finding a path with NavMeshQuery.
    /// 
    /// <para>If this is called off the main thread, it should be called with the navmesh locked
    /// for reading via <see cref="NavMeshLock"/>.</para>
    /// </summary>
    /// <param name="query">The query to request navmesh geometry from.</param>
    /// <param name="startPos">The position which should be the start of the path.</param>
    /// <param name="endPos">The position which should be the end of the path.</param>
    /// <param name="path">The polygons provided by <see cref="NavMeshQuery.GetPathResult"/>.</param>
    /// <param name="pathSize">The number of polygons that make up the path, returned by <see cref="NavMeshQuery.GetPathResult"/>.</param>
    /// <param name="straightPath">An array that will be filled with the resulting straight path.</param>
    /// <param name="straightPathCount">The number of corners in the resulting straight path corners.</param>
    /// <returns>The completion status of the calculation, with any relevant detail flags.</returns>
    [Obsolete("Use NativeArray<Vector3> for the straightPath parameter")]
    public static PathQueryStatus FindStraightPath(this NavMeshQuery query,
        float3 startPos, float3 endPos,
        NativeSlice<PolygonId> path, int pathSize,
        NativeArray<NavMeshLocation> straightPath,
        out int straightPathCount)
    {
        using var straightPathPositions = new NativeArray<Vector3>(straightPath.Length, Allocator.Temp);
        using var straightPathFlags = new NativeArray<StraightPathFlags>(straightPath.Length, Allocator.Temp);
        using var straightPathRefs = new NativeArray<PolygonId>(straightPath.Length, Allocator.Temp);
        var result = FindStraightPath(query, startPos, endPos, path, pathSize, straightPathPositions, straightPathFlags, straightPathRefs, out straightPathCount);
        for (var i = 0; i < straightPathCount; i++)
            straightPath[i] = new(straightPathPositions[i], straightPathRefs[i]);
        return result;
    }

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private unsafe delegate PathQueryStatus FindStraightPathDelegate(NavMeshQuery self,
        in Vector3 startPos, in Vector3 endPos,
        PolygonId* path, int pathSize,
        Vector3* straightPath,
        StraightPathFlags* straightPathFlags,
        PolygonId* straightPathRefs,
        ref int straightPathCount,
        int maxStraightPath);

    private static FindStraightPathDelegate findStraightPathMethod;

    internal static void SetUpNativeMethodPointers(IntPtr baseAddress)
    {
        var functionOffset = 0xA59270UL;
        if (NativeFunctions.IsDebugBuild)
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
