using System;
using System.Runtime.InteropServices;

using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Experimental.AI;

namespace PathfindingLib.Utilities;

[StructLayout(LayoutKind.Sequential, Pack = 4)]
public struct NavMeshPathNative
{
    public int offMeshLinkModCount;
    public int status;
    public IntPtr polygons;
    public unsafe fixed int padding[4];
    public ulong size;
    public ulong capacity;
    public Vector3 sourcePosition;
    public Vector3 targetPosition;
    public Span<PolygonId> currentPolygons
    {
        get
        {
            unsafe
            {
                return new Span<PolygonId>((PolygonId*)polygons.ToPointer(), (int)size);
            }
        }
    }
    public readonly override string ToString()
    {
        return $"modcount {offMeshLinkModCount} status {status} polys {(ulong)polygons:X16} source {sourcePosition} target {targetPosition} size {size} capacity {capacity}";
    }

    public static NavMeshPathNative GetNativeData(NavMeshPath path)
    {
        unsafe
        {
            return *(NavMeshPathNative*)path.m_Ptr.ToPointer();
        }
    }
}
