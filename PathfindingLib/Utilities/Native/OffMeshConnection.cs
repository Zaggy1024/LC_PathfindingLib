using System.Runtime.InteropServices;

using Unity.Mathematics.Geometry;
using UnityEngine;

namespace PathfindingLib.Utilities.Native;

[StructLayout(LayoutKind.Explicit, Size = 0xD0)]
internal struct OffMeshConnection
{
    [FieldOffset(0x00)] public int AgentTypeID;
    [FieldOffset(0x04)] public MinMaxAABB Bounds;
    [FieldOffset(0x20)] public OffMeshLinkEndPoint EndPointA;
    [FieldOffset(0x50)] public OffMeshLinkEndPoint EndPointB;
    [FieldOffset(0x80)] public Vector3 AxisX;
    [FieldOffset(0x8C)] public Vector3 AxisY;
    [FieldOffset(0x98)] public Vector3 AxisZ;
    [FieldOffset(0xA4)] public float Width;
    [FieldOffset(0xA8)] public float CostModifier;
    [FieldOffset(0xAC)] public bool Bidirectional;
    [FieldOffset(0xB0)] public int AreaMask;
    [FieldOffset(0xB4)] public byte Area;
    [FieldOffset(0xB6)] public ushort LinkType;
    [FieldOffset(0xB8)] public int UserID;
    [FieldOffset(0xBC)] public int FirstLinkIndex;
    [FieldOffset(0xC0)] public uint Salt;
    [FieldOffset(0xC4)] public int Next;
}

[StructLayout(LayoutKind.Explicit, Size = 0x30)]
internal struct OffMeshLinkEndPoint
{
    [FieldOffset(0x00)] public Vector3 Pos;
    [FieldOffset(0x0C)] public Vector3 MappedA;
    [FieldOffset(0x18)] public Vector3 MappedB;
    [FieldOffset(0x28)] public ulong TileRef;
}
