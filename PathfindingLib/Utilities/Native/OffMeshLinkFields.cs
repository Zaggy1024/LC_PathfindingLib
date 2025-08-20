using System.Runtime.InteropServices;

using UnityEngine;

namespace PathfindingLib.Utilities.Native;

// See OffMeshLink::UpdateMovedPositions().
[StructLayout(LayoutKind.Explicit)]
internal unsafe ref struct OffMeshLinkFields
{
    [FieldOffset(0x00)] public ulong ConnectionID;
    [FieldOffset(0x08)] public PPtr<NativeTransform> Start;
    [FieldOffset(0x0C)] public PPtr<NativeTransform> End;
    [FieldOffset(0x10)] public Vector3 LastEndPosition;
    [FieldOffset(0x1C)] public Vector3 LastStartPosition;
    [FieldOffset(0x28)] public float AutoUpdateDistance;
    [FieldOffset(0x2C)] public float CostOverride;
    [FieldOffset(0x30)] public uint Area;
    [FieldOffset(0x34)] public int AgentTypeID;
    [FieldOffset(0x38)] public int ManagerLinkIndex;
    [FieldOffset(0x3C)] public bool AutoUpdatePositions;
    [FieldOffset(0x3D)] public bool NeedsInitialUpdate;
    [FieldOffset(0x3E)] public bool Bidirectional;
    [FieldOffset(0x3F)] public bool Activated;
}
