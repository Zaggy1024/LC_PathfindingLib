using System;
using System.Runtime.InteropServices;

using MonoMod.RuntimeDetour;
using UnityEngine;
using UnityEngine.AI;
using Unity.Mathematics.Geometry;

using PathfindingLib.Utilities.Native;
using PathfindingLib.API;

namespace PathfindingLib.Patches.Native;

internal static class PatchOffMeshLinkUpdatePositions
{
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private delegate void UpdatePositionsDelegate(IntPtr offMeshLink);

    private static NativeDetour detour;

    private static UpdatePositionsDelegate original;

    internal static void Apply()
    {
        var functionOffset = 0xA3E2E0;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x1297D30;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        var hookPtr = Marshal.GetFunctionPointerForDelegate<UpdatePositionsDelegate>(UpdatePositionsDetour);

        detour = new NativeDetour(functionAddress, hookPtr);
        original = detour.GenerateTrampoline<UpdatePositionsDelegate>();
    }

    private static MinMaxAABB AABBFromPoints(Vector3 a, Vector3 b)
    {
        return new MinMaxAABB(Vector3.Min(a, b), Vector3.Max(a, b));
    }

    internal static void UpdateConnection(ref OffMeshConnection connection, Vector3 start, Vector3 end, Vector3 up, float width, float costModifier, bool bidirectional, byte area, bool activated, int instanceID, int agentTypeID)
    {
        connection.EndPointA.Pos = start;
        connection.EndPointB.Pos = end;

        var delta = end - start;
        var length = delta.magnitude;
        var forward = Vector3.forward;
        if (length >= 0.000001)
            forward = delta / length;

        connection.AxisY = up;
        connection.AxisX = Vector3.Cross(connection.AxisY, forward);
        connection.AxisZ = Vector3.Cross(connection.AxisX, connection.AxisY);
        connection.Width = width;

        if (costModifier < 0)
            costModifier = -1;
        connection.CostModifier = costModifier;

        connection.Bidirectional = bidirectional;
        connection.Area = area;
        connection.AreaMask = activated ? 1 << (area & 0x1F) : 0;

        connection.LinkType = 0;
        connection.UserID = instanceID;
        connection.AgentTypeID = agentTypeID;

        if (width > 0)
        {
            var axisXMagnitude = connection.AxisX.magnitude;
            var extents = axisXMagnitude > 0.00001 ? connection.AxisX.normalized * (width * 0.5f) : Vector3.zero;
            connection.Bounds = AABBFromPoints(connection.EndPointA.Pos - extents, connection.EndPointA.Pos + extents);
            connection.Bounds.Encapsulate(AABBFromPoints(connection.EndPointB.Pos - extents, connection.EndPointB.Pos + extents));
        }
        else
        {
            connection.Bounds = new MinMaxAABB(Vector3.Min(start, end), Vector3.Max(start, end));
        }

        // Unconnect doesn't seem to always remove the tile refs to allow the connection to update.
        connection.EndPointA.TileRef = 0;
        connection.EndPointB.TileRef = 0;
    }

    private static unsafe bool TryUpdatingConnectionInPlace(IntPtr offMeshLink)
    {
        // OffMeshLink::UpdatePositions() calls NavMesh::RemoveOffMeshConnection() and subsequently
        // NavMesh::AddOffMeshConnection(). This removes the link from several different pieces of
        // memory, and bumps the modification count of the navmesh. This seems to cause paths to be
        // invalidated unnecessarily. Instead, if a connection already exists for this link, reuse
        // and update it.

        if (NativeHelpers.GetNavMesh() == IntPtr.Zero)
            return false;

        ref var fields = ref NativeNavMeshUtils.GetOffMeshLinkFields(offMeshLink);
        var wrapper = NativeFunctions.ScriptingWrapperFor<OffMeshLink>(offMeshLink);
        var instanceID = NativeHelpers.GetInstanceID(offMeshLink);

        if (wrapper == null)
            return false;

        if (!wrapper.isActiveAndEnabled)
            return false;

        if (wrapper.area == 1)
            return false;

        var startTransform = fields.Start.Get();
        var endTransform = fields.End.Get();

        if (startTransform == null)
            return false;
        if (endTransform == null)
            return false;

        if (fields.ConnectionID == 0)
            return false;

        var connectionIndex = (uint)(fields.ConnectionID & 0xffff);
        var connectionSalt = (uint)(fields.ConnectionID >> 48);

        ref var connectionsList = ref NativeHelpers.GetOffMeshConnectionFreeList();
        if (connectionIndex >= connectionsList.Capacity)
            return false;
        ref var connection = ref connectionsList.Elements[connectionIndex];
        if (connection.Salt != connectionSalt)
            return false;

        var startPos = NativeNavMeshUtils.GetOffMeshLinkEndPointPosition(startTransform);
        var endPos = NativeNavMeshUtils.GetOffMeshLinkEndPointPosition(endTransform);

        NavMeshLock.BeginWrite();

        var navMesh = NativeHelpers.GetNavMesh();
        PatchConnectUnconnectOffMeshConnection.UnconnectOffMeshConnection(navMesh, connectionIndex);

        UpdateConnection(ref connection, startPos, endPos, Vector3.up, width: 0, fields.CostOverride, fields.Bidirectional, (byte)fields.Area, fields.Activated, instanceID, fields.AgentTypeID);

        var extents = NativeFunctions.GetLinkQueryExtents(fields.AgentTypeID);
        PatchConnectUnconnectOffMeshConnection.ConnectOffMeshConnection(navMesh, connectionIndex, extents.x, extents.y);

        NavMeshLock.EndWrite();

        fields.LastStartPosition = startPos;
        fields.LastEndPosition = endPos;
        fields.AutoUpdateDistance = MathF.Min(extents.x, extents.y);

        return true;
    }

    private static void UpdatePositionsDetour(IntPtr offMeshLink)
    {
        if (!TryUpdatingConnectionInPlace(offMeshLink))
            original(offMeshLink);
    }
}
