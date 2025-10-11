using System;

using HarmonyLib;
using Unity.AI.Navigation;
using UnityEngine;

using PathfindingLib.Utilities.Native;
using PathfindingLib.Patches.Native;
using PathfindingLib.API;

namespace PathfindingLib.Patches;

[HarmonyPatch(typeof(NavMeshLink))]
internal static class PatchNavMeshLink
{
    private static unsafe bool TryUpdatingConnectionInPlace(NavMeshLink link)
    {
        var navMesh = NativeHelpers.GetNavMesh();
        if (navMesh == IntPtr.Zero)
            return false;

        var handle = link.m_LinkInstance.id;
        var linkIndex = handle & 0xFFFF;

        ref var links = ref NativeNavMeshUtils.GetNavMeshLinkRegistry();
        if (linkIndex >= links.Capacity)
            return false;

        ref var linkData = ref links.Elements[linkIndex];
        if (linkData.ConnectionID == 0)
            return false;

        var handleUseCount = (handle >> 16) & 0xffff;
        if (linkData.UseCount != handleUseCount)
            return false;

        var position = link.transform.position;
        var rotation = link.transform.rotation;

        var matrix = Matrix4x4.TRS(position, rotation, Vector3.one);
        var start = matrix.MultiplyPoint3x4(link.startPoint);
        var end = matrix.MultiplyPoint3x4(link.endPoint);

        var up = rotation * Vector3.up;

        var connectionIndex = (uint)linkData.ConnectionID & 0xFFFF;
        var connectionSalt = (uint)(linkData.ConnectionID >> 48);

        ref var connectionsList = ref NativeHelpers.GetOffMeshConnectionFreeList();
        if (connectionIndex >= connectionsList.Capacity)
            return false;
        ref var connection = ref connectionsList.Elements[connectionIndex];
        if (connection.Salt != connectionSalt)
            return false;

        NavMeshLock.BeginWrite();

        PatchConnectUnconnectOffMeshConnection.UnconnectOffMeshConnection(navMesh, connectionIndex);

        PatchOffMeshLinkUpdatePositions.UpdateConnection(ref connection, start, end, up, link.width, link.costModifier, link.bidirectional, (byte)link.area, link.isActiveAndEnabled, link.GetInstanceID(), link.agentTypeID);

        var extents = NativeFunctions.GetLinkQueryExtents(link.agentTypeID);
        PatchConnectUnconnectOffMeshConnection.ConnectOffMeshConnection(navMesh, connectionIndex, extents.x, extents.y);

        NavMeshLock.EndWrite();

        link.m_LastPosition = position;
        link.m_LastRotation = rotation;

        return true;
    }

    [HarmonyPrefix]
    [HarmonyPatch(nameof(NavMeshLink.UpdateLink))]
    private static bool UpdateLinkPrefix(NavMeshLink __instance)
    {
        return !TryUpdatingConnectionInPlace(__instance);
    }
}