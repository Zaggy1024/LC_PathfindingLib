using System;
using System.Collections.Generic;

using UnityEngine;

namespace PathfindingLib.Utilities.Native;

public static class NativeNavMeshUtils
{
    internal static unsafe bool CarvingHasDataToApply(IntPtr navMeshCarving)
    {
        if (navMeshCarving == IntPtr.Zero)
            return false;

        // Within NavMeshCarving::ApplyCarveResults(), these offsets are derived from a statement
        // that decompiles similarly to the following in a debug build:
        //   auto var = (*(longlong*)(this + 0x40) - *(longlong*)(this + 0x38)) / 0x50;
        // Note that the division by 0x50 may be decompiled instead as a bit shift right followed by an
        // integer multiplication. This is a compiler optimization that doesn't get fully reversed, likely
        // due to the compiler assuming that those pointers are aligned to a multiple of the divisor.
        //
        // In a more human-readable form, this would be equivalent to:
        //   auto jobCount = this->m_JobDataVectorEnd - this->m_JobDataVector;
        // The function call when resizing m_JobDataVector appears to indicate it to be the beginning of
        // an std::vector<> with an element type of CarveData. Therefore, the statement would really be:
        //   auto jobCount = this->m_JobData.size();
        //
        // In order to determine whether there are any jobs in flight, we first extract the pointer to
        // the start and end of the valid data, then check if the end is greater than the start.
        var jobData = 0UL;
        var jobDataEnd = 0UL;
        if (NativeFunctions.IsDebugBuild)
        {
            jobData = *(ulong*)(navMeshCarving + 0x38);
            jobDataEnd = *(ulong*)(navMeshCarving + 0x40);
        }
        else
        {
            jobData = *(ulong*)((ulong)navMeshCarving + 0x28);
            jobDataEnd = *(ulong*)((ulong)navMeshCarving + 0x30);
        }

        return jobDataEnd > jobData;
    }

    // See NavMeshManager::Update().
    private static unsafe void GetOffMeshLinkList(IntPtr navMeshManager, out IntPtr offMeshLinks, out int offMeshLinkCount)
    {
        if (NativeFunctions.IsDebugBuild)
        {
            offMeshLinks = *(IntPtr*)(navMeshManager + 0x70);
            offMeshLinkCount = *(int*)(navMeshManager + 0x88);
            return;
        }

        offMeshLinks = *(IntPtr*)(navMeshManager + 0x60);
        offMeshLinkCount = *(int*)(navMeshManager + 0x70);
    }

    private static unsafe IntPtr GetOffMeshLinkFromList(IntPtr links, int index)
    {
        return *(((IntPtr*)links) + index);
    }

    internal static unsafe IEnumerable<IntPtr> GetOffMeshLinks(IntPtr navMeshManager)
    {
        GetOffMeshLinkList(navMeshManager, out var links, out var linkCount);

        if (links == IntPtr.Zero)
            yield break;

        for (var i = 0; i < linkCount; i++)
            yield return GetOffMeshLinkFromList(links, i);
    }

    // See OffMeshLink::UpdateMovedPositions()
    internal static unsafe void GetOffMeshLinkFields(IntPtr offMeshLink, out bool autoUpdate, out bool skipUpdate, out float updateDistance, out IntPtr startTransformPtr, out IntPtr endTransformPtr, out Vector3 lastStartTransformPos, out Vector3 lastEndTransformPos)
    {
        var fieldsBaseAddress = offMeshLink + (NativeFunctions.IsDebugBuild ? 0x60 : 0x48);

        startTransformPtr = NativeFunctions.DerefPPtr(fieldsBaseAddress + 0x0);
        endTransformPtr = NativeFunctions.DerefPPtr(fieldsBaseAddress + 0x4);
        lastEndTransformPos = *(Vector3*)(fieldsBaseAddress + 0x8);
        lastStartTransformPos = *(Vector3*)(fieldsBaseAddress + 0x14);
        updateDistance = *(float*)(fieldsBaseAddress + 0x20);
        autoUpdate = *(bool*)(fieldsBaseAddress + 0x34);
        skipUpdate = *(bool*)(fieldsBaseAddress + 0x35);
    }

    internal static unsafe void GetOffMeshLinkData(IntPtr offMeshLink, out bool autoUpdate, out bool skipUpdate, out float updateDistance, out Vector3 startPos, out Vector3 endPos, out Vector3 lastStartPos, out Vector3 lastEndPos)
    {
        GetOffMeshLinkFields(offMeshLink, out autoUpdate, out skipUpdate, out updateDistance, out var startTransformPtr, out var endTransformPtr, out lastStartPos, out lastEndPos);
        startPos = NativeFunctions.GetPosition(startTransformPtr);
        endPos = NativeFunctions.GetPosition(endTransformPtr);
    }

    internal static unsafe bool OffMeshLinkWillUpdate(IntPtr offMeshLink)
    {
        GetOffMeshLinkData(offMeshLink, out var autoUpdate, out var skipUpdate, out var updateDistance, out var startPos, out var endPos, out var lastStartPos, out var lastEndPos);

        if (skipUpdate)
            return false;
        if (!autoUpdate)
            return false;

        var updateDistanceSqr = updateDistance * updateDistance;
        if ((startPos - lastStartPos).sqrMagnitude > updateDistanceSqr)
            return true;
        if ((endPos - lastEndPos).sqrMagnitude > updateDistanceSqr)
            return true;

        return false;
    }
}
