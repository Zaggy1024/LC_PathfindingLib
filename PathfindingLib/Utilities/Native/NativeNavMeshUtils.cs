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
        if (NativeHelpers.IsDebugBuild)
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
        if (NativeHelpers.IsDebugBuild)
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

    internal static unsafe ref OffMeshLinkFields GetOffMeshLinkFields(IntPtr offMeshLink)
    {
        var fieldOffset = 0x40;
        if (NativeHelpers.IsDebugBuild)
            fieldOffset = 0x58;
        return ref *(OffMeshLinkFields*)(offMeshLink + fieldOffset);
    }

    internal static unsafe Vector3 GetOffMeshLinkEndPointPosition(NativeTransform* transform)
    {
        if (transform == null)
            return Vector3.positiveInfinity;
        return NativeFunctions.GetPosition((IntPtr)transform);
    }

    internal static unsafe bool OffMeshLinkWillUpdate(IntPtr offMeshLink)
    {
        ref var fields = ref GetOffMeshLinkFields(offMeshLink);
        var startTransform = fields.Start.Get();
        var endTransform = fields.End.Get();
        var startPos = GetOffMeshLinkEndPointPosition(startTransform);
        var endPos = GetOffMeshLinkEndPointPosition(endTransform);

        if (!fields.AutoUpdatePositions)
            return false;
        if (fields.NeedsInitialUpdate)
            return false;

        var updateDistanceSqr = fields.AutoUpdateDistance * fields.AutoUpdateDistance;
        if ((startPos - fields.LastStartPosition).sqrMagnitude > updateDistanceSqr)
            return true;
        if ((endPos - fields.LastEndPosition).sqrMagnitude > updateDistanceSqr)
            return true;

        return false;
    }

    // See NavMeshManager::AddLink()
    internal static unsafe ref FreeList<NavMeshLinkRegistryEntry> GetNavMeshLinkRegistry()
    {
        var offset = 0x80;
        if (NativeHelpers.IsDebugBuild)
            offset = 0x98;
        return ref *(FreeList<NavMeshLinkRegistryEntry>*)(NativeHelpers.GetNavMeshManager() + offset);
    }
}
