using System;

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
}
