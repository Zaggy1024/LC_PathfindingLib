using System;
using System.Runtime.InteropServices;

using MonoMod.RuntimeDetour;

using PathfindingLib.API.SmartPathfinding;
using PathfindingLib.Utilities.Native;

namespace PathfindingLib.Patches.Native;

internal static class PatchNavMeshAgent
{
    internal static void Apply(IntPtr baseAddress)
    {
        HookAwakeFromLoad(baseAddress);
    }

    // Detour for NavMeshAgent::AwakeFromLoad()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private delegate void AwakeFromLoadDelegate(IntPtr thisNavMeshAgent, int awakeFromLoadMode);

    private static NativeDetour awakeFromLoadDetour;
    private static AwakeFromLoadDelegate awakeFromLoadOriginal;

    private static void HookAwakeFromLoad(IntPtr baseAddress)
    {
        var awakeFromLoadOffset = 0xA3A810UL;
        if (NativeFunctions.IsDebugBuild)
            awakeFromLoadOffset = 0x1293A20UL;
        var awakeFromLoadAddress = (IntPtr)((ulong)baseAddress + awakeFromLoadOffset);

        var hookPtr = Marshal.GetFunctionPointerForDelegate<AwakeFromLoadDelegate>(AwakeFromLoadDetour);

        awakeFromLoadDetour = new NativeDetour(awakeFromLoadAddress, hookPtr);
        awakeFromLoadOriginal = awakeFromLoadDetour.GenerateTrampoline<AwakeFromLoadDelegate>();
    }

    private unsafe static void AwakeFromLoadDetour(IntPtr thisNavMeshAgent, int awakeFromLoadMode)
    {
        if (awakeFromLoadMode == 3)
        {
            var maskAddress = NativeFunctions.IsDebugBuild ? 0x98L : 0x80L;
            ref var mask = ref *(int*)((long)thisNavMeshAgent + maskAddress);

            var oldMask = mask;
            var agentName = NativeFunctions.GetName(thisNavMeshAgent);
            if (oldMask != -1 && (oldMask & SmartPathfinding.NonSmartAgentOffMeshLinkAreaMask) != 0)
                PathfindingLibPlugin.Instance.Logger.LogWarning($"Prefabbed NavMeshAgent {agentName} unexpectedly had the non-smart navmesh area enabled. This may indicate that a mod is already using this navmesh area.");

            mask |= SmartPathfinding.NonSmartAgentOffMeshLinkAreaMask;

            PathfindingLibPlugin.Instance.Logger.LogDebug($"Changed prefabbed NavMeshAgent {agentName}'s area mask from 0x{oldMask:X8} to 0x{mask:X8} to allow traversal of non-smart off-mesh links.");
        }

        awakeFromLoadOriginal(thisNavMeshAgent, awakeFromLoadMode);
    }
}
