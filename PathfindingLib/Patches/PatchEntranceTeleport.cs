using HarmonyLib;
using Unity.Netcode;

using PathfindingLib.Utilities;
using System.Collections.Generic;

namespace PathfindingLib.Patches;

internal static class PatchEntranceTeleport
{
    internal static HashSet<EntranceTeleport> unconnectedEntranceTeleports = [];
    internal static List<EntranceTeleport> allEntranceTeleports = [];

    [HarmonyPostfix]
    [HarmonyPatch(typeof(EntranceTeleport), nameof(EntranceTeleport.Awake))]
    private static void AwakePostfix(EntranceTeleport __instance)
    {
        allEntranceTeleports.Add(__instance);
        unconnectedEntranceTeleports.Add(__instance);
    }

    [HarmonyPostfix]
    [HarmonyPatch(typeof(EntranceTeleport), nameof(EntranceTeleport.Update))]
    private static void UpdatePostfix(EntranceTeleport __instance)
    {
        if (!unconnectedEntranceTeleports.Contains(__instance))
            return;

        foreach (var otherTeleport in allEntranceTeleports)
        {
            if (otherTeleport.entranceId == __instance.entranceId && otherTeleport.isEntranceToBuilding != __instance.isEntranceToBuilding)
            {
                SmartPathLinks.RegisterEntranceTeleport(__instance, otherTeleport.entrancePoint);
                unconnectedEntranceTeleports.Remove(__instance);
                break;
            }
        }
    }

    [HarmonyPostfix]
    [HarmonyPatch(typeof(NetworkBehaviour), nameof(NetworkBehaviour.OnDestroy))]
    private static void DestroyPostfix(NetworkBehaviour __instance)
    {
        if (__instance is not EntranceTeleport teleport)
            return;

        allEntranceTeleports.Remove(teleport);
        unconnectedEntranceTeleports.Remove(teleport);
        SmartPathLinks.UnregisterEntranceTeleport(teleport);
    }
}
