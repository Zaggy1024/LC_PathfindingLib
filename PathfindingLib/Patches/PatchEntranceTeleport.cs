using System.Collections.Generic;

using HarmonyLib;
using Unity.Netcode;

using PathfindingLib.Utilities;

namespace PathfindingLib.Patches;

internal static class PatchEntranceTeleport
{
    internal static List<EntranceTeleport> allEntranceTeleports = [];

    [HarmonyPostfix]
    [HarmonyPatch(typeof(EntranceTeleport), nameof(EntranceTeleport.Awake))]
    private static void AwakePostfix(EntranceTeleport __instance)
    {
        allEntranceTeleports.Add(__instance);
    }

    [HarmonyPostfix]
    [HarmonyPatch(typeof(NetworkBehaviour), nameof(NetworkBehaviour.OnDestroy))]
    private static void DestroyPostfix(NetworkBehaviour __instance)
    {
        if (__instance is not EntranceTeleport teleport)
            return;

        allEntranceTeleports.Remove(teleport);
        SmartPathLinks.UnregisterEntranceTeleport(teleport);
    }

    [HarmonyPostfix]
    [HarmonyPatch(typeof(RoundManager), nameof(RoundManager.SetExitIDs))]
    private static void SetExitIDsPostfix()
    {
        //PathfindingLibPlugin.Instance.Logger.LogInfo($"SetExitIDsPostfix");
        foreach (var teleport in allEntranceTeleports)
        {
            if (!teleport.isActiveAndEnabled)
                continue;

            foreach (var otherTeleport in allEntranceTeleports)
            {
                if (!otherTeleport.isActiveAndEnabled)
                    continue;
                if (teleport.entranceId != otherTeleport.entranceId)
                    continue;
                if (teleport.isEntranceToBuilding == otherTeleport.isEntranceToBuilding)
                    continue;

                //PathfindingLibPlugin.Instance.Logger.LogInfo($"{teleport} ({teleport.GetInstanceID()}) connects to {teleport} ({teleport.GetInstanceID()})");
                SmartPathLinks.RegisterEntranceTeleport(teleport, otherTeleport.entrancePoint);
                break;
            }
        }
    }
}
