using HarmonyLib;

using PathfindingLib.Components;

namespace PathfindingLib.Patches;

[HarmonyPatch(typeof(MineshaftElevatorController))]
internal static class PatchMineshaftElevatorController
{
    [HarmonyPostfix]
    [HarmonyPatch(nameof(MineshaftElevatorController.OnEnable))]
    private static void OnEnablePostfix(MineshaftElevatorController __instance)
    {
        if (__instance.TryGetComponent(out MineshaftElevatorAdapter adapter))
            adapter.enabled = true;

        adapter = __instance.gameObject.AddComponent<MineshaftElevatorAdapter>();
        adapter.controller = __instance;
    }

    [HarmonyPostfix]
    [HarmonyPatch(nameof(MineshaftElevatorController.OnDisable))]
    private static void OnDisablePostfix(MineshaftElevatorController __instance)
    {
        if (__instance.TryGetComponent(out MineshaftElevatorAdapter adapter))
            adapter.enabled = false;
    }
}
