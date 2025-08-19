using BepInEx;
using BepInEx.Logging;
using HarmonyLib;
using UnityEngine;

using PathfindingLib.Components;
using PathfindingLib.Patches;
using PathfindingLib.Patches.Native;
using PathfindingLib.Utilities;
using PathfindingLib.Utilities.Native;

namespace PathfindingLib;

[BepInPlugin(PluginGUID, PluginName, PluginVersion)]
public class PathfindingLibPlugin : BaseUnityPlugin
{
    public const string PluginName = "PathfindingLib";
    public const string PluginGUID = "Zaggy1024." + PluginName;
    public const string PluginVersion = "2.1.3";

    private readonly Harmony harmony = new(PluginGUID);

    internal static PathfindingLibPlugin Instance { get; private set; }
    internal new ManualLogSource Logger => base.Logger;

    public void Awake()
    {
        Instance = this;

        ApplyAllNativePatches();

        harmony.PatchAll(typeof(PatchNavMeshSurface));
        harmony.PatchAll(typeof(PatchEntranceTeleport));
        harmony.PatchAll(typeof(PatchMineshaftElevatorController));

        var disposerObject = new GameObject("SmartPathTaskDisposer");
        DontDestroyOnLoad(disposerObject);
        disposerObject.hideFlags = HideFlags.HideAndDontSave;
        disposerObject.AddComponent<SmartPathTaskDisposer>();
    }

    private static void ApplyAllNativePatches()
    {
        NativeFunctions.SetUpNativeMethodPointers();
        NavMeshQueryUtils.SetUpNativeMethodPointers();

        PatchNavMeshManagerUpdate.Apply();
        PatchConnectUnconnectOffMeshConnection.Apply();
        PatchApplyCarveResults.Apply();
        PatchNavMeshAgent.Apply();
    }
}
