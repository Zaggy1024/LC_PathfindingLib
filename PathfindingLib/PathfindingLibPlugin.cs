using System.Diagnostics;

using BepInEx;
using BepInEx.Logging;
using HarmonyLib;
using UnityEngine;

using PathfindingLib.Components;
using PathfindingLib.Patches;
using PathfindingLib.Patches.Native;
using PathfindingLib.Utilities;

namespace PathfindingLib;

[BepInPlugin(PluginGUID, PluginName, PluginVersion)]
public class PathfindingLibPlugin : BaseUnityPlugin
{
    public const string PluginName = "PathfindingLib";
    public const string PluginGUID = "Zaggy1024." + PluginName;
    public const string PluginVersion = "1.0.1";

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

    private static ProcessModule GetUnityPlayerModule()
    {
        var modules = Process.GetCurrentProcess().Modules;
        for (var i = 0; i < modules.Count; i++)
        {
            var module = modules[i];
            if (module.ModuleName.Contains("UnityPlayer"))
                return module;
        }

        return null;
    }

    private static void ApplyAllNativePatches()
    {
        var module = GetUnityPlayerModule();
        NativeFunctions.SetUpNativeMethodPointers(module.BaseAddress);
        NavMeshQueryUtils.SetUpNativeMethodPointers(module.BaseAddress);
        PatchApplyCarveResults.Apply(module.BaseAddress);
        PatchNavMeshAgent.Apply(module.BaseAddress);
    }
}
