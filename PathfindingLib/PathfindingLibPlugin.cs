using System.Diagnostics;
using System.Threading;

using BepInEx;
using BepInEx.Logging;
using HarmonyLib;

using PathfindingLib.Patches;
using PathfindingLib.Patches.Native;

namespace PathfindingLib;

[BepInPlugin(PluginGUID, PluginName, PluginVersion)]
public class PathfindingLibPlugin : BaseUnityPlugin
{
    public const string PluginName = "PathfindingLib";
    public const string PluginGUID = "Zaggy1024." + PluginName;
    public const string PluginVersion = "0.0.9";

    private readonly Harmony harmony = new(PluginGUID);

    internal static PathfindingLibPlugin Instance { get; private set; }
    internal new ManualLogSource Logger => base.Logger;

    internal static Thread MainThread = null;

    public void Awake()
    {
        Instance = this;

        ApplyAllNativePatches();

        harmony.PatchAll(typeof(PatchNavMeshSurface));

        MainThread = Thread.CurrentThread;
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
        PatchApplyCarvingResults.Apply(module.BaseAddress);
    }

    internal static new bool CurrentThreadIsMainThread()
    {
        return Thread.CurrentThread == MainThread;
    }
}
