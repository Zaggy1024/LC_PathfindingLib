using BepInEx;
using BepInEx.Logging;
using HarmonyLib;

using PathfindingLib.API;

namespace PathfindingLib;

[BepInPlugin(PluginName, PluginGUID, PluginVersion)]
public class PathfindingLibPlugin : BaseUnityPlugin
{
    public const string PluginName = "PathfindingLib";
    public const string PluginGUID = "Zaggy1024." + PluginName;
    public const string PluginVersion = "0.0.3";

    private readonly Harmony harmony = new(PluginGUID);

    internal static PathfindingLibPlugin Instance { get; private set; }
    internal new ManualLogSource Logger => base.Logger;

    public void Awake()
    {
        Instance = this;

        if (!NavMeshLock.Initialize(harmony))
        {
            Logger.LogInfo($"Failed to initialize navmesh concurrency safeties.");
            return;
        }
    }
}
