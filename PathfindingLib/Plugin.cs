using BepInEx;
using BepInEx.Logging;
using HarmonyLib;

using PathfindingLib.API;

namespace PathfindingLib;

[BepInPlugin(MOD_UNIQUE_NAME, MOD_NAME, MOD_VERSION)]
public class Plugin : BaseUnityPlugin
{
    internal const string MOD_NAME = "PathfindingLib";
    internal const string MOD_UNIQUE_NAME = "Zaggy1024." + MOD_NAME;
    internal const string MOD_VERSION = "0.0.1";

    private readonly Harmony harmony = new(MOD_UNIQUE_NAME);

    public static Plugin Instance { get; private set; }
    public new ManualLogSource Logger => base.Logger;

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
