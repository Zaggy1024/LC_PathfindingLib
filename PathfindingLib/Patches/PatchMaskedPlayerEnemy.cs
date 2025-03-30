using System;
using System.Collections.Generic;
using System.Reflection;
using System.Reflection.Emit;

using HarmonyLib;
using UnityEngine;

using PathfindingLib.API.SmartPathfinding;
using PathfindingLib.Utilities.Internal.IL;

namespace PathfindingLib.Patches;

[HarmonyPatch(typeof(MaskedPlayerEnemy))]
internal class PatchMaskedPlayerEnemy
{
    private static Dictionary<MaskedPlayerEnemy, SmartPathTask> tasks = [];

    private static void UseTeleport(MaskedPlayerEnemy masked, EntranceTeleport teleport)
    {
        if (teleport.exitPoint == null && !teleport.FindExitPoint())
            return;

        masked.agent.Warp(teleport.exitPoint.position);
        masked.SetEnemyOutside(!teleport.isEntranceToBuilding);
    }

    private static void DoAIInterval(MaskedPlayerEnemy masked)
    {
        try
        {
            var targetPosition = StartOfRound.Instance.localPlayerController.transform.position;

            if (tasks.TryGetValue(masked, out var task))
            {
                if (!task.IsComplete)
                    return;
                if (task.Result.HasValue)
                {
                    var destination = task.Result.Value;
                    PathfindingLibPlugin.Instance.Logger.LogInfo($"Destination result is: {destination}");
                    switch (destination.Type)
                    {
                        case SmartDestinationType.DirectToDestination:
                            masked.SetDestinationToPosition(targetPosition);
                            break;
                        case SmartDestinationType.Elevator:
                            masked.SetDestinationToPosition(destination.Position);

                            if (Vector3.Distance(masked.transform.position, destination.Position) < 1f)
                                destination.ElevatorFloor.CallElevator();
                            break;
                        case SmartDestinationType.EntranceTeleport:
                            masked.SetDestinationToPosition(destination.Position);

                            if (Vector3.Distance(masked.transform.position, destination.Position) < 1f)
                                UseTeleport(masked, destination.EntranceTeleport);
                            break;
                    }

                    PathfindingLibPlugin.Instance.Logger.LogInfo($"Destination is {masked.destination}");
                }
            }

            task = SmartPathTask.StartPathTask(masked.transform.position, targetPosition, masked.agent);
            tasks[masked] = task;
        }
        catch (Exception ex)
        {
            PathfindingLibPlugin.Instance.Logger.LogFatal(ex);
        }
    }

    [HarmonyTranspiler]
    [HarmonyPatch(nameof(MaskedPlayerEnemy.DoAIInterval))]
    private static IEnumerable<CodeInstruction> DoAIIntervalTranspiler(IEnumerable<CodeInstruction> instructions)
    {
        var injector = new ILInjector(instructions)
            .Find([
                ILMatcher.Ldarg(0),
                ILMatcher.Call(typeof(EnemyAI).GetMethod(nameof(EnemyAI.DoAIInterval))),
            ])
            .GoToMatchEnd();
        return injector
            .Remove(injector.Instructions.Count - injector.Index)
            .Insert([
                new(OpCodes.Ldarg_0),
                new(OpCodes.Call, typeof(PatchMaskedPlayerEnemy).GetMethod(nameof(DoAIInterval), BindingFlags.NonPublic | BindingFlags.Static)),
                new(OpCodes.Ret),
            ])
            .ReleaseInstructions();
    }
}
