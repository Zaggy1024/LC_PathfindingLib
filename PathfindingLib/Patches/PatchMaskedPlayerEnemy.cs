using System.Collections.Generic;
using System.Reflection;
using System.Reflection.Emit;

using GameNetcodeStuff;
using HarmonyLib;
using UnityEngine;

using PathfindingLib.API.SmartPathfinding;
using PathfindingLib.Utilities.Internal.IL;

namespace PathfindingLib.Patches;

[HarmonyPatch(typeof(MaskedPlayerEnemy))]
internal class PatchMaskedPlayerEnemy
{
    private static Dictionary<MaskedPlayerEnemy, SmartPathTask> tasks = [];

    enum GoToDestinationResult
    {
        Success,
        InProgress,
        Failure,
    }

    private static void UseTeleport(MaskedPlayerEnemy masked, EntranceTeleport teleport)
    {
        if (teleport.exitPoint == null && !teleport.FindExitPoint())
            return;

        masked.TeleportMaskedEnemyAndSync(teleport.exitPoint.position, setOutside: !teleport.isEntranceToBuilding);
    }

    private static GoToDestinationResult GoToDestination(MaskedPlayerEnemy masked, Vector3 targetPosition)
    {
        var result = GoToDestinationResult.InProgress;

        if (tasks.TryGetValue(masked, out var task))
        {
            if (!task.IsResultReady(0))
                return result;

            if (task.GetResult(0) is SmartPathDestination destination)
            {
                result = GoToDestinationResult.Success;
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
                    default:
                        result = GoToDestinationResult.Failure;
                        break;
                }

                PathfindingLibPlugin.Instance.Logger.LogInfo($"Destination is {masked.destination}");
            }
            else
            {
                result = GoToDestinationResult.Failure;
            }
        }

        task = new SmartPathTask();
        task.StartPathTask(masked.agent, masked.transform.position, targetPosition);
        tasks[masked] = task;
        return result;
    }

    private static void DoAIInterval(MaskedPlayerEnemy masked)
    {
        if (masked.isEnemyDead)
        {
            masked.agent.speed = 0f;
            return;
        }

        PlayerControllerB playerControllerB;
        switch (masked.currentBehaviourStateIndex)
        {
            // Roaming
            case 0:
                masked.LookAndRunRandomly(canStartRunning: true);
                playerControllerB = masked.CheckLineOfSightForClosestPlayer();
                if (playerControllerB != null)
                {
                    masked.LookAtPlayerServerRpc((int)playerControllerB.playerClientId);
                    masked.SetMovingTowardsTargetPlayer(playerControllerB);
                    masked.SwitchToBehaviourState(1);
                    break;
                }
                masked.interestInShipCooldown += masked.AIIntervalTime;
                if (masked.interestInShipCooldown >= 17f && Vector3.Distance(masked.transform.position, StartOfRound.Instance.elevatorTransform.position) < 22f)
                {
                    masked.SwitchToBehaviourState(2);
                    break;
                }
                if (Time.realtimeSinceStartup - masked.timeAtLastUsingEntrance > 3f)
                {
                    if (masked.GetClosestPlayer() == null)
                    {
                        var result = GoToDestination(masked, RoundManager.FindMainEntrancePosition(getTeleportPosition: true, getOutsideEntrance: !masked.isOutside));

                        if (result == GoToDestinationResult.InProgress)
                            return;
                        if (result == GoToDestinationResult.Success)
                        {
                            masked.StopSearch(masked.searchForPlayers);
                            return;
                        }
                    }
                }
                if (!masked.searchForPlayers.inProgress)
                    masked.StartSmartSearch(masked.transform.position, masked.searchForPlayers);
                break;
            // Chasing
            case 1:
                masked.LookAndRunRandomly(canStartRunning: true, onlySetRunning: true);
                playerControllerB = masked.CheckLineOfSightForClosestPlayer(70f, 50, 1, 3f);
                if (playerControllerB != null)
                {
                    masked.lostPlayerInChase = false;
                    masked.lostLOSTimer = 0f;
                    if (playerControllerB != masked.targetPlayer)
                    {
                        masked.SetMovingTowardsTargetPlayer(playerControllerB);
                        masked.LookAtPlayerServerRpc((int)playerControllerB.playerClientId);
                    }
                    if (masked.mostOptimalDistance > 17f)
                    {
                        if (masked.handsOut)
                        {
                            masked.handsOut = false;
                            masked.SetHandsOutServerRpc(setOut: false);
                        }
                        if (!masked.running)
                        {
                            masked.running = true;
                            masked.creatureAnimator.SetBool("Running", value: true);
                            masked.SetRunningServerRpc(running: true);
                        }
                    }
                    else if (masked.mostOptimalDistance < 6f)
                    {
                        if (!masked.handsOut)
                        {
                            masked.handsOut = true;
                            masked.SetHandsOutServerRpc(setOut: true);
                        }
                    }
                    else if (masked.mostOptimalDistance < 12f)
                    {
                        if (masked.handsOut)
                        {
                            masked.handsOut = false;
                            masked.SetHandsOutServerRpc(setOut: false);
                        }
                        if (masked.running && !masked.runningRandomly)
                        {
                            masked.running = false;
                            masked.creatureAnimator.SetBool("Running", value: false);
                            masked.SetRunningServerRpc(running: false);
                        }
                    }
                    break;
                }
                masked.lostLOSTimer += masked.AIIntervalTime;
                if (masked.lostLOSTimer > 10f)
                {
                    masked.SwitchToBehaviourState(0);
                    masked.targetPlayer = null;
                }
                else if (masked.lostLOSTimer > 3.5f)
                {
                    masked.lostPlayerInChase = true;
                    masked.StopLookingAtTransformServerRpc();
                    masked.targetPlayer = null;
                    if (masked.running)
                    {
                        masked.running = false;
                        masked.creatureAnimator.SetBool("Running", value: false);
                        masked.SetRunningServerRpc(running: false);
                    }
                    if (masked.handsOut)
                    {
                        masked.handsOut = false;
                        masked.SetHandsOutServerRpc(setOut: false);
                    }
                }
                break;
            case 2:
                {
                    if (!masked.isInsidePlayerShip)
                    {
                        masked.interestInShipCooldown -= masked.AIIntervalTime;
                    }
                    if (Vector3.Distance(masked.transform.position, StartOfRound.Instance.insideShipPositions[0].position) > 27f || masked.interestInShipCooldown <= 0f)
                    {
                        masked.SwitchToBehaviourState(0);
                        break;
                    }
                    PlayerControllerB closestPlayer = masked.GetClosestPlayer();
                    if (closestPlayer != null)
                    {
                        PlayerControllerB playerControllerB2 = masked.CheckLineOfSightForClosestPlayer(70f, 20, 0);
                        if (playerControllerB2 != null)
                        {
                            if (masked.stareAtTransform != playerControllerB2.gameplayCamera.transform)
                            {
                                masked.LookAtPlayerServerRpc((int)playerControllerB2.playerClientId);
                            }
                            masked.SetMovingTowardsTargetPlayer(playerControllerB2);
                            masked.SwitchToBehaviourState(1);
                        }
                        else if (masked.isInsidePlayerShip && closestPlayer.HasLineOfSightToPosition(masked.transform.position + Vector3.up * 0.7f, 4f, 20))
                        {
                            if (masked.stareAtTransform != closestPlayer.gameplayCamera.transform)
                            {
                                masked.LookAtPlayerServerRpc((int)closestPlayer.playerClientId);
                            }
                            masked.SetMovingTowardsTargetPlayer(closestPlayer);
                            masked.SwitchToBehaviourState(1);
                        }
                        else if (masked.mostOptimalDistance < 6f)
                        {
                            if (masked.stareAtTransform != closestPlayer.gameplayCamera.transform)
                            {
                                masked.stareAtTransform = closestPlayer.gameplayCamera.transform;
                                masked.LookAtPlayerServerRpc((int)closestPlayer.playerClientId);
                            }
                        }
                        else if (masked.mostOptimalDistance > 12f && masked.stareAtTransform != null)
                        {
                            masked.stareAtTransform = null;
                            masked.StopLookingAtTransformServerRpc();
                        }
                    }
                    masked.SetDestinationToPosition(masked.shipHidingSpot);
                    if (!masked.crouching && Vector3.Distance(masked.transform.position, masked.shipHidingSpot) < 0.4f)
                    {
                        masked.agent.speed = 0f;
                        masked.crouching = true;
                        masked.SetCrouchingServerRpc(setOut: true);
                    }
                    else if (masked.crouching && Vector3.Distance(masked.transform.position, masked.shipHidingSpot) > 1f)
                    {
                        masked.crouching = false;
                        masked.SetCrouchingServerRpc(setOut: false);
                    }
                    break;
                }
        }
        if (!(masked.targetPlayer != null) || !masked.PlayerIsTargetable(masked.targetPlayer) || (masked.currentBehaviourStateIndex != 1 && masked.currentBehaviourStateIndex != 2))
        {
            return;
        }
        if (masked.lostPlayerInChase)
        {
            masked.movingTowardsTargetPlayer = false;
            if (!masked.searchForPlayers.inProgress)
            {
                masked.StartSmartSearch(masked.transform.position, masked.searchForPlayers);
            }
        }
        else
        {
            if (masked.searchForPlayers.inProgress)
            {
                masked.StopSearch(masked.searchForPlayers);
            }
            masked.SetMovingTowardsTargetPlayer(masked.targetPlayer);
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
