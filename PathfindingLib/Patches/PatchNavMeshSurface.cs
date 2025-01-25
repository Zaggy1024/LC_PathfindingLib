using System.Collections.Generic;
using System.Reflection;
using System.Reflection.Emit;

using HarmonyLib;
using Unity.AI.Navigation;
using UnityEngine;

using PathfindingLib.API;
using PathfindingLib.Utilities.Internal.IL;

namespace PathfindingLib.Patches;

[HarmonyPatch(typeof(NavMeshSurface))]
internal static class PatchNavMeshSurface
{
    private static readonly MethodInfo m_BeginNavMeshWrite = typeof(NavMeshLock).GetMethod(nameof(NavMeshLock.BeginWrite), BindingFlags.Public | BindingFlags.Static);
    private static readonly MethodInfo m_EndNavMeshWrite = typeof(NavMeshLock).GetMethod(nameof(NavMeshLock.EndWrite), BindingFlags.Public | BindingFlags.Static);

    [HarmonyTranspiler]
    [HarmonyPatch(typeof(NavMeshSurface), nameof(NavMeshSurface.BuildNavMesh))]
    [HarmonyPatch(typeof(NavMeshSurface), nameof(NavMeshSurface.UpdateActive))]
    [HarmonyPatch(typeof(NavMeshSurface), nameof(NavMeshSurface.AddData))]
    [HarmonyPatch(typeof(NavMeshSurface), nameof(NavMeshSurface.RemoveData))]
    private static IEnumerable<CodeInstruction> LockWriteForDurationOfMethodTranspiler(IEnumerable<CodeInstruction> instructions)
    {
        return new ILInjector(instructions)
            .Insert([new(OpCodes.Call, m_BeginNavMeshWrite)])
            .GoToEnd()
            .ReverseFind([
                ILMatcher.Opcode(OpCodes.Ret)
            ])
            .Insert([new(OpCodes.Call, m_EndNavMeshWrite)])
            .ReleaseInstructions();
    }

    [HarmonyTranspiler]
    [HarmonyPatch(nameof(NavMeshSurface.UpdateDataIfTransformChanged))]
    private static IEnumerable<CodeInstruction> UpdateDataIfTransformChangedTranspiler(IEnumerable<CodeInstruction> instructions)
    {
        var injector = new ILInjector(instructions)
            .Find([
                ILMatcher.Ldarg(0),
                ILMatcher.Call(typeof(NavMeshSurface).GetMethod(nameof(NavMeshSurface.RemoveData))),
            ]);
        if (!injector.IsValid)
        {
            PathfindingLibPlugin.Instance.Logger.LogError($"Failed to find the call to {nameof(NavMeshSurface.RemoveData)} in {nameof(NavMeshSurface)}.{nameof(NavMeshSurface.UpdateDataIfTransformChanged)}.");
            return instructions;
        }

        injector
            .Insert([
                new(OpCodes.Call, m_BeginNavMeshWrite),
            ])
            .Find([
                ILMatcher.Ldarg(0),
                ILMatcher.Call(typeof(NavMeshSurface).GetMethod(nameof(NavMeshSurface.AddData))),
            ])
            .GoToMatchEnd();
        if (!injector.IsValid)
        {
            PathfindingLibPlugin.Instance.Logger.LogError($"Failed to find the call to {nameof(NavMeshSurface.AddData)} in {nameof(NavMeshSurface)}.{nameof(NavMeshSurface.UpdateDataIfTransformChanged)}.");
            return instructions;
        }

        return injector
            .Insert([
                new(OpCodes.Call, m_EndNavMeshWrite),
            ])
            .ReleaseInstructions();
    }

    [HarmonyTranspiler]
    [HarmonyPatch(nameof(NavMeshSurface.UpdateNavMesh))]
    private static IEnumerable<CodeInstruction> UpdateNavMeshTranspiler(IEnumerable<CodeInstruction> instructions)
    {
        return new ILInjector(instructions)
            .Insert([
                new(OpCodes.Call, m_BeginNavMeshWrite),
            ])
            .Find(ILMatcher.Opcode(OpCodes.Ret))
            .Insert([
                new(OpCodes.Dup),
                new(OpCodes.Call, typeof(PatchNavMeshSurface).GetMethod(nameof(EndNavMeshWriteAtEndOfAsyncOperation), BindingFlags.NonPublic | BindingFlags.Static, null, [typeof(AsyncOperation)], null)),
            ])
            .ReleaseInstructions();
    }

    private static void EndNavMeshWriteAtEndOfAsyncOperation(AsyncOperation operation)
    {
        operation.completed += _ => NavMeshLock.EndWrite();
    }
}
