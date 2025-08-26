using System;
using System.Runtime.InteropServices;

using UnityEngine;

namespace PathfindingLib.Utilities.Native;

internal static class NativeFunctions
{
    internal static void SetUpNativeMethodPointers()
    {
        SetUpGetName();
        SetUpGetPPtr();
        SetUpGetPosition();
        SetUpGetQueryExtents();
        SetUpGetLinkQueryExtents();
        SetUpGetCrowdAgent();
    }

    // Delegate for Component::GetName()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate char* GetNameDelegate(IntPtr component);

    private static GetNameDelegate getNameMethod;

    private static void SetUpGetName()
    {
        var functionOffset = 0x3714F0;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x540F00;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        getNameMethod = Marshal.GetDelegateForFunctionPointer<GetNameDelegate>(functionAddress);
    }

    internal static unsafe string GetName(IntPtr component)
    {
        return Marshal.PtrToStringAnsi((IntPtr)getNameMethod(component));
    }

    // Delegate for PPtr<>::operator->()
    // Called by OffMeshLink::UpdateMovedPositions(), which is
    // inlined into NavMeshManager::Update() in Release.
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private delegate IntPtr DerefPPtrDelegate(IntPtr thisPPtr);

    private static DerefPPtrDelegate derefPPtrMethod;

    private static void SetUpGetPPtr()
    {
        var functionOffset = 0x16A8C0;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x48B610;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        derefPPtrMethod = Marshal.GetDelegateForFunctionPointer<DerefPPtrDelegate>(functionAddress);
    }

    internal static unsafe IntPtr DerefPPtr(IntPtr pptr)
    {
        return derefPPtrMethod(pptr);
    }

    // Delegate for Transform::GetPosition()
    // Called immediately after PPtr<>::operator->() in the location mentioned above.
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate void GetPositionDelegate(IntPtr transform, Vector3* result);

    private static GetPositionDelegate getPositionMethod;

    private static void SetUpGetPosition()
    {
        var functionOffset = 0x667050;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0xC76330;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        getPositionMethod = Marshal.GetDelegateForFunctionPointer<GetPositionDelegate>(functionAddress);
    }

    internal static unsafe Vector3 GetPosition(IntPtr transform)
    {
        Vector3 result = default;
        getPositionMethod(transform, &result);
        return result;
    }

    // Delegate for NavMeshManager::GetQueryExtents(int agentTypeID)
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate void GetQueryExtentsDelegate(IntPtr navMeshManager, Vector3* result, int agentTypeID);

    private static GetQueryExtentsDelegate getQueryExtentsMethod;

    private static void SetUpGetQueryExtents()
    {
        var functionOffset = 0xA270C0;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x127CD40;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        getQueryExtentsMethod = Marshal.GetDelegateForFunctionPointer<GetQueryExtentsDelegate>(functionAddress);
    }

    internal static unsafe Vector3 GetQueryExtents(int agentTypeID)
    {
        var result = Vector3.zero;
        getQueryExtentsMethod(NativeHelpers.GetNavMeshManager(), &result, agentTypeID);
        return result;
    }

    // Delegate for NavMeshManager::GetLinkQueryExtents(int agentTypeID)
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate void GetLinkQueryExtentsDelegate(IntPtr navMeshManager, Vector3* result, int agentTypeID);

    private static GetLinkQueryExtentsDelegate getLinkQueryExtentsMethod;

    private static void SetUpGetLinkQueryExtents()
    {
        var functionOffset = 0xA27220;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x127C9D0;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        getLinkQueryExtentsMethod = Marshal.GetDelegateForFunctionPointer<GetLinkQueryExtentsDelegate>(functionAddress);
    }

    internal static unsafe Vector3 GetLinkQueryExtents(int agentTypeID)
    {
        var result = Vector3.zero;
        getLinkQueryExtentsMethod(NativeHelpers.GetNavMeshManager(), &result, agentTypeID);
        return result;
    }

    // Get CrowdAgent from NavMeshAgent:
    // Delegate for debug-only method CrowdManager::GetAgentByRef(ulong id)
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate IntPtr GetAgentByRefDelegate(IntPtr crowdManager, ulong id);

    private static GetAgentByRefDelegate GetAgentByRefMethod;

    // Delegate for release-only method NavMeshAgent::GetInternalAgent()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate IntPtr GetInternalAgentDelegate(IntPtr agent);

    private static GetInternalAgentDelegate GetInternalAgentMethod;

    private static void SetUpGetCrowdAgent()
    {
        if (NativeHelpers.IsDebugBuild)
            GetAgentByRefMethod = Marshal.GetDelegateForFunctionPointer<GetAgentByRefDelegate>(NativeHelpers.BaseAddress + 0x129CDD0);
        else
            GetInternalAgentMethod = Marshal.GetDelegateForFunctionPointer<GetInternalAgentDelegate>(NativeHelpers.BaseAddress + 0xA39EE0);
    }

    private static unsafe IntPtr GetCrowdAgent(IntPtr agent)
    {
        if (GetInternalAgentMethod != null)
            return GetInternalAgentMethod(agent);

        return GetAgentByRefMethod(NativeHelpers.GetCrowdManager(), NativeHelpers.GetAgentID(agent));
    }

    internal static unsafe Vector3 GetAgentPosition(IntPtr agent)
    {
        var internalAgent = GetCrowdAgent(agent);
        if (internalAgent == IntPtr.Zero)
            return Vector3.positiveInfinity;
        return *(Vector3*)internalAgent;
    }
}
