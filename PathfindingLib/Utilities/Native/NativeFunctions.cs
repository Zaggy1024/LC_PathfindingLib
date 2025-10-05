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
        SetUpGetAgentQueryFilter();
        SetUpGrowString();
        SetUpScriptingWrapperFor();
    }

    // Delegate for Component::GetName()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate char* GetNameDelegate(IntPtr component);

    private static GetNameDelegate getNameMethod;

    private static void SetUpGetName()
    {
        var functionOffset = 0x376D30;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x54D410;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        getNameMethod = Marshal.GetDelegateForFunctionPointer<GetNameDelegate>(functionAddress);
    }

    internal static unsafe string GetName(IntPtr component)
    {
        return Marshal.PtrToStringAnsi((IntPtr)getNameMethod(component));
    }

    // Delegate for PPtr<Transform>::operator->()
    // Called by OffMeshLink::UpdateMovedPositions(), which is
    // inlined into NavMeshManager::Update() in Release.
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private delegate IntPtr DerefPPtrDelegate(IntPtr thisPPtr);

    private static DerefPPtrDelegate derefPPtrMethod;

    private static void SetUpGetPPtr()
    {
        var functionOffset = 0x153DE0;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x22D160;
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
        var functionOffset = 0x679BD0;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0xCF2630;
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
        var functionOffset = 0xA49E30;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x12C7830;
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
        var functionOffset = 0xA49F90;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x12C7520;
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

    private static GetAgentByRefDelegate getAgentByRefMethod;

    // Delegate for release-only method NavMeshAgent::GetInternalAgent()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate IntPtr GetInternalAgentDelegate(IntPtr agent);

    private static GetInternalAgentDelegate getInternalAgentMethod;

    private static void SetUpGetCrowdAgent()
    {
        if (NativeHelpers.IsDebugBuild)
            getAgentByRefMethod = Marshal.GetDelegateForFunctionPointer<GetAgentByRefDelegate>(NativeHelpers.BaseAddress + 0x12E7C50);
        else
            getInternalAgentMethod = Marshal.GetDelegateForFunctionPointer<GetInternalAgentDelegate>(NativeHelpers.BaseAddress + 0xA5D290);
    }

    private static unsafe IntPtr GetCrowdAgent(IntPtr agent)
    {
        if (getInternalAgentMethod != null)
            return getInternalAgentMethod(agent);

        return getAgentByRefMethod(NativeHelpers.GetCrowdManager(), NativeHelpers.GetAgentID(agent));
    }

    internal static unsafe Vector3 GetAgentPosition(IntPtr agent)
    {
        var internalAgent = GetCrowdAgent(agent);
        if (internalAgent == IntPtr.Zero)
            return Vector3.positiveInfinity;
        return *(Vector3*)internalAgent;
    }

    // Get QueryFilter from NavMeshAgent:
    // Delegate for debug-only method CrowdManager::GetAgentFilter(ulong id)
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate QueryFilter* GetAgentFilterDelegate(IntPtr crowdManager, ulong id);

    private static GetAgentFilterDelegate getAgentFilterMethod;

    // Delegate for release-only method NavMeshAgent::GetFilter()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate QueryFilter* GetFilterDelegate(IntPtr agent);

    private static GetFilterDelegate getFilterMethod;

    private static void SetUpGetAgentQueryFilter()
    {
        if (NativeHelpers.IsDebugBuild)
            getAgentFilterMethod = Marshal.GetDelegateForFunctionPointer<GetAgentFilterDelegate>(NativeHelpers.BaseAddress + 0x12E7CA0);
        else
            getFilterMethod = Marshal.GetDelegateForFunctionPointer<GetFilterDelegate>(NativeHelpers.BaseAddress + 0xA5D200);
    }

    internal static unsafe QueryFilter* GetAgentQueryFilter(IntPtr agent)
    {
        if (getFilterMethod != null)
            return getFilterMethod(agent);

        return getAgentFilterMethod(NativeHelpers.GetCrowdManager(), NativeHelpers.GetAgentID(agent));
    }

    // Delegate for core::StringStorageDefault<char>::grow()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate char* GrowStringDelegate(IntPtr str, ulong size);

    private static GrowStringDelegate growStringMethod;

    private static void SetUpGrowString()
    {
        var functionOffset = 0x15DE30;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x232870;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        growStringMethod = Marshal.GetDelegateForFunctionPointer<GrowStringDelegate>(functionAddress);
    }

    internal static unsafe char* GrowString(IntPtr str, ulong size)
    {
        return growStringMethod(str, size);
    }

    // Delegate for Scripting::ScriptingWrapperFor()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate IntPtr* ScriptingWrapperForDelegate(IntPtr* result, IntPtr nativeObj);

    private static ScriptingWrapperForDelegate scriptingWrapperForMethod;

    private static void SetUpScriptingWrapperFor()
    {
        var functionOffset = 0x78E9D0;
        if (NativeHelpers.IsDebugBuild)
            functionOffset = 0x101CD70;
        var functionAddress = NativeHelpers.BaseAddress + functionOffset;

        scriptingWrapperForMethod = Marshal.GetDelegateForFunctionPointer<ScriptingWrapperForDelegate>(functionAddress);
    }

    internal static unsafe T ScriptingWrapperFor<T>(IntPtr nativeObj)
    {
        IntPtr pointer = IntPtr.Zero;
        IntPtr* result = scriptingWrapperForMethod(&pointer, nativeObj);
#pragma warning disable CS8500
        return *(T*)result;
#pragma warning restore CS8500
    }
}
