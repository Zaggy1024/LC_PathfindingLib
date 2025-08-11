using System;
using System.Runtime.InteropServices;

using UnityEngine;

namespace PathfindingLib.Utilities.Native;

internal static class NativeFunctions
{
    internal readonly static bool IsDebugBuild = UnityEngine.Debug.isDebugBuild;

    internal static void SetUpNativeMethodPointers(IntPtr baseAddress)
    {
        SetUpGetName(baseAddress);
        SetUpGetPPtr(baseAddress);
        SetUpGetPosition(baseAddress);
    }

    // Delegate for Component::GetName()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate char* GetNameDelegate(IntPtr component);

    private static GetNameDelegate getNameMethod;

    private static void SetUpGetName(IntPtr baseAddress)
    {
        var functionOffset = 0x3714F0;
        if (IsDebugBuild)
            functionOffset = 0x540F00;
        var functionAddress = baseAddress + functionOffset;

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

    private static void SetUpGetPPtr(IntPtr baseAddress)
    {
        var functionOffset = 0x16A8C0;
        if (IsDebugBuild)
            functionOffset = 0x48B610;
        var functionAddress = baseAddress + functionOffset;

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

    private static void SetUpGetPosition(IntPtr baseAddress)
    {
        var functionOffset = 0x667050;
        if (IsDebugBuild)
            functionOffset = 0xC76330;
        var functionAddress = baseAddress + functionOffset;

        getPositionMethod = Marshal.GetDelegateForFunctionPointer<GetPositionDelegate>(functionAddress);
    }

    internal static unsafe Vector3 GetPosition(IntPtr transform)
    {
        var result = Vector3.zero;
        getPositionMethod(transform, &result);
        return result;
    }
}
