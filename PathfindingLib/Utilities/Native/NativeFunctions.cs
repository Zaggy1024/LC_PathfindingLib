using System;
using System.Runtime.InteropServices;

namespace PathfindingLib.Utilities.Native;

internal static class NativeFunctions
{
    internal readonly static bool IsDebugBuild = UnityEngine.Debug.isDebugBuild;

    internal static void SetUpNativeMethodPointers(IntPtr baseAddress)
    {
        SetUpGetName(baseAddress);
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
}
