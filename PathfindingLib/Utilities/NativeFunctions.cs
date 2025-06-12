using System;
using System.Runtime.InteropServices;

namespace PathfindingLib.Utilities;

internal static class NativeFunctions
{
    internal readonly static bool IsDebugBuild = UnityEngine.Debug.isDebugBuild;

    internal static void SetUpNativeMethodPointers(IntPtr baseAddress)
    {
        SetUpGetName(baseAddress);
    }

    // Delegate for Component::GetName()
    [UnmanagedFunctionPointer(CallingConvention.ThisCall)]
    private unsafe delegate char* GetNameDelegate(IntPtr thisComponent);

    private static GetNameDelegate getNameMethod;

    private static void SetUpGetName(IntPtr baseAddress)
    {
        var getNameOffset = 0x3714F0UL;
        if (IsDebugBuild)
            getNameOffset = 0x540f00UL;
        var getNameAddress = (IntPtr)((ulong)baseAddress + getNameOffset);

        getNameMethod = Marshal.GetDelegateForFunctionPointer<GetNameDelegate>(getNameAddress);
    }

    internal static unsafe string GetName(IntPtr thisComponent)
    {
        return Marshal.PtrToStringAnsi((IntPtr)getNameMethod(thisComponent));
    }
}
