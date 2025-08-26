using System;

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace PathfindingLib.Utilities.Collections;

internal static class NativeArrayExtensions
{
    internal static unsafe ref T GetRef<T>(this NativeArray<T> array, int index) where T : unmanaged
    {
        if (index < 0)
            throw new IndexOutOfRangeException("Index cannot be negative.");
        if (index >= array.Length)
            throw new IndexOutOfRangeException($"Index {index} is not within array length of {array.Length}.");
        return ref ((T*)array.m_Buffer)[index];
    }

    internal static void SetAllElements<T>(this NativeArray<T> array, T value) where T : unmanaged
    {
        unsafe
        {
            var ptr = (T*)array.m_Buffer;
            var count = array.Length;

            for (var i = 0; i < count; i++)
                ptr[i] = value;
        }
    }

    internal static unsafe void CopyFrom<T>(this NativeArray<T> to, Span<T> from) where T : struct
    {
        if (from.Length == 0)
            return;
        UnsafeUtility.MemCpy(to.GetUnsafePtr(), UnsafeUtility.AddressOf(ref from[0]), from.Length);
    }
}
