using System;

using Unity.Collections;

namespace PathfindingLib.Utilities.Collections;

internal static class NativeArrayExtensions
{
    internal static unsafe ref T GetRef<T>(this NativeArray<T> array, int index) where T : unmanaged
    {
        if (index < 0)
            throw new IndexOutOfRangeException("Index cannot be negative.");
        if (index >= array.Length)
            throw new IndexOutOfRangeException("Index is too large.");
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
}
