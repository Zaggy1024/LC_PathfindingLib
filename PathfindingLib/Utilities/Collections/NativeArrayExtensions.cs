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
}
