using System;
using System.Collections.Generic;

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace PathfindingLib.Utilities.Collections;

internal static class NativeListExtensions
{
    internal static unsafe void Insert<T>(this ref UnsafeList<T> self, int index, T element) where T : unmanaged
    {
        self.InsertRangeWithBeginEnd(index, index + 1);
        self[index] = element;
    }

    public static unsafe int AddOrdered<T>(this ref UnsafeList<T> self, T item, IComparer<T> comparer) where T : unmanaged
    {
        var index = NativeSortExtension.BinarySearch(self.Ptr, self.Length, item, comparer);
        if (index < 0) index = ~index;
        self.Insert(index, item);
        return index;
    }

    public static unsafe int FindIndex<T>(this in UnsafeList<T> self, Predicate<T> predicate) where T : unmanaged
    {
        for (var i = 0; i < self.Length; i++)
        {
            if (predicate(self[i]))
                return i;
        }

        return -1;
    }
}
