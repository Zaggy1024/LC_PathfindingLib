using System;
using System.Collections;
using System.Collections.Generic;

using Unity.Collections;

namespace PathfindingLib.Utilities.Collections;

internal struct NativeArrayBuilder<T> : IEnumerable<T>, IDisposable where T : unmanaged
{
    private NativeArray<T> array;
    private int size;

    public int Capacity
    {
        get { return array.Length; }
        set
        {
            var newArray = new NativeArray<T>(value, Allocator.Persistent);
            array.CopyTo(newArray);
            array.Dispose();
            array = newArray;
        }
    }

    public void Add(in T item)
    {
        if (size >= Capacity)
        {
            int nextCapacity = 8;
            while (nextCapacity <= size)
                nextCapacity <<= 1;
            Capacity = nextCapacity;
        }

        array.GetRef(size++) = item;
    }

    public void Clear()
    {
        size = 0;
    }

    public readonly int Count => size;

    public readonly NativeArray<T> Get()
    {
        return array;
    }

    public readonly IEnumerator<T> GetEnumerator()
    {
        for (var i = 0; i < size; i++)
            yield return array[i];
    }

    readonly IEnumerator IEnumerable.GetEnumerator()
    {
        for (var i = 0; i < size; i++)
            yield return array[i];
    }

    public void Dispose()
    {
        array.Dispose();
    }
}
