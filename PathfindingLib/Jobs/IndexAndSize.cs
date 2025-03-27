using System.Collections.Generic;

namespace PathfindingLib.Jobs;

internal struct IndexAndSize(int index, int size)
{
    internal int index = index;
    internal int size = size;

    internal readonly bool IsValid<T>(List<T> list)
    {
        return index >= 0 && index < list.Count && index + size <= list.Count;
    }
}
