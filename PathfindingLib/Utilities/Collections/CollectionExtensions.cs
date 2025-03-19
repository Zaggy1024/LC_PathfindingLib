using System.Collections.Generic;

namespace PathfindingLib.Utilities.Collections;

public static class CollectionExtensions
{
    public static int AddOrdered<T>(this List<T> self, T item, IComparer<T> comparer = default)
    {
        var index = self.BinarySearch(item, comparer);
        if (index < 0) index = ~index;
        self.Insert(index, item);
        return index;
    }
}
