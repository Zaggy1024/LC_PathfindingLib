using System;
using System.Collections.Generic;
using System.Text;

namespace PathfindingLib.Jobs;

internal struct IndexAndSize(int index, int size)
{
    internal int index = index;
    internal int size = size;
}
