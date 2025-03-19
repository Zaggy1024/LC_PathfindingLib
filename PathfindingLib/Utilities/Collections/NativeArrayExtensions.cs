using Unity.Collections;

namespace PathfindingLib.Utilities.Collections;

internal static class NativeArrayExtensions
{
    public static unsafe ref T GetRef<T>(this NativeArray<T> array, int index) where T : unmanaged
    {
        return ref ((T*)array.m_Buffer)[index];
    }
}
