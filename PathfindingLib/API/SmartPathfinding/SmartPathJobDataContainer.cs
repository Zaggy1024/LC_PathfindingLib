using System;
using System.Collections.Generic;

using Unity.Collections;
using UnityEngine;
using UnityEngine.Pool;

using PathfindingLib.API.Smart;
using PathfindingLib.Jobs;
using PathfindingLib.Utilities;

namespace PathfindingLib.API.SmartPathfinding;

internal class SmartPathJobDataContainer : IDisposable
{
    private static readonly ObjectPool<SmartPathJobDataContainer> pool = new(() => new(), actionOnDestroy: v => v.Dispose());

    internal static readonly List<string> linkNames = [];

    internal readonly List<Vector3> linkDestinationsManaged = [];
    internal readonly List<SmartPathDestination> linkOriginDestinations = [];
    internal NativeArray<Vector3> linkOrigins;
    internal NativeArray<IndexAndSize> linkDestinationSlices;
    internal NativeArray<Vector3> linkDestinations;
    internal int linkCount;
    internal int destinationCount;

    internal static SmartPathJobDataContainer GetJobData()
    {
        var jobData = pool.Get();
        jobData.FillJobData();
        return jobData;
    }

    internal static void ReleaseJobData(ref SmartPathJobDataContainer jobData)
    {
        jobData.Clear();
        pool.Release(jobData);
        jobData = null;
    }

    internal void FillJobData()
    {
        Dispose();

        linkCount = SmartPathLinks.links.Count;
        if (linkDestinationsManaged.Capacity < linkCount)
        {
            linkDestinationsManaged.Capacity = linkCount;
            linkOriginDestinations.Capacity = linkCount;
        }
        if (linkCount > linkOrigins.Length)
        {
            linkOrigins = new NativeArray<Vector3>(linkCount, Allocator.Persistent);
            linkDestinationSlices = new NativeArray<IndexAndSize>(linkCount, Allocator.Persistent);
        }

        var fillNames = linkNames.Count == 0;

        var teleportIndex = 0;
        var teleportDestinationsIndex = 0;
        for (var node = SmartPathLinks.links.First; node != null; node = node.Next)
        {
            if (fillNames)
                linkNames.Add(node.Value.origin.ToString());

            linkOriginDestinations.Add(node.Value.origin);

            linkOrigins[teleportIndex] = node.Value.origin.Position;

            var destinations = node.Value.destinations;
            linkDestinationSlices[teleportIndex] = new(teleportDestinationsIndex, destinations.Count);

            foreach (var destination in node.Value.destinations)
                linkDestinationsManaged.Add(destination.position);

            teleportIndex++;
            teleportDestinationsIndex += destinations.Count;
        }

        var teleportDestinationsArray = NoAllocHelpers.ExtractArrayFromListT(linkDestinationsManaged);
        linkDestinations = new(linkDestinationsManaged.Count, Allocator.Persistent);
        NativeArray<Vector3>.Copy(teleportDestinationsArray, linkDestinations);
        destinationCount = linkDestinationsManaged.Count;
    }

    private void Clear()
    {
        linkDestinationsManaged.Clear();
        linkOriginDestinations.Clear();
    }

    public void Dispose()
    {
        linkOrigins.Dispose();
        linkDestinationSlices.Dispose();
        linkDestinations.Dispose();

        linkOrigins = default;
        linkDestinationSlices = default;
        linkDestinations = default;
    }
}
