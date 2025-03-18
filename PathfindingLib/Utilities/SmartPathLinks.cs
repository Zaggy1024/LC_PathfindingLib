using PathfindingLib.API.Smart;
using PathfindingLib.Jobs;
using System.Collections.Generic;

using Unity.Collections;
using UnityEngine;

namespace PathfindingLib.Utilities;

internal static class SmartPathLinks
{
    internal struct LinkData(SmartPathDestination origin)
    {
        internal SmartPathDestination origin = origin;
        internal List<Transform> destinations = new(1);
    }

    internal static readonly LinkedList<LinkData> links = [];
    internal static readonly Dictionary<Transform, LinkedListNode<LinkData>> linkLookup = [];

    internal static void RegisterEntranceTeleport(EntranceTeleport teleport, Transform exit)
    {
        if (linkLookup.ContainsKey(teleport.entrancePoint))
            return;

        var node = links.AddLast(new LinkData(SmartPathDestination.EntranceTeleportDestination(teleport)));
        node.Value.destinations.Add(exit);
        linkLookup[teleport.entrancePoint] = node;
    }

    internal static void UnregisterEntranceTeleport(EntranceTeleport teleport)
    {
        if (!linkLookup.TryGetValue(teleport.entrancePoint, out var node))
        {
            PathfindingLibPlugin.Instance.Logger.LogWarning($"Attempted to remove {teleport} from smart pathfinding data without it being present.");
            return;
        }

        links.Remove(node);
        linkLookup.Remove(teleport.entrancePoint);
    }
}
