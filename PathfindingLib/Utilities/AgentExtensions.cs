using System;

using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.AI;

using PathfindingLib.Utilities.Native;

namespace PathfindingLib.Utilities;

public static class AgentExtensions
{
    /// <summary>
    /// Gets the position that a call to <see cref="NavMeshAgent.CalculatePath"/> would use as the start of the
    /// path.
    /// 
    /// This will return the end of the agent's current navmesh link so that paths can be calculated while they
    /// are still in transit across the link.
    /// </summary>
    /// <param name="agent">The agent that must calculate a path.</param>
    /// <returns>The position at which the path should originate.</returns>
    public static Vector3 GetPathOrigin(this NavMeshAgent agent)
    {
        var ptr = agent.GetCachedPtr();
        if (ptr == null)
            return agent.transform.position;
        return NativeFunctions.GetAgentPosition(ptr);
    }

    /// <summary>
    /// Get the parameters for a NavMeshQuery to use when calculating a path.
    /// </summary>
    /// <param name="agent">The agent to retrieve the parameters from.</param>
    /// <param name="position">The position of the agent according to the crowd manager. This should be passed as the start of the path.</param>
    /// <param name="agentTypeID">The agent type, defining which navmesh it will use.</param>
    /// <param name="areaMask">The area mask to limit the agent's movement to certain area types.</param>
    /// <param name="costs">The costs for this agent to traverse each area type. This may contain either 32 elements or 0.</param>
    public static unsafe void GetQueryFilter(this NavMeshAgent agent, out int agentTypeID, out int areaMask, out Span<float> costs)
    {
        agentTypeID = -1;
        areaMask = -1;
        costs = default;

        var nativeAgent = agent.GetCachedPtr();
        if (nativeAgent == IntPtr.Zero)
            return;

        var filter = NativeFunctions.GetAgentQueryFilter(nativeAgent);
        if (filter == null)
            return;

        agentTypeID = (int)filter->AgentType;
        areaMask = (int)filter->AreaMask;
        costs = new Span<float>(UnsafeUtility.AddressOf(ref filter->CostsStart), 32);
    }
}
