using UnityEngine;
using UnityEngine.AI;

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
        if (agent.isOnOffMeshLink)
            return agent.currentOffMeshLinkData.endPos;

        return agent.transform.position;
    }
}
