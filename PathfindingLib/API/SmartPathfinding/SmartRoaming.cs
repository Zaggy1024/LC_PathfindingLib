using System.Collections;
using System.Collections.Generic;

using UnityEngine;

namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public static class SmartRoaming
{
    public static bool useVanilla = false;

    public delegate void SmartTraversalFunction(EnemyAI enemy, in SmartPathDestination destination);

    private static readonly List<Vector3> unsearchedNodePositions = [];

    public struct Config(SmartPathfindingLinkFlags allowedLinks)
    {
        /// <summary>
        /// The links that the agent is allowed to use when roaming.
        /// </summary>
        public SmartPathfindingLinkFlags allowedLinks = allowedLinks;
        /// <summary>
        /// The amount of time that an agent is allowed to spend navigating to a single node in the search routine.
        /// </summary>
        public float timeToNavigateToCurrentDestination = 16f;
        /// <summary>
        /// Whether to count the time spent in smart link traversal towards <see cref="timeToNavigateToCurrentDestination"/>.
        /// </summary>
        public bool countLinkTraversalForNavigationTime = false;
    }

    public static void StartSmartSearch(this EnemyAI enemy, Vector3 startOfSearch, Config config, SmartTraversalFunction traversalFunction, AISearchRoutine? newSearch = null)
    {
        newSearch ??= new AISearchRoutine();

        enemy.StopSearch(enemy.currentSearch);
        enemy.movingTowardsTargetPlayer = false;

        newSearch.currentSearchStartPosition = startOfSearch;
        newSearch.startedSearchAtSelf = Vector3.Distance(startOfSearch, enemy.transform.position) < 2;
        if (newSearch.unsearchedNodes.Count == 0)
            newSearch.unsearchedNodes.AddRange(enemy.allAINodes);
        enemy.currentSearch = newSearch;

        enemy.searchRoutineRandom = new System.Random(enemy.RoundUpToNearestFive(startOfSearch.x) + enemy.RoundUpToNearestFive(startOfSearch.z));
        if (useVanilla)
            enemy.searchCoroutine = enemy.StartCoroutine(enemy.CurrentSearchCoroutine());
        else
            enemy.searchCoroutine = enemy.StartCoroutine(enemy.CurrentSmartSearchCoroutine(config, traversalFunction));
        enemy.currentSearch.inProgress = true;
    }

    public static void StartSmartSearch(this EnemyAI enemy, Vector3 startOfSearch, SmartPathfindingLinkFlags allowedLinks, SmartTraversalFunction traversalFunction, AISearchRoutine? newSearch = null)
    {
        var config = new Config(allowedLinks);
        StartSmartSearch(enemy, startOfSearch, config, traversalFunction, newSearch);
    }

    private static void SendSmartAIToSmartDestination(EnemyAI enemy, in SmartPathDestination destination)
    {
        ((ISmartAI)enemy).GoToSmartPathDestination(in destination);
    }

    public static void StartSmartSearch<T>(this T enemy, Vector3 startOfSearch, Config config, AISearchRoutine? newSearch = null) where T : EnemyAI, ISmartAI
    {
        enemy.StartSmartSearch(startOfSearch, config, SendSmartAIToSmartDestination, newSearch);
    }

    public static void StartSmartSearch<T>(this T enemy, Vector3 startOfSearch, SmartPathfindingLinkFlags allowedLinks, AISearchRoutine? newSearch = null) where T : EnemyAI, ISmartAI
    {
        enemy.StartSmartSearch(startOfSearch, allowedLinks, SendSmartAIToSmartDestination, newSearch);
    }

    public static IEnumerator CurrentSmartSearchCoroutine(this EnemyAI enemy, Config config, SmartTraversalFunction traversalFunction)
    {
        yield return null;

        var currentNodePathTask = new SmartPathTask();

        while (enemy.searchCoroutine != null && enemy.IsOwner)
        {
            yield return null;

            // If all nodes have been searched, loop the search or exit it.
            if (enemy.currentSearch.unsearchedNodes.Count == 0)
            {
                enemy.FinishedCurrentSearchRoutine();

                if (!enemy.currentSearch.loopSearch)
                {
                    enemy.currentSearch.inProgress = false;
                    enemy.searchCoroutine = null;
                    yield break;
                }

                enemy.currentSearch.unsearchedNodes.Clear();
                enemy.currentSearch.unsearchedNodes.AddRange(enemy.allAINodes);
                enemy.currentSearch.timesFinishingSearch++;
                enemy.currentSearch.nodesEliminatedInCurrentSearch = 0;
                yield return new WaitForSeconds(1);
            }

            // Select the next node to move to.
            if (enemy.currentSearch.choseTargetNode && enemy.currentSearch.unsearchedNodes.Contains(enemy.currentSearch.nextTargetNode))
            {
                enemy.currentSearch.currentTargetNode = enemy.currentSearch.nextTargetNode;
            }
            else
            {
                enemy.currentSearch.waitingForTargetNode = true;
                enemy.StartCalculatingNextSmartTargetNode(config.allowedLinks);

                yield return new WaitUntil(() => enemy.currentSearch.choseTargetNode);
            }

            enemy.currentSearch.waitingForTargetNode = false;
            if (enemy.currentSearch.unsearchedNodes.Count == 0 || enemy.currentSearch.currentTargetNode == null)
                continue;

            // Go to the selected node.
            enemy.currentSearch.unsearchedNodes.Remove(enemy.currentSearch.currentTargetNode);

            var wasWaitingOnLink = false;
            void GoToCurrentDestination()
            {
                if (currentNodePathTask.IsResultReady(0))
                {
                    if (currentNodePathTask.GetResult(0) is SmartPathDestination destination)
                    {
                        traversalFunction(enemy, in destination);

                        wasWaitingOnLink = destination.Type == SmartDestinationType.Elevator && enemy.agent.velocity.sqrMagnitude < 0.25f;
                    }
                    else
                    {
                        wasWaitingOnLink = false;
                    }
                }
                currentNodePathTask.StartPathTask(enemy.agent, enemy.transform.position, enemy.currentSearch.currentTargetNode.transform.position, config.allowedLinks);
            }
            GoToCurrentDestination();

            // Eliminate all nodes within the area of the current target.
            var currentTargetPos = enemy.currentSearch.currentTargetNode.transform.position;
            var searchPrecisionSqr = enemy.currentSearch.searchPrecision * enemy.currentSearch.searchPrecision;
            for (var i = enemy.currentSearch.unsearchedNodes.Count - 1; i >= 0; i--)
            {
                if ((currentTargetPos - enemy.currentSearch.unsearchedNodes[i].transform.position).sqrMagnitude < searchPrecisionSqr)
                    enemy.EliminateNodeFromSearch(i);

                if (i % 10 == 0)
                {
                    yield return null;
                    currentTargetPos = enemy.currentSearch.currentTargetNode.transform.position;
                    searchPrecisionSqr = enemy.currentSearch.searchPrecision * enemy.currentSearch.searchPrecision;
                }
            }

            // Calculate the next node ahead of time.
            enemy.StartCalculatingNextSmartTargetNode(config.allowedLinks);

            // Wait for the current target to be reached.
            currentTargetPos = enemy.currentSearch.currentTargetNode.transform.position;
            searchPrecisionSqr = enemy.currentSearch.searchPrecision * enemy.currentSearch.searchPrecision;

            var pathingTime = config.timeToNavigateToCurrentDestination;
            var checkTime = 0f;
            while (enemy.searchCoroutine != null && pathingTime >= 0)
            {
                if (config.countLinkTraversalForNavigationTime || !wasWaitingOnLink)
                    pathingTime -= Time.deltaTime;

                if (checkTime <= 0)
                {
                    if (enemy.currentSearch.onlySearchNodesInLOS && Physics.Linecast(currentTargetPos, enemy.currentSearch.currentSearchStartPosition, StartOfRound.Instance.collidersAndRoomMaskAndDefault, QueryTriggerInteraction.Ignore))
                        break;
                    checkTime = 0.5f;
                }

                checkTime -= Time.deltaTime;

                yield return null;
                if (enemy.searchCoroutine == null)
                    break;

                if ((currentTargetPos - enemy.transform.position).sqrMagnitude < searchPrecisionSqr)
                {
                    enemy.ReachedNodeInSearch();
                    break;
                }

                GoToCurrentDestination();
            }
        }

        if (!enemy.IsOwner)
            enemy.StopSearch(enemy.currentSearch);
    }

    public static void StartCalculatingNextSmartTargetNode(this EnemyAI enemy, SmartPathfindingLinkFlags allowedLinks)
    {
        var search = enemy.currentSearch;

        if (enemy.chooseTargetNodeCoroutine == null)
        {
            search.choseTargetNode = false;
            enemy.chooseTargetNodeCoroutine = enemy.StartCoroutine(enemy.ChooseNextNodeInSmartSearchRoutine(allowedLinks));
            return;
        }

        if (!search.calculatingNodeInSearch)
        {
            search.choseTargetNode = false;
            search.calculatingNodeInSearch = true;
            enemy.StopCoroutine(enemy.chooseTargetNodeCoroutine);
            enemy.chooseTargetNodeCoroutine = enemy.StartCoroutine(enemy.ChooseNextNodeInSmartSearchRoutine(allowedLinks));
        }
    }

    public static IEnumerator ChooseNextNodeInSmartSearchRoutine(this EnemyAI enemy, SmartPathfindingLinkFlags allowedLinks)
    {
        yield return null;

        GameObject? chosenNode = null;
        var chosenNodeDistance = 500f;

        // Collect all node positions into a list to pass to the smart pathfinding task.
        unsearchedNodePositions.Clear();
        foreach (var node in enemy.currentSearch.unsearchedNodes)
            unsearchedNodePositions.Add(node.transform.position);

        // Start a task to calculate paths from the enemy to each node.
        var pathsFromEnemyTask = new SmartPathTask();
        pathsFromEnemyTask.StartPathTask(enemy.agent, enemy.transform.position, unsearchedNodePositions, allowedLinks);

        // Start a task to calculate paths from the search origin to each node if necessary.
        var pathsFromSearchStart = !enemy.currentSearch.startedSearchAtSelf ? new SmartPathTask() : null;
        pathsFromSearchStart?.StartPathTask(enemy.agent, enemy.currentSearch.currentSearchStartPosition, unsearchedNodePositions, allowedLinks);

        for (var i = enemy.currentSearch.unsearchedNodes.Count - 1; i >= 0; i--)
        {
            if (!enemy.IsOwner)
            {
                enemy.currentSearch.calculatingNodeInSearch = false;
                yield break;
            }

            if (i % 5 == 0)
            {
                yield return null;
            }

            while (!pathsFromEnemyTask.IsResultReady(i))
                yield return null;
            if (pathsFromSearchStart != null)
            {
                while (!pathsFromSearchStart.IsResultReady(i))
                    yield return null;
            }

            var nodePosition = enemy.currentSearch.unsearchedNodes[i].transform.position;

            if ((nodePosition - enemy.currentSearch.currentSearchStartPosition).sqrMagnitude > enemy.currentSearch.searchWidth * enemy.currentSearch.searchWidth)
            {
                enemy.EliminateNodeFromSearch(i);
                continue;
            }

            if (enemy.agent.isOnNavMesh && !pathsFromEnemyTask.PathSucceeded(i))
            {
                enemy.EliminateNodeFromSearch(i);
                continue;
            }

            if (enemy.currentSearch.onlySearchNodesInLOS && Physics.Linecast(nodePosition, enemy.currentSearch.currentSearchStartPosition, StartOfRound.Instance.collidersAndRoomMaskAndDefault, QueryTriggerInteraction.Ignore))
            {
                enemy.EliminateNodeFromSearch(i);
                continue;
            }

            var pathDistance = pathsFromSearchStart != null ? pathsFromSearchStart.GetPathLength(i) : pathsFromEnemyTask.GetPathLength(i);

            if (pathDistance < chosenNodeDistance && (!enemy.currentSearch.randomized || chosenNode == null || enemy.searchRoutineRandom.Next(0, 100) < 65))
            {
                chosenNodeDistance = pathDistance;
                chosenNode = enemy.currentSearch.unsearchedNodes[i];

                if (chosenNodeDistance <= 0 && !enemy.currentSearch.randomized)
                    break;
            }
        }

        if (enemy.currentSearch.waitingForTargetNode)
            enemy.currentSearch.currentTargetNode = chosenNode;
        else
            enemy.currentSearch.nextTargetNode = chosenNode;

        enemy.currentSearch.choseTargetNode = true;
        enemy.currentSearch.calculatingNodeInSearch = false;
        enemy.chooseTargetNodeCoroutine = null;
    }
}
