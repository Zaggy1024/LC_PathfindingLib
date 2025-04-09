using System.Collections;

using UnityEngine;

namespace PathfindingLib.API.SmartPathfinding;

public static class SmartRoaming
{
    public static bool useVanilla = false;

    public static void StartSmartSearch(this EnemyAI enemy, Vector3 startOfSearch, AISearchRoutine newSearch = null)
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
            enemy.searchCoroutine = enemy.StartCoroutine(enemy.CurrentSmartSearchCoroutine());
        enemy.currentSearch.inProgress = true;
    }

    public static IEnumerator CurrentSmartSearchCoroutine(this EnemyAI enemy)
    {
        yield return null;

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
                enemy.StartCalculatingNextTargetNode();

                yield return new WaitUntil(() => enemy.currentSearch.choseTargetNode);
            }

            enemy.currentSearch.waitingForTargetNode = false;
            if (enemy.currentSearch.unsearchedNodes.Count == 0 || enemy.currentSearch.currentTargetNode == null)
                continue;

            // Go to the selected node.
            enemy.currentSearch.unsearchedNodes.Remove(enemy.currentSearch.currentTargetNode);
            enemy.SetDestinationToPosition(enemy.currentSearch.currentTargetNode.transform.position);

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
            enemy.StartCalculatingNextTargetNode();

            // Wait for the current target to be reached.
            currentTargetPos = enemy.currentSearch.currentTargetNode.transform.position;
            searchPrecisionSqr = enemy.currentSearch.searchPrecision * enemy.currentSearch.searchPrecision;
            var pathingEndTime = Time.time + 16f;
            while (enemy.searchCoroutine != null && Time.time < pathingEndTime)
            {
                if (enemy.currentSearch.onlySearchNodesInLOS && Physics.Linecast(currentTargetPos, enemy.currentSearch.currentSearchStartPosition, StartOfRound.Instance.collidersAndRoomMaskAndDefault, QueryTriggerInteraction.Ignore))
                    break;

                yield return new WaitForSeconds(0.5f);

                if ((currentTargetPos - enemy.transform.position).sqrMagnitude < searchPrecisionSqr)
                {
                    enemy.ReachedNodeInSearch();
                    break;
                }
            }
        }

        if (!enemy.IsOwner)
            enemy.StopSearch(enemy.currentSearch);
    }
}
