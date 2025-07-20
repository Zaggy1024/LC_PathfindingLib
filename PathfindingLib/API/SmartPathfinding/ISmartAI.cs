namespace PathfindingLib.API.SmartPathfinding;

#nullable enable

public interface ISmartAI
{
    public void GoToSmartPathDestination(in SmartPathDestination destination);
}
