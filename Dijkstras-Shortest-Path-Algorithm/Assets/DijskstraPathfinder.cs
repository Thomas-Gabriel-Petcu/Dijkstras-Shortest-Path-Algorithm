using System.Collections.Generic;
using UnityEngine;

public class DijkstraPathfinder : MonoBehaviour
{
    public List<Transform> waypoints;
    private Dictionary<Transform, float> distances;
    private Dictionary<Transform, Transform> previous;
    public Transform startingPosition;
    public Transform targetPosition;

    void Start()
    {
        FindShortestPath(startingPosition, targetPosition);
    }

    public void FindShortestPath(Transform start, Transform end)
    {
        distances = new Dictionary<Transform, float>();
        previous = new Dictionary<Transform, Transform>();
        List<Transform> unvisitedNodes = new List<Transform>();

        foreach (Transform waypoint in waypoints)
        {
            distances[waypoint] = Mathf.Infinity;
            previous[waypoint] = null;
            unvisitedNodes.Add(waypoint);
        }

        distances[start] = 0;

        while (unvisitedNodes.Count > 0)
        {
            Transform currentNode = null;
            float shortestDistance = Mathf.Infinity;

            foreach (Transform waypoint in unvisitedNodes)
            {
                if (distances[waypoint] < shortestDistance)
                {
                    shortestDistance = distances[waypoint];
                    currentNode = waypoint;
                }
            }

            if (currentNode == end)
            {
                break;
            }

            unvisitedNodes.Remove(currentNode);

            foreach (Transform neighbor in currentNode.GetComponent<Waypoint>().neighbors)
            {
                float distance = Vector3.Distance(currentNode.position, neighbor.position);

                float altDistance = distances[currentNode] + distance;

                if (altDistance < distances[neighbor])
                {
                    distances[neighbor] = altDistance;
                    previous[neighbor] = currentNode;
                }
            }
        }

        List<Transform> path = new List<Transform>();

        while (previous[end] != null)
        {
            path.Add(end);
            end = previous[end];
        }

        path.Reverse();

        foreach (Transform waypoint in path)
        {
            Debug.Log(waypoint.name);
        }
    }
}