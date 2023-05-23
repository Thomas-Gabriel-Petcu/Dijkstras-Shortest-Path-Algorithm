using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Unity.VisualScripting.Antlr3.Runtime.Tree;
using UnityEngine;

public class Pathfinding : MonoBehaviour
{
    public Transform startTransform;
    public Transform endTransform;
    public float cellSize = 1;

    // Start is called before the first frame update
    void Start()
    {
        //FindShortestPath(startTransform, endTransform);
        Invoke("Delay",2f);
    }
    private void Delay()
    {
        StartCoroutine(FindShortestPathDelayed(startTransform, endTransform, 0.25f));

    }

    // Update is called once per frame
    void Update()
    {

    }

    public void FindShortestPath(Transform start, Transform goal)
    {
        Vector3[,] waypoints = null;
        Dictionary<Vector3, float> distances = new Dictionary<Vector3, float>();
        Dictionary<Vector3, Vector3> previous = new Dictionary<Vector3, Vector3>();
        List<Vector3> unvisitedWaypoints = new List<Vector3>();
        Vector3 currentWaypoint = Vector3.zero;
        List<Vector3> shortestPath = new List<Vector3>();
        List<GameObject> cubes = new List<GameObject>();//remove later
        int moveCost = 1;

        Vector3 size = new Vector3(Mathf.Abs(goal.position.x - start.position.x), 0f, Mathf.Abs(goal.position.z - start.position.z));
        int numWidthCells = Mathf.CeilToInt(size.x);
        int numLengthCells = Mathf.CeilToInt(size.z);
        waypoints = new Vector3[numWidthCells, numLengthCells];
        Vector3 startPoint = new Vector3(Mathf.Min(start.position.x, goal.position.x), start.position.y, Mathf.Min(start.position.z, goal.position.z));
        Vector3 currentPos = startPoint;

        for (int row = 0; row < numWidthCells; row++)
        {
            for (int col = 0; col < numLengthCells; col++)
            {
                Vector3 v = new Vector3(currentPos.x + row + cellSize / 2, currentPos.y, currentPos.z + col + cellSize / 2);
                waypoints[row, col] = v;
                //remove cubes related code later
                //only used for visualization
                GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                go.transform.localScale = new Vector3(0.25f, 0.25f, 0.25f);
                Instantiate(go, v, Quaternion.identity);
                cubes.Add(go);
            }
        }
        //Initialize waypoint distances as 0 for start and infinite for others
        //set them as unvisited
        foreach (Vector3 waypoint in waypoints)
        {
            if (distances.Count == 0)
            {
                distances.Add(waypoint, 0);
                //Debug.Log("Added start waypoint with distance 0");
                continue;
            }
            distances.Add(waypoint, Mathf.Infinity);
            //Debug.Log("Added waypoint with distance infinite");
            unvisitedWaypoints.Add(waypoint);
        }
        currentWaypoint = distances.Aggregate((l, r) => l.Value < r.Value ? l : r).Key;
        shortestPath.Add(currentWaypoint);

        while (unvisitedWaypoints.Count > 0)
        {
            //determining unvisited neighbours
            List<Vector3> unvisitedNeighbours = new List<Vector3>();
            foreach (Vector3 neighbour in GetNeighbours(waypoints, currentWaypoint))
            {
                //if (!unvisitedWaypoints.Contains(neighbour))
                //    continue;
                unvisitedNeighbours.Add(neighbour);
            }

            foreach (Vector3 neighbour in unvisitedNeighbours)
            {
                List<Vector3> neighboursOfNeighbour = new List<Vector3>();
                neighboursOfNeighbour = GetNeighbours(waypoints, neighbour);
                float dist = Mathf.Infinity;
                foreach (Vector3 l_neighbour in neighboursOfNeighbour)
                {
                    if (distances[l_neighbour] != Mathf.Infinity && (distances[l_neighbour] < dist))
                    {
                        dist = distances[l_neighbour];
                        previous[neighbour] = l_neighbour;
                        distances[neighbour] = distances[previous[neighbour]] + moveCost;
                    }
                    //found the smallest value
                }
                //found neighbour of neighbour which has a set distance
                //and has the smallest set distance

            }
            Vector3 nextWaypoint = GetWaypointOfLowestDistance(unvisitedWaypoints, distances);
            unvisitedWaypoints.Remove(currentWaypoint);
            currentWaypoint = nextWaypoint;
            shortestPath.Add(currentWaypoint);
            float estimateDistance = Vector3.Distance(currentWaypoint, goal.position);
            if (estimateDistance < moveCost)
            {
                Debug.Log("Ended search");
                goto stopSearch;

            }
            #region Original pathing
            ////determine distance to neighbour
            //foreach (Vector3 neighbour in unvisitedNeighbours)
            //{
            //    float newDistance = moveCost + distances[currentWaypoint];
            //    if (newDistance < distances[neighbour])
            //    {
            //        distances[neighbour] = newDistance; //set distance
            //        previous[neighbour] = currentWaypoint; //update previous
            //    }
            //}
            //Vector3 nextWaypoint = GetWaypointOfLowestDistance(unvisitedNeighbours, distances);

            //currentWaypoint = nextWaypoint;
            //unvisitedWaypoints.Remove(currentWaypoint);
            //shortestPath.Add(currentWaypoint);
            //float estimateDistance = Vector3.Distance(currentWaypoint, goal.position);
            //if (estimateDistance < moveCost)
            //{
            //    Debug.Log("Ended search");
            //    goto stopSearch;

            //}
            #endregion
        }
    stopSearch:

        foreach (var item in shortestPath)
        {
            Debug.Log($"Waypoint {shortestPath.IndexOf(item) + 1} is {item} with distance {distances[item]}");
            Collider[] co = Physics.OverlapSphere(item, 0.25f);
            co[0].GetComponent<MeshRenderer>().material.color = Color.blue;
        }
    }

    private int[] FindIndicesIn2DArray(Vector3[,] array, Vector3 t)
    {
        
        for (int row = 0; row < array.GetLength(0); row++)
        {
            for (int col = 0; col < array.GetLength(1); col++)
            {
                if (t == array[row, col])
                {
                    int[] arr = new int[2];
                    arr[1] = col;
                    arr[0] = row;
                    return arr;
                }
            }
        }
        return null;
    }
    private Vector3 GetWaypointOfLowestDistance(List<Vector3> waypoints, Dictionary<Vector3, float> dict)
    {
        float dist = Mathf.Infinity;
        Vector3 waypoint = Vector3.zero;
        foreach (KeyValuePair<Vector3, float> item in dict)
        {
            if (waypoints.Contains(item.Key) && item.Value < dist)
            {
                dist = item.Value;
                waypoint = item.Key;
            }
        }
        return waypoint;
    }
    private List<Vector3> GetNeighbours(Vector3[,] array, Vector3 v)
    {
        List<Vector3> neighbours = new List<Vector3>();
        var col = FindIndicesIn2DArray(array, v)[0];
        var row = FindIndicesIn2DArray(array, v)[1];
        //Debug.Log($"array with index 0 {index[0]} and index 1 {index[1]}");
        if (row < array.GetLength(0) - 1)//grab top neighbour
        {
            neighbours.Add(array[row + 1, col]);
            //Debug.Log($"top neighbour is {array[row + 1, col]}");
        }
        if (col < (array.GetLength(1) - 1))//grab right neighbour
        {
            neighbours.Add(array[row, col + 1]);
        }
        if (row > 0) //grab bottom Neighbour
        {
            neighbours.Add(array[row - 1, col]);
        }
        if (col > 0) //grab left neightbour
        {
            neighbours.Add(array[col - 1, row]);
        }
        return neighbours;
    }
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.black;
        Vector3 size = new Vector3(Mathf.Abs(endTransform.position.x - startTransform.position.x), 0f, Mathf.Abs(endTransform.position.z - startTransform.position.z));
        int numHorizontalCells = Mathf.CeilToInt(size.x);
        int numVerticalCells = Mathf.CeilToInt(size.z);

        // Determine the starting point for drawing the grid lines
        Vector3 start = new Vector3(Mathf.Min(startTransform.position.x, endTransform.position.x), startTransform.position.y, Mathf.Min(startTransform.position.z, endTransform.position.z));

        Vector3 currentPos = start;

        // Draw vertical lines
        for (int i = 0; i <= numHorizontalCells; i++)
        {
            Gizmos.DrawLine(currentPos, currentPos + Vector3.forward * Mathf.CeilToInt(size.z));
            currentPos += Vector3.right * cellSize;
        }

        currentPos = start;

        // Draw horizontal lines
        for (int i = 0; i <= numVerticalCells; i++)
        {
            Gizmos.DrawLine(currentPos, currentPos + Vector3.right * Mathf.CeilToInt(size.x));
            currentPos += Vector3.forward * cellSize;
        }
    }
    public IEnumerator FindShortestPathDelayed(Transform start, Transform goal, float delay)
    {
        Vector3[,] waypoints = null;
        Dictionary<Vector3, float> distances = new Dictionary<Vector3, float>();
        Dictionary<Vector3, Vector3> previous = new Dictionary<Vector3, Vector3>();
        List<Vector3> unvisitedWaypoints = new List<Vector3>();
        Vector3 currentWaypoint = Vector3.zero;
        List<Vector3> shortestPath = new List<Vector3>();
        List<GameObject> cubes = new List<GameObject>();//remove later
        int moveCost = 1;

        Vector3 size = new Vector3(Mathf.Abs(goal.position.x - start.position.x), 0f, Mathf.Abs(goal.position.z - start.position.z));
        int numWidthCells = Mathf.CeilToInt(size.x);
        int numLengthCells = Mathf.CeilToInt(size.z);
        waypoints = new Vector3[numWidthCells, numLengthCells];
        Vector3 startPoint = new Vector3(Mathf.Min(start.position.x, goal.position.x), start.position.y, Mathf.Min(start.position.z, goal.position.z));
        Vector3 currentPos = startPoint;

        for (int row = 0; row < numWidthCells; row++)
        {
            for (int col = 0; col < numLengthCells; col++)
            {
                Vector3 v = new Vector3(currentPos.x + row + cellSize / 2, currentPos.y, currentPos.z + col + cellSize / 2);
                waypoints[row, col] = v;
                //remove cubes related code later
                //only used for visualization
                GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                go.transform.localScale = new Vector3(0.25f, 0.25f, 0.25f);
                Instantiate(go, v, Quaternion.identity);
                cubes.Add(go);
            }
        }

        //Initialize waypoints with distances and marking as unvisited
        //distance 0 for start, infinite for rest
        foreach (Vector3 waypoint in waypoints)
        {
            if (distances.Count == 0)
            {
                distances.Add(waypoint, 0);
                //Debug.Log("Added start waypoint with distance 0");
                continue;
            }
            distances.Add(waypoint, Mathf.Infinity);
            //Debug.Log("Added waypoint with distance infinite");
            unvisitedWaypoints.Add(waypoint);
        }
        currentWaypoint = distances.Aggregate((l, r) => l.Value < r.Value ? l : r).Key;
        shortestPath.Add(currentWaypoint);
        while (unvisitedWaypoints.Count > 0)
        {
            //determining unvisited neighbours
            List<Vector3> unvisitedNeighbours = new List<Vector3>();
            foreach (Vector3 neighbour in GetNeighbours(waypoints, currentWaypoint))
            {
                if (!unvisitedWaypoints.Contains(neighbour))
                    continue;
                unvisitedNeighbours.Add(neighbour);
            }

            foreach (Vector3 neighbour in unvisitedNeighbours)
            {
                List<Vector3> neighboursOfNeighbour = new List<Vector3>();
                neighboursOfNeighbour = GetNeighbours(waypoints, neighbour);
                float dist = Mathf.Infinity;
                foreach (Vector3 l_neighbour in neighboursOfNeighbour)
                {
                    if (distances[l_neighbour] != Mathf.Infinity && (distances[l_neighbour] < dist))
                    {
                        dist = distances[l_neighbour];
                        previous[neighbour] = l_neighbour;
                        Debug.Log($"previous of {neighbour} is {l_neighbour}");
                        distances[neighbour] = distances[previous[neighbour]] + moveCost;
                    }
                    //found the smallest value
                }
                //found neighbour of neighbour which has a set distance
                //and has the smallest set distance

            }
            Vector3 nextWaypoint = GetWaypointOfLowestDistance(unvisitedWaypoints, distances);
            unvisitedWaypoints.Remove(currentWaypoint);
            currentWaypoint = nextWaypoint;
            if (!shortestPath.Contains(previous[currentWaypoint]))
            {
                shortestPath.Add(previous[currentWaypoint]);
            }
            float estimateDistance = Vector3.Distance(currentWaypoint, goal.position);
            //yield return new WaitForSeconds(1f);
            if (estimateDistance < moveCost)
            {
                Debug.Log("Ended search");
                goto stopSearch;

            }
            
        }
    stopSearch:;
        foreach (var item in shortestPath)
        {
            yield return new WaitForSeconds(1);
            Debug.Log($"Waypoint {shortestPath.IndexOf(item) + 1} is {item} with distance {distances[item]}");
            Collider[] co = Physics.OverlapSphere(item, 0.25f);
            co[0].GetComponent<MeshRenderer>().material.color = Color.blue;
        }
    }
}