using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Unity.VisualScripting.Antlr3.Runtime.Tree;
using UnityEditor;
using UnityEngine;
using static UnityEditor.Progress;

public class Pathfinding : MonoBehaviour
{
    public Transform startTransform;
    public Transform endTransform;
    public float cellSize = 1;
    public float allowedIterations = 100; //Infinite while prevention.

    // Start is called before the first frame update
    void Start()
    {
        FindShortestPath(startTransform, endTransform);
    }

    // Update is called once per frame
    void Update()
    {

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
            if (waypoints.Contains(item.Key))
            {
                //Debug.Log($"{item.Key} is in dictionary " + dict);
                if (item.Value < dist)
                {
                dist = item.Value;
                waypoint = item.Key;
                }
            }
        }
        //Debug.Log(waypoint);
        return waypoint;
    }
    private List<Vector3> GetNeighbours(Vector3[,] array, Vector3 v)
    {
        List<Vector3> neighbours = new List<Vector3>();
        var x = FindIndicesIn2DArray(array, v)[0];
        var y = FindIndicesIn2DArray(array, v)[1];
        //Debug.Log(x + ", "+ y);
        if (y < array.GetLength(1) - 1)//grab top neighbour
        {
            neighbours.Add(array[x, y + 1]);
        }
        if (x < (array.GetLength(0) - 1))//grab right neighbour
        {
            neighbours.Add(array[x + 1, y]);
        }
        if (y > 0) //grab bottom Neighbour
        {
            neighbours.Add(array[x, y - 1]);
        }
        if (x > 0) //grab left neightbour
        {
            neighbours.Add(array[x - 1, y]);
        }
        return neighbours;
    }
    private List<Vector3> GetNodesOfDistance(List<Vector3> nodes,Dictionary<Vector3,float> dict, float dist)
    {
        List<Vector3> nodesOfDist = new List<Vector3>();
        foreach (Vector3 node in nodes)
        {
            if (dict[node] == dist)
            {
                nodesOfDist.Add(node);
            }
        }
        return nodesOfDist;
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
    
    public List<Vector3> FindShortestPath(Transform start, Transform goal)
    {
        Vector3[,] nodes = null;
        Dictionary<Vector3, float> distances = new Dictionary<Vector3, float>();
        Dictionary<Vector3, Vector3> previous = new Dictionary<Vector3, Vector3>();
        List<Vector3> unvisitedNodes = new List<Vector3>();
        Vector3 currentNode = Vector3.zero;
        List<Vector3> shortestPath = new List<Vector3>();
        List<GameObject> cubes = new List<GameObject>();//remove later
        int moveCost = 1;
        int iterationNr = 0;

        Vector3 size = new Vector3(Mathf.Abs(goal.position.x - start.position.x), 0f, Mathf.Abs(goal.position.z - start.position.z));
        int numWidthCells = Mathf.CeilToInt(size.x);
        int numLengthCells = Mathf.CeilToInt(size.z);
        nodes = new Vector3[numWidthCells, numLengthCells];
        Vector3 startPoint = new Vector3(Mathf.Min(start.position.x, goal.position.x), start.position.y, Mathf.Min(start.position.z, goal.position.z));
        Vector3 currentPos = startPoint;
        List<Vector3> openNodes = new List<Vector3>();
        List<Vector3> interestedNodes = new List<Vector3>();
        List<Vector3> closedNodes = new List<Vector3>();

        for (int row = 0; row < numWidthCells; row++)
        {
            for (int col = 0; col < numLengthCells; col++)
            {
                Vector3 v = new Vector3(currentPos.x + row + cellSize / 2, currentPos.y, currentPos.z + col + cellSize / 2);
                nodes[row, col] = v;
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
        foreach (Vector3 node in nodes)
        {
            if (distances.Count == 0)
            {
                distances.Add(node, 0);
                currentNode = node;
                openNodes.Add(currentNode);
                //Debug.Log("Added start waypoint with distance 0");
                continue;
            }
            distances.Add(node, Mathf.Infinity);
            //Debug.Log("Added waypoint with distance infinite");
            unvisitedNodes.Add(node);
        }
        currentNode = distances.Aggregate((l, r) => l.Value < r.Value ? l : r).Key;
        shortestPath.Add(currentNode);
        
        while (iterationNr < allowedIterations)
        {
            iterationNr++;
            Vector3 lowestDistNode = GetWaypointOfLowestDistance(openNodes, distances);
            if (lowestDistNode == Vector3.zero)
            {
                continue;
            }
            float lowestDist = distances[lowestDistNode];
            //find open nodes of lowest distance
            foreach (Vector3 node in GetNodesOfDistance(openNodes, distances, lowestDist))
            {
                interestedNodes.Add(node);
                openNodes.Remove(node);
            }
            foreach (Vector3 node in interestedNodes)
            {
                currentNode = node;
                float distToGoal = Vector3.Distance(goal.position, currentNode);
                closedNodes.Add(node);
                if (distToGoal <= (moveCost / 2))
                {
                    goto reachedGoal;
                }
                List<Vector3> level1Neighbors = GetNeighbours(nodes, node);
                foreach (Vector3 l1N in level1Neighbors)
                {
                    if (!openNodes.Contains(l1N) && !closedNodes.Contains(l1N))
                    {
                        openNodes.Add(l1N);
                    }
                    List<Vector3> level2Neighbors = GetNeighbours(nodes, l1N);
                    Vector3 lowestDistL2N = GetWaypointOfLowestDistance(level2Neighbors, distances);
                    if (distances[lowestDistL2N] < distances[l1N])
                    {
                        distances[l1N] = distances[lowestDistL2N] + moveCost;
                        previous[l1N] = lowestDistL2N;
                    }
                }
            }

        }
    reachedGoal:
        iterationNr = 0;
        List<Vector3> path = new List<Vector3>();
        
        while (previous.ContainsKey(currentNode))
        {
            path.Add(currentNode);
            currentNode = previous[currentNode];
        }
        path.Add(currentNode);
        foreach (var item in path)
        {
            Collider[] coll = Physics.OverlapSphere(item, 0.25f);
            coll[0].GetComponent<MeshRenderer>().material.color = Color.yellow;
        }
        path.Reverse();
        return path;
    }
}