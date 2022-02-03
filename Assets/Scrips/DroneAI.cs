using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
// using Random = UnityEngine.Random;
using Random = System.Random;

namespace Scrips
{
[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{

    private DroneController m_Drone; // the car controller we want to use

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    private void Start()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


        Vector3 start_pos = terrain_manager.myInfo.start_pos;
        Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

        List<Vector3> my_path = new List<Vector3>();

        // Plan your path here
        // RRT function
        var watch = new System.Diagnostics.Stopwatch();
        watch.Start();
        //List<Vector3> ori_my_path = BidirectionalRRT(start_pos, goal_pos);
        List<Vector3> ori_my_path = Rrt(start_pos, goal_pos);
        watch.Stop();
        Debug.Log($"Execution Time RRT: {watch.ElapsedMilliseconds} ms");




        // Plot your path to see if it makes sense
        Vector3 old_wp = start_pos;
        foreach (var wp in my_path)
        {
            Debug.DrawLine(old_wp, wp, Color.red, 100f);
            old_wp = wp;
        }

        
    }


    private void FixedUpdate()
    {
        // Execute your path here
        // ...

        // this is how you access information about the terrain
        int i = terrain_manager.myInfo.get_i_index(transform.position.x);
        int j = terrain_manager.myInfo.get_j_index(transform.position.z);
        float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
        float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

        Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z), Color.white, 1f);

        // this is how you control the car
        m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);

    }

 

    // Update is called once per frame
    void Update()
    {
        
    }
    public List<Vector3> Rrt(Vector3 startPoint, Vector3 goalPoint)
        {
            float xLow = terrain_manager.myInfo.x_low;
            float xHigh = terrain_manager.myInfo.x_high;
            float zLow = terrain_manager.myInfo.z_low;
            float zHigh = terrain_manager.myInfo.z_high;
            bool pathFound = false;
            int iter = 0;
            List<Vector3> myPath = new List<Vector3>();
            Node<Vector3> finalNode = null;

            var forwardTree = new Node<Vector3>(startPoint);
            Node<Vector3> forwardNewParent = forwardTree.Root;
            Node<Vector3> fMeetNode = null;
            bool forwardTreeTraversal = true;
            
            var backwardTree = new Node<Vector3>(goalPoint);
            Node<Vector3> backwardNewParent = backwardTree.Root;
            Node<Vector3> bMeetNode = null;
            bool backwardTreeTraversal = true;

            while (pathFound == false)
            {
                iter += 1;
                if (iter > 10000)
                    pathFound = true;
                Node<Vector3> fLeefNode = BuildTree(xLow, xHigh, zLow, zHigh, startPoint, goalPoint, forwardTree, forwardNewParent);
                Node<Vector3> bLeefNode = BuildTree(xLow, xHigh, zLow, zHigh, goalPoint, startPoint, backwardTree, backwardNewParent);
                (pathFound, fMeetNode, bMeetNode) = FindMeetingPoint(forwardTree, backwardTree, pathFound, iter, fLeefNode, bLeefNode);
            }
            
            // We have found a path to the goal, so now we traverse and combine the trees and find the path nodes
            while (forwardTreeTraversal)
            {
                myPath.Add(fMeetNode.Value);
                //Debug.DrawLine(fMeetNode.Value, fMeetNode.Parent.Value, Color.yellow, 100f);
                fMeetNode = fMeetNode.Parent;
                if (fMeetNode == forwardTree.Root)
                {
                    myPath.Add(forwardTree.Root.Value);
                    myPath.Reverse();
                    forwardTreeTraversal = false;
                    
                }
            }
            while (backwardTreeTraversal)
            {
                myPath.Add(bMeetNode.Value);
                //Debug.DrawLine(bMeetNode.Value, bMeetNode.Parent.Value, Color.green, 100f);
                bMeetNode = bMeetNode.Parent;
                if (bMeetNode == backwardTree.Root)
                {
                    myPath.Add(backwardTree.Root.Value);
                    backwardTreeTraversal = false;
                    
                }
            }

            return myPath;
        }

        public (bool, Node<Vector3>, Node<Vector3>) FindMeetingPoint(Node<Vector3> forwardTree, Node<Vector3> backwardTree, bool pathFound, int iter, Node<Vector3> fLeafNode, Node<Vector3> bLeafNode)
        {
            float distanceThreshold = 4;
            Node<Vector3> bMeetNode;
            Node<Vector3> fMeetNode;
            if (bLeafNode != null)
            {
                foreach (Node<Vector3> fNode in forwardTree.All)
                {
                    float distanceTrees = Vector3.Distance(fNode.Value, bLeafNode.Value);
                    if (distanceTrees < distanceThreshold)
                    {
                        fMeetNode = fNode;
                        bMeetNode = bLeafNode;
                        Debug.Log(String.Format("Found goal in {0} iterations.", iter));
                        pathFound = true;
                        return (pathFound, fMeetNode, bMeetNode);

                    }
                }
            }

            if (fLeafNode != null)
            {
                foreach (Node<Vector3> bNode in backwardTree.All)
                {
                    float distanceTrees = Vector3.Distance(fLeafNode.Value, bNode.Value);
                    if (distanceTrees < distanceThreshold)
                    {
                        fMeetNode = fLeafNode;
                        bMeetNode = bNode;
                        Debug.Log(String.Format("Found goal in {0} iterations.", iter));
                        pathFound = true;
                        return (pathFound, fMeetNode, bMeetNode);

                    }
                }
            }

            return (pathFound, null, null);
        }

        public Node<Vector3> BuildTree(float xLow, float xHigh, float zLow, float zHigh, Vector3 startPoint, Vector3 goalPoint, Node<Vector3> tree, Node<Vector3> newParent)
        {
            Node<Vector3> currNode;
            // Pick a random position, find a waypoint between it and a node and add it to the tree
            Vector3 randomPoint = FindRandomPoint(xLow, xHigh, zLow, zHigh, goalPoint);
            Vector3 parentPoint = startPoint;
            //Debug.DrawLine(start_pos, randomPoint, Color.red, 100f);
                
            float dist = Vector3.Distance(startPoint, randomPoint);
            foreach (Node<Vector3> node in tree.All)
            {
                float newDist = Vector3.Distance(node.Value, randomPoint);
                if (dist > newDist)
                {
                    dist = newDist;
                    newParent = node;
                    parentPoint = newParent.Value;
                }
            }
            Vector3 wayPoint = FindWayPoint(parentPoint, randomPoint);
            //Debug.DrawLine(start_pos, wayPoint, Color.green, 100f);

            bool onObstacle = CheckObstaclePoint(wayPoint);
            if (!onObstacle)
            {
                    
                bool edgeOnObstacle =CheckObstacleEdge(parentPoint, wayPoint);
                if (!edgeOnObstacle)
                {
                    currNode = newParent.Add(wayPoint);
                    foreach (var node in tree.All.Values())
                    {
                        Debug.DrawLine(newParent.Value, wayPoint, Color.blue, 100f);
                        
                    }
                    return currNode;
                }
            }

            return null;
        }

        public Vector3 FindRandomPoint(float xLow, float xHigh, float zLow, float zHigh, Vector3 goalPoint)
        {
            Vector3 randomPoint = new Vector3(0,0,0);
            bool onObstacle = true;
            Random random = new Random();
            
            while (onObstacle)
            {
                int goalProb = random.Next(1, 101);
                if (goalProb < 20)
                {
                    randomPoint = goalPoint;
                }
                else
                {
                    randomPoint = new Vector3(UnityEngine.Random.Range(xLow, xHigh), 0,
                        UnityEngine.Random.Range(zLow, zHigh));
                }
                onObstacle = CheckObstaclePoint(randomPoint);
            }
            return randomPoint;
        }

        public Vector3 FindWayPoint(Vector3 start_pos, Vector3 endPoint)
        { 
            const int stepSize = 10;
            (float xDistWayPoint, float zDistWayPoint) = GetCoordDistanceBetweenPoints(start_pos, endPoint);
            float distWayPoint = Vector3.Distance(start_pos, endPoint);
            float wayPointX = start_pos.x+(stepSize * xDistWayPoint / distWayPoint);
            float wayPointZ = start_pos.z+(stepSize * zDistWayPoint / distWayPoint);
            Vector3 wayPoint = new Vector3(wayPointX, 0,wayPointZ);
            return wayPoint;
        }
        public (float, float) GetCoordDistanceBetweenPoints(Vector3 pointA, Vector3 pointB)
        {
            float xDistance = pointB.x - pointA.x;
            float zDistance = pointB.z - pointA.z;
            return (xDistance, zDistance);
        }
        public bool CheckObstaclePoint(Vector3 point)
        {
            int i = terrain_manager.myInfo.get_i_index(point.x);
            int j = terrain_manager.myInfo.get_j_index(point.z);
            // float obstacle = terrain_manager.myInfo.traversability[i, j];// 1.0 if there is an obstacle on point and otherwise 0.0 
            bool obstacle = terrain_manager.myInfo.CheckObs(i, j);
            if (!obstacle)
            {
                return false;
            }
            return true;
        }

        public bool CheckObstacleEdge(Vector3 startPoint, Vector3 endPoint)
        {
            float dist = Vector3.Distance(startPoint, endPoint);
            for (float d = 0 ; d <= 1 ; d += 1f / dist)
            {
                Vector3 edgePoint = Vector3.Lerp(startPoint, endPoint, d);
                int i = terrain_manager.myInfo.get_i_index(edgePoint.x);
                int j = terrain_manager.myInfo.get_j_index(edgePoint.z);
                // float obstacle = terrain_manager.myInfo.expanded_traversability[i, j];// 1.0 if there is an obstacle on point and otherwise 0.0 
                bool obstacle = terrain_manager.myInfo.CheckObs(i, j);

                // if (obstacle == 1.0)
                if (obstacle)
                {
                    return true;
                }
            }
            
            return false;
        }
}
}