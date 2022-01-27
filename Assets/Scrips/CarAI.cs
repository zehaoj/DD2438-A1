using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Plan your path here

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;
            
            // RRT function
            List<Vector3> myPath = Rrt(start_pos, goal_pos);
            myPath.Reverse();

            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in myPath)
            {
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }
        }


        private void FixedUpdate()
        {
            // Execute your path here

            // this is how you access information about the terrain from the map
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));

            // this is how you access information about the terrain from a simulated laser range finder
            RaycastHit hit;
            float maxRange = 50f;
            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                Debug.Log("Did Hit");
            }
            
            // this is how you control the car
            m_Car.Move(1f, 1f, 1f, 0f);

        }

        public List<Vector3> Rrt(Vector3 start_pos, Vector3 goal_pos)
        {
            float xLow = terrain_manager.myInfo.x_low;
            float xHigh = terrain_manager.myInfo.x_high;
            float zLow = terrain_manager.myInfo.z_low;
            float zHigh = terrain_manager.myInfo.z_high;
            float distanceGoal;
            const int distanceThreshold = 5;
            bool pathFound = false;
            GameObject car = GameObject.Find("Car");
            var tree = new Node<Vector3>(start_pos);
            Node<Vector3> newParent = tree.Root;
            int iter = 0;
            bool treeTraversal = true;
            List<Vector3> myPath = new List<Vector3>();
            Node<Vector3> finalNode = null;
            Node<Vector3> currNode;
            Vector3 parentPoint = start_pos;
            

            while (pathFound == false)
            {
                iter += 1;
                // Pick a random position, find a waypoint between it and a node and add it to the tree
                Vector3 randomPoint = FindRandomPoint(xLow, xHigh, zLow, zHigh);
                //Debug.DrawLine(start_pos, randomPoint, Color.red, 100f);
                
                float dist = Vector3.Distance(start_pos, randomPoint);
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

                bool onObstacle = CheckObstacleStatus(wayPoint);
                if (!onObstacle)
                {
                    currNode = newParent.Add(wayPoint);
                    foreach (var node in tree.All.Values())
                    {
                        Debug.DrawLine(newParent.Value, wayPoint, Color.blue, 100f);
                        
                    }
                    distanceGoal = Vector3.Distance(wayPoint, goal_pos);
                    if (distanceGoal < distanceThreshold)
                    {
                        Debug.Log(String.Format("Found goal in {0} iterations.",
                            iter));
                        finalNode = currNode;
                        pathFound = true;

                    }
                }
            }
            // We have found a path to the goal, so now we traverse the tree and find the path nodes
            while (treeTraversal)
            {
                myPath.Add(finalNode.Value);
                finalNode = finalNode.Parent;
                if (finalNode == tree.Root)
                {
                    myPath.Add(tree.Root.Value);
                    treeTraversal = false;
                }
            }

            return myPath;
        }

        public Vector3 FindRandomPoint(float xLow, float xHigh, float zLow, float zHigh)
        {
            Vector3 randomPoint = new Vector3(0,0,0);
            bool onObstacle = true;
            
            while (onObstacle)
            {
                randomPoint = new Vector3(UnityEngine.Random.Range(xLow, xHigh), 0,
                    UnityEngine.Random.Range(zLow, zHigh));
                onObstacle = CheckObstacleStatus(randomPoint);
            }
            return randomPoint;
        }

        public Vector3 FindWayPoint(Vector3 start_pos, Vector3 endPoint)
        { 
            const int stepSize = 5;
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
        public bool CheckObstacleStatus(Vector3 point)
        {
            int i = terrain_manager.myInfo.get_i_index(point.x);
            int j = terrain_manager.myInfo.get_j_index(point.z);
            float obstacle = terrain_manager.myInfo.traversability[i, j];// 1.0 if there is an obstacle on point and otherwise 0.0 
            if (obstacle == 0)
            {
                return false;
            }
            return true;
        }
    }
}
