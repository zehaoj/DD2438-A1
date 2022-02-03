﻿using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;
using Random = System.Random;

namespace Scrips
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;

        public int next_waypoint_idx;

        public float max_speed;

        public float goal_distance;
        public float lookahead_dist;
        public float start_time;
        public int speedup_stratagy;
        public Vector3 goal_pos;
        Rigidbody my_rigidbody;

        public float k_p = 0.5f;
        public float k_d = 0.5f;

        public bool start_turn = false;

        TerrainManager terrain_manager;

        List<Vector3> my_path = new List<Vector3>();

        List<float> path_curvature = new List<float>();

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            bool point_injection = false;
            bool point_smoothing = true;

            // Plan your path here
            // Replace the code below that makes a random path
            // ...

            my_rigidbody = GetComponent<Rigidbody>();

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            float dist_begin = Mathf.Sqrt(Mathf.Pow(start_pos[0] - goal_pos[0],2) + Mathf.Pow(start_pos[2] - goal_pos[2], 2));
            Debug.Log("dis_begin" + dist_begin);
            if (dist_begin > 100)
                speedup_stratagy = 2;
            else
                speedup_stratagy = 1;

            // RRT function
            //List<Vector3> ori_my_path = Rrt(start_pos, goal_pos);
            //ori_my_path.Reverse();
            var watch = new System.Diagnostics.Stopwatch();
            watch.Start();
            List<Vector3> ori_my_path = BidirectionalRRT(start_pos, goal_pos);
            watch.Stop();
            Debug.Log($"Execution Time BidirectionalRRT: {watch.ElapsedMilliseconds} ms");

            // Inject waypoints between
            List<Vector3> injected_path = new List<Vector3>();

            if (point_injection)
            {
                int spacing = 5;
                for (int cur_idx = 0; cur_idx < ori_my_path.Count - 1; cur_idx++)
                {
                    Vector3 point_to = ori_my_path[cur_idx + 1] - ori_my_path[cur_idx];
                    Vector3 unit_vector = point_to.normalized;
                    int num_points_that_fit = (int) Math.Ceiling(point_to.magnitude / spacing);
                    Vector3 single_vector = unit_vector * spacing;
                    for (int i = 0; i < num_points_that_fit; i++)
                    {
                        injected_path.Add(ori_my_path[cur_idx] + single_vector * i);
                    }
                }
            } else {
                injected_path = ori_my_path;
            }
            


            // make the waypoints smooth
            // Vector3 old_wp;
            // List<Vector3> my_path2 = new List<Vector3>();

            if (point_smoothing)
            {
                Debug.Log("Start smoothing");

                float tolerance = 0.01f;
                float change = tolerance;
                float b = 0.8f;
                float a = 1 - b;

                float[,] smooth_path = new float[injected_path.Count, 2];
                
                for (int i = 0; i < injected_path.Count; i++)
                {
                    smooth_path[i, 0] = injected_path[i][0];
                    smooth_path[i, 1] = injected_path[i][2];
                }

                while (change >= tolerance)
                {
                    change = 0.0f;
                    for (int i = 1; i < injected_path.Count - 1; i++)
                    {
                        float ori_x = injected_path[i][0];
                        float ori_y = injected_path[i][2];
                        float last_time_x = smooth_path[i, 0];
                        float last_time_y = smooth_path[i, 1];

                        float tmp_x = smooth_path[i, 0] + a * (injected_path[i][0] - smooth_path[i, 0]) + b * (smooth_path[i - 1, 0] + smooth_path[i+1, 0] - 2 * smooth_path[i, 0]);
                        float tmp_y = smooth_path[i, 1] + a * (injected_path[i][2] - smooth_path[i, 1]) + b * (smooth_path[i - 1, 1] + smooth_path[i+1, 1] - 2 * smooth_path[i, 1]);
                        int tmp_i = terrain_manager.myInfo.get_i_index(tmp_x);
                        int tmp_j = terrain_manager.myInfo.get_j_index(tmp_y);

                        if (!terrain_manager.myInfo.CheckObs(tmp_i, tmp_j)) {
                            smooth_path[i, 0] = tmp_x;
                            smooth_path[i, 1] = tmp_y;
                        }
                        change += Math.Abs(smooth_path[i, 0] + smooth_path[i, 1] - last_time_x - last_time_y);
                    }
                }

                for (int i = 0; i < injected_path.Count; i++)
                {
                    my_path.Add(new Vector3(smooth_path[i, 0], 0, smooth_path[i, 1]));
                }
            } else {
                my_path = injected_path;
            }
            

            // find the curvature of each points
            float max_cur = 0f;

            path_curvature.Add(0f);
            for (int i = 1; i < my_path.Count - 1; i++)
            {
                float radius = findRadius(my_path[i - 1][0], my_path[i - 1][2], my_path[i][0], my_path[i][2], my_path[i + 1][0], my_path[i + 1][2]);
                if (radius < 0.1) 
                {
                    path_curvature.Add(0f);
                }
                else
                {
                    path_curvature.Add(1 / radius);
                    max_cur = Math.Max(max_cur, 1 / radius);
                }
            }
            path_curvature.Add(0f);
            Debug.Log("Max" + max_cur);
            
            next_waypoint_idx = 1;

            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }
            start_time = Time.time;
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
                if (iter > 10000)
                    pathFound = true;
                // Pick a random position, find a waypoint between it and a node and add it to the tree
                Vector3 randomPoint = FindRandomPoint(xLow, xHigh, zLow, zHigh, goal_pos);
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

        public List<Vector3> BidirectionalRRT(Vector3 startPoint, Vector3 goalPoint)
        {
            float xLow = terrain_manager.myInfo.x_low;
            float xHigh = terrain_manager.myInfo.x_high;
            float zLow = terrain_manager.myInfo.z_low;
            float zHigh = terrain_manager.myInfo.z_high;
            bool pathFound = false;

            var forwardTree = new Node<Vector3>(startPoint);
            Node<Vector3> forwardNewParent = forwardTree.Root;
            Node<Vector3> fMeetNode = null;
            bool forwardTreeTraversal = true;
            
            var backwardTree = new Node<Vector3>(goalPoint);
            Node<Vector3> backwardNewParent = backwardTree.Root;
            Node<Vector3> bMeetNode = null;
            bool backwardTreeTraversal = true;
            
            int iter = 0;
            List<Vector3> myPath = new List<Vector3>();
            Node<Vector3> finalNode = null;
            
            while (pathFound == false)
            {
                iter += 1;
                Node<Vector3> fLeefNode = BuildTree(xLow, xHigh, zLow, zHigh, startPoint, goalPoint, forwardTree, forwardNewParent);
                Node<Vector3> bLeefNode = BuildTree(xLow, xHigh, zLow, zHigh, goalPoint, startPoint, backwardTree, backwardNewParent);
                (pathFound, fMeetNode, bMeetNode) = FindMeetingPoint(forwardTree, backwardTree, pathFound, iter, fLeefNode, bLeefNode);
            }
            // We have found a path to the goal, so now we traverse the tree and find the path nodes
            while (forwardTreeTraversal)
            {
                myPath.Add(fMeetNode.Value);
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
                Debug.Log($"goalProb: {goalProb}");
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
            const int stepSize = 4;
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

        // function used to find radius given three points
        private float findRadius(float x1, float y1, float x2, float y2, float x3, float y3)
        {
            float x12 = x1 - x2;
            float x13 = x1 - x3;
        
            float y12 = y1 - y2;
            float y13 = y1 - y3;
        
            float y31 = y3 - y1;
            float y21 = y2 - y1;
        
            float x31 = x3 - x1;
            float x21 = x2 - x1;
        
            // x1^2 - x3^2
            float sx13 = (float) Math.Pow(x1, 2) - (float) Math.Pow(x3, 2);
        
            // y1^2 - y3^2
            float sy13 = (float) Math.Pow(y1, 2) - (float) Math.Pow(y3, 2);
        
            float sx21 = (float) Math.Pow(x2, 2) - (float) Math.Pow(x1, 2);
            float sy21 = (float) Math.Pow(y2, 2) - (float) Math.Pow(y1, 2);
        
            float f = ((sx13) * (x12)
                    + (sy13) * (x12)
                    + (sx21) * (x13)
                    + (sy21) * (x13))
                    / (2 * ((y31) * (x12) - (y21) * (x13)));
            float g = ((sx13) * (y12)
                    + (sy13) * (y12)
                    + (sx21) * (y13)
                    + (sy21) * (y13))
                    / (2 * ((x31) * (y12) - (x21) * (y13)));
        
            float c = -(float) Math.Pow(x1, 2) - (float) Math.Pow(y1, 2) - 2 * g * x1 - 2 * f * y1;
        
            float h = -g;
            float k = -f;
            float sqr_of_r = h * h + k * k - c;
        
            // r is the radius
            float r = (float) Math.Sqrt(sqr_of_r);
            return r;        
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
                // Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                // Debug.Log("Did Hit");
            }

            // current car position
            Vector3 car_pos = new Vector3(transform.position.x, 0, transform.position.z);

            float min_Dist = 100000f;
            int min_idx = next_waypoint_idx;

            for (int idx = next_waypoint_idx; idx < my_path.Count; idx ++)
            {
                Vector3 cur_lookahead_position = my_path[idx];
                float cur_lookahead_dist = Mathf.Sqrt(Mathf.Pow(car_pos[0] - cur_lookahead_position[0], 2) + Mathf.Pow(car_pos[2] - cur_lookahead_position[2], 2));
                if (cur_lookahead_dist < min_Dist) {
                    min_idx = idx;
                    min_Dist = cur_lookahead_dist;
                }
            }
            // look ahead waypoint position
            next_waypoint_idx = min_idx;
            Vector3 lookahead_position = my_path[next_waypoint_idx];
            Debug.DrawLine(transform.position, my_path[next_waypoint_idx], Color.yellow);

            lookahead_dist = Mathf.Sqrt(Mathf.Pow(car_pos[0] - lookahead_position[0], 2) + Mathf.Pow(car_pos[2] - lookahead_position[2], 2));
            goal_distance = Mathf.Sqrt(Mathf.Pow(car_pos[0] - goal_pos[0],2) + Mathf.Pow(car_pos[2] - goal_pos[2], 2));
        
            // if close to the look ahead waypoint, choose next waypoint to look next
            if ((lookahead_dist < 5) && (next_waypoint_idx < my_path.Count - 1))
            {
                next_waypoint_idx ++;
            }

            

            // a PD-controller to get desired velocity
            Vector3 position_error = lookahead_position - transform.position;


            if (speedup_stratagy == 1) {
                // 1. speed up to fullest when finishing 20% of the whole path
                // suitable for short dist
                float driven_percentage = (float) ((next_waypoint_idx - 1) / (float) my_path.Count);
                max_speed = (float) Math.Min(15f, 15f * (driven_percentage / 0.2));
                max_speed = Math.Min(max_speed, 1f / path_curvature[next_waypoint_idx]);
            } else if (speedup_stratagy == 2) {
                // 2. speed up to fullest within 10 seconds
                // suitable for long dist
                float driven_time_percentage = (float) ((Time.time - start_time) / 10f);
                max_speed = (float) Math.Min(15f, 15f * driven_time_percentage);
                max_speed = Math.Min(max_speed, 1f / path_curvature[next_waypoint_idx]);
            }

            Vector3 velocity_error = my_rigidbody.velocity * (max_speed - my_rigidbody.velocity.magnitude);
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            // this is how you control the car
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration + " Velo:" + my_rigidbody.velocity.magnitude + "max_speed:" + max_speed);
            

            if (Vector3.Angle(transform.forward, lookahead_position - car_pos) > 90F) {
                Debug.Log("reorient");
                start_turn = true;
                int sign = steering > 0 ? 10 : -10;
                m_Car.Move(sign, 1f, 3f, 0f);
            } else if (start_turn) {
                if (Vector3.Angle(transform.forward, lookahead_position - car_pos) < 30F)
                    start_turn = false;
                else {
                    int sign = steering > 0 ? 10 : -10;
                    m_Car.Move(sign, 1f, 3f, 0f);
                }
            } else {
                // start to break when approaching target
                if (goal_distance < 10)
                {
                    Debug.Log("Close enough, stopping");
                    m_Car.Move(steering, 0f, 1f, 1f);
                }
                else
                {
                    int sign = acceleration > 0 ? 1 : -1;
                    m_Car.Move(steering, sign * Math.Min(4f, Math.Abs(acceleration)), acceleration, 0f);
                }
            }
        }
    }
}
