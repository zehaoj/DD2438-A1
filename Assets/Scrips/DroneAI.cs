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

        public int next_waypoint_idx;

        public float max_speed;

        public float max_speed_last_time = 0f;

        public float real_speed_last_time = 0f;

        public float goal_distance;

        public float lookahead_dist;

        public float start_time;

        public int speedup_stratagy;

        public float after_hard_turn_time;

        public bool just_hard_turn = false;

        public bool just_hit = false;

        public float hit_time = 0;

        public float finished_hit_time = 0;

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
            // get the drone controller
            m_Drone = GetComponent<DroneController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            bool point_injection = true;
            bool point_smoothing = true;
            bool point_move_away = true;

            // Plan your path here
            List<Vector3> my_path_tmp = new List<Vector3>();
            my_rigidbody = GetComponent<Rigidbody>();

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            float dist_begin = Mathf.Sqrt(Mathf.Pow(start_pos[0] - goal_pos[0],2) + Mathf.Pow(start_pos[2] - goal_pos[2], 2));
            Debug.Log("dis_begin" + dist_begin);
            if (dist_begin > 100)
                speedup_stratagy = 2;
            else
                speedup_stratagy = 1;

            // RRT* inspo version
            // var watch = new System.Diagnostics.Stopwatch();
            // watch.Start();
            // List<Vector3> ori_my_path = RrtStar(start_pos, goal_pos);//Rrt(start_pos, goal_pos);
            // watch.Stop();
            // Debug.Log($"Execution Time RRT*: {watch.ElapsedMilliseconds} ms");

            // regular RRT
            var watch2 = new System.Diagnostics.Stopwatch();
            watch2.Start();
            List<Vector3> ori_my_path = Rrt(start_pos, goal_pos);//Rrt(start_pos, goal_pos);
            watch2.Stop();
            Debug.Log($"Execution Time RRT: {watch2.ElapsedMilliseconds} ms");


            // Inject waypoints between
            List<Vector3> injected_path = new List<Vector3>();

            if (point_injection)
            {
                injected_path = PathInjection(ori_my_path, 2f);
            } else {
                injected_path = ori_my_path;
            }

            // DrawPath(injected_path, 1);

            // make the waypoints away from walls
            if (point_move_away)
            {
                injected_path = PathMoveAway(injected_path);
            }
            
            // DrawPath(injected_path, 3);

            // make the waypoints smooth
            if (point_smoothing)
            {
                my_path_tmp = PathSmoothing(injected_path);
            } else {
                my_path_tmp = injected_path;
            }

            // DrawPath(my_path, 1);
            
            if (point_move_away)
            {
                my_path_tmp = PathMoveAway(my_path_tmp);
            }

            // DrawPath(my_path_tmp, 2);
        
            
            my_path_tmp = PathSmoothing(my_path_tmp);
            // my_path = PathMoveAway(my_path);


            // my_path = injected_path;

            // my_path = PathFinalFineMoveAway(my_path);
            // DrawPath(my_path, 2);
            // my_path = PathSmoothing(my_path, 0.8f);



            for (int i = 0; i < my_path_tmp.Count; i++) {
                int m = terrain_manager.myInfo.get_i_index(my_path_tmp[i][0]);
                int n = terrain_manager.myInfo.get_j_index(my_path_tmp[i][2]);
                if (!terrain_manager.myInfo.CheckObs(m, n)) {
                    // Debug.Log("aeoijgoeajgoia");
                    my_path.Add(my_path_tmp[i]);
                }
            }
            DrawPath(my_path, 1);


            // my_path = PathInjection(my_path, 0.5f);
            
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

            start_time = Time.time;      
        }


        public void DrawPath(List<Vector3> draw_path, int coler) {
            Vector3 old_wp = terrain_manager.myInfo.start_pos;
            foreach (var wp in draw_path)
            {
                if (coler == 1)
                    Debug.DrawLine(old_wp, wp, Color.yellow, 100f);
                else if (coler == 2)
                    Debug.DrawLine(old_wp, wp, Color.blue, 100f);
                else if (coler == 3)
                    Debug.DrawLine(old_wp, wp, Color.green, 100f);
                old_wp = wp;
            }
        }


        public List<Vector3> PathInjection(List<Vector3> ori_my_path, float spacing) {
            List<Vector3> injected_path = new List<Vector3>();
            // int spacing = 1;
            for (int cur_idx = 0; cur_idx < ori_my_path.Count - 1; cur_idx++)
            {
                Vector3 point_to = ori_my_path[cur_idx + 1] - ori_my_path[cur_idx];
                Vector3 unit_vector = point_to.normalized;
                int num_points_that_fit = (int) Math.Ceiling(point_to.magnitude / spacing);
                Vector3 single_vector = unit_vector * spacing;
                for (int i = 0; i < num_points_that_fit; i++)
                {
                    // if (i == 0)
                        // continue;
                    injected_path.Add(ori_my_path[cur_idx] + single_vector * i);
                }
            }
            return injected_path;
        }

        public List<Vector3> PathMoveAway(List<Vector3> injected_path) {
            Debug.Log("Start moving points away");
            for (int i = 1; i < injected_path.Count; i++)
            {
                int m = terrain_manager.myInfo.get_i_index(injected_path[i][0]);
                int n = terrain_manager.myInfo.get_j_index(injected_path[i][2]);
                float move_z = terrain_manager.myInfo.CheckObsUpandDown(m, n);
                n = terrain_manager.myInfo.get_j_index(injected_path[i][2] + move_z);
                float move_x = terrain_manager.myInfo.CheckObsLeftandRight(m, n);
                float old_x = injected_path[i][0];
                float old_z = injected_path[i][2];

                injected_path[i] = new Vector3(old_x + move_x, 0f, old_z + move_z);
            }
            return injected_path;
        }


        public List<Vector3> PathFinalFineMoveAway(List<Vector3> injected_path) {
            Debug.Log("Start moving points away last time");
            for (int i = 1; i < injected_path.Count; i++)
            {
                int m = terrain_manager.myInfo.get_i_index(injected_path[i][0]);
                int n = terrain_manager.myInfo.get_j_index(injected_path[i][2]);
                // float move_z = terrain_manager.myInfo.CheckObsUpandDown(m, n);
                // n = terrain_manager.myInfo.get_j_index(injected_path[i][2] + move_z);
                float move_x1 = terrain_manager.myInfo.CheckObsDiagonalUp(m, n);

                m = terrain_manager.myInfo.get_j_index(injected_path[i][0] + move_x1);
                n = terrain_manager.myInfo.get_j_index(injected_path[i][2] + move_x1);
                
                float move_x2 = terrain_manager.myInfo.CheckObsDiagonalDown(m, n);

                float old_x = injected_path[i][0];
                float old_z = injected_path[i][2];

                // int sign_x = move_x > 0 ? 3 : -3;
                // int sign_z = move_z > 0 ? 3 : -3;

                float last_x = injected_path[i - 1][0];
                float last_z = injected_path[i - 1][2];
                
                // if (Math.Abs((old_x + move_x) - last_x) > 5f)
                    // injected_path[i] = new Vector3(old_x + move_x, 0f, old_z + move_z);
                injected_path[i] = new Vector3(old_x + move_x1 + move_x2, 0f, old_z + move_x1 - move_x2);
            }
            return injected_path;
        }

        public List<Vector3> PathSmoothing(List<Vector3> injected_path, float b = 0.9f)
        {
            Debug.Log("Start smoothing");

            float tolerance = 0.01f;
            float change = tolerance;
            // float b = 0.8f;
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

                    if (!terrain_manager.myInfo.CheckObsAround(tmp_i, tmp_j)) {
                        smooth_path[i, 0] = tmp_x;
                        smooth_path[i, 1] = tmp_y;
                    }
                    change += Math.Abs(smooth_path[i, 0] + smooth_path[i, 1] - last_time_x - last_time_y);
                }
            }

            for (int i = 0; i < injected_path.Count; i++)
            {
                injected_path[i] = new Vector3(smooth_path[i, 0], 0, smooth_path[i, 1]);
            }
            return injected_path;
        }

        public List<Vector3> Rrt(Vector3 startPoint, Vector3 goalPoint)
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
            int stepSize = 8;
            
            while (pathFound == false)
            {
                iter += 1;
                if (iter > 30000) {
                    pathFound = true;
                    Debug.Log("Quit early");
                }
                Node<Vector3> fLeefNode = BuildTree(xLow, xHigh, zLow, zHigh, startPoint, goalPoint, forwardTree, forwardNewParent, stepSize);
                Node<Vector3> bLeefNode = BuildTree(xLow, xHigh, zLow, zHigh, goalPoint, startPoint, backwardTree, backwardNewParent, stepSize);
                (pathFound, fMeetNode, bMeetNode) = FindMeetingPoint(forwardTree, backwardTree, pathFound, iter, fLeefNode, bLeefNode);
            }
            // We have found a path to the goal, so now we traverse and combine the trees and find the path nodes
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

        public List<Vector3> RrtStar(Vector3 startPoint, Vector3 goalPoint)
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
            int iterLim = 10000;
            Node<Vector3> currNode;
            int stepSize = 8;


            while (pathFound == false)
            {
                iter += 1;
                if (iter > 30000) {
                    pathFound = true;
                    Debug.Log("Quit early");
                }
                Node<Vector3> fLeefNode = BuildTreeStar(xLow, xHigh, zLow, zHigh, startPoint, goalPoint, forwardTree, forwardNewParent, stepSize);
                Node<Vector3> bLeefNode = BuildTreeStar(xLow, xHigh, zLow, zHigh, goalPoint, startPoint, backwardTree, backwardNewParent, stepSize);
                (pathFound, fMeetNode, bMeetNode) = FindMeetingPoint(forwardTree, backwardTree, pathFound, iter, fLeefNode, bLeefNode);

            }
            // We have found a path to the goal, so now we traverse and combine the trees and find the path nodes
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

        public Node<Vector3> BuildTreeStar(float xLow, float xHigh, float zLow, float zHigh, Vector3 startPoint,
            Vector3 goalPoint, Node<Vector3> tree, Node<Vector3> newParent, int stepSize=10)
        {
            Node<Vector3> currNode = null;
            // Pick a random position, find a waypoint between it and a node and add it to the tree
                Vector3 randomPoint = FindRandomPoint(xLow, xHigh, zLow, zHigh, goalPoint);
                Vector3 parentPoint = startPoint;

                float dist = Vector3.Distance(startPoint, randomPoint);
                List <Node<Vector3>> neighbors = new List<Node<Vector3>>();
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
                Vector3 wayPoint = FindWayPoint(parentPoint, randomPoint, stepSize);
                bool onObstacle = CheckObstaclePoint(wayPoint);
                if (!onObstacle)
                {
                    foreach (Node<Vector3> node in tree.All)
                    {
                        float newDist = Vector3.Distance(node.Value, wayPoint);
                        if (newDist <= stepSize)
                        {
                            neighbors.Add(node);
                        }
                    }

                    float minCost = newParent.Cost + Vector3.Distance(newParent.Value, wayPoint);
                    foreach (Node<Vector3> node in neighbors)
                    {
                        if ((node.Cost + Vector3.Distance(wayPoint, node.Value)) < minCost)
                        {
                            minCost = node.Cost + Vector3.Distance(wayPoint, node.Value);
                            newParent = node;
                        }
                    }
                    bool edgeOnObstacle =CheckObstacleEdge(parentPoint, wayPoint);
                    if (!edgeOnObstacle)
                    {
                        currNode = newParent.Add(wayPoint);
                        /*foreach (var node in tree.All.Values())
                        { 
                            Debug.DrawLine(newParent.Value, wayPoint, Color.green, 100f);
                        }*/
                    }
                }
                return currNode;
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

        public Node<Vector3> BuildTree(float xLow, float xHigh, float zLow, float zHigh, Vector3 startPoint, Vector3 goalPoint, Node<Vector3> tree, Node<Vector3> newParent, int stepSize)
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
            Vector3 wayPoint = FindWayPoint(parentPoint, randomPoint, stepSize);
            //Debug.DrawLine(start_pos, wayPoint, Color.green, 100f);

            bool onObstacle = CheckObstaclePoint(wayPoint);
            if (!onObstacle)
            {
                    
                bool edgeOnObstacle = CheckObstacleEdge(parentPoint, wayPoint);
                if (!edgeOnObstacle)
                {
                    currNode = newParent.Add(wayPoint);
                    return currNode;
                }
            }

            return null;
        }

        public Vector3 FindRandomPoint(float xLow, float xHigh, float zLow, float zHigh, Vector3 goalPoint)
        {
            Vector3 randomPoint = new Vector3(0,0,0);
            bool onObstacle = true;
            Random random = new Random(1);
            
            while (onObstacle)
            {
                int goalProb = random.Next(1, 101);
                // Debug.Log($"goalProb: {goalProb}");
                if (goalProb < 5)
                {
                    randomPoint = goalPoint;
                }
                else
                {
                    randomPoint = new Vector3(UnityEngine.Random.Range(xLow, xHigh), 0,
                        UnityEngine.Random.Range(zLow, zHigh));
                    // Debug.Log(transform.position.x);
                    // randomPoint = new Vector3(UnityEngine.Random.Range(Math.Max(transform.position.x - 20f, xLow), Math.Min(transform.position.x + 20f, xHigh)), 0f,
                        // UnityEngine.Random.Range(Math.Max((transform.position.z - 20f, zLow), Math.Min(transform.position.z + 20f, zHigh))));
                }
                onObstacle = CheckObstaclePoint(randomPoint);
            }
            return randomPoint;
        }

        public Vector3 FindWayPoint(Vector3 start_pos, Vector3 endPoint, int stepSize = 8)
        { 
            // const int stepSize = 8;
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

            // current car position
            Vector3 car_pos = new Vector3(transform.position.x, 0, transform.position.z);

            float min_Dist = 100000f;
            int min_idx = next_waypoint_idx;

            for (int idx = next_waypoint_idx + 1; idx < my_path.Count; idx ++)
            {
                if (CheckObstacleEdge(car_pos, my_path[idx])) {
                    min_idx --;
                    break;
                }
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

            lookahead_dist = Mathf.Sqrt(Mathf.Pow(car_pos[0] - lookahead_position[0], 2) + Mathf.Pow(car_pos[2] - lookahead_position[2], 2));
            goal_distance = Mathf.Sqrt(Mathf.Pow(car_pos[0] - goal_pos[0],2) + Mathf.Pow(car_pos[2] - goal_pos[2], 2));
        
            // if close to the look ahead waypoint, choose next waypoint to look next
            int max_lookahead_dist;

            float avg_curvature = 0f;
            for (int idx = next_waypoint_idx; idx < Math.Min(next_waypoint_idx + 10, my_path.Count); idx++)
            {
                avg_curvature += path_curvature[idx];
            }
            avg_curvature /= 10;
            Debug.Log("avg curv: " + avg_curvature);


            bool no_full_speed = (avg_curvature > 0.07f);
            if (no_full_speed)
                max_lookahead_dist = 8;
            else
                max_lookahead_dist = 16;

            while ((lookahead_dist < max_lookahead_dist) && (next_waypoint_idx < my_path.Count - 1))
            {
                if (CheckObstacleEdge(car_pos, my_path[next_waypoint_idx])) {
                    next_waypoint_idx --;
                    break;
                }
                // Debug.Log("dis:" + lookahead_dist);
                next_waypoint_idx ++;
                lookahead_dist = Mathf.Sqrt(Mathf.Pow(car_pos[0] - my_path[next_waypoint_idx][0], 2) + Mathf.Pow(car_pos[2] - my_path[next_waypoint_idx][2], 2));
            }
            Debug.Log(next_waypoint_idx);
            Debug.DrawLine(transform.position, my_path[next_waypoint_idx], Color.yellow);

            

            // a PD-controller to get desired velocity
            Vector3 position_error = lookahead_position - transform.position;

            float full_speed = 12f;
            if (speedup_stratagy == 1) {
                // 1. speed up to fullest when finishing 20% of the whole path
                // suitable for short dist
                float driven_percentage = (float) ((next_waypoint_idx - 1) / (float) my_path.Count);
                max_speed = (float) Math.Min(full_speed, full_speed * (driven_percentage / 0.2));
            } else if (speedup_stratagy == 2) {
                // 2. speed up to fullest within 10 seconds
                // suitable for long dist
                float driven_time_percentage = (float) ((Time.time - start_time) / 20f);
                max_speed = (float) Math.Min(full_speed, full_speed * driven_time_percentage);
            }

            if (no_full_speed)
                max_speed = Math.Min(max_speed, Math.Max(1f / path_curvature[next_waypoint_idx], 3));


            // just finished going back, speed up slowly
            float finished_hit_time_percentage = (float) ((Time.time - finished_hit_time) / 5f);
            max_speed = (float) Math.Min(max_speed, full_speed * finished_hit_time_percentage);
            

            // look ahead for sharp turns in the far front
            if (next_waypoint_idx < my_path.Count - 6) {
                for (int lookahead_idx = 0; lookahead_idx < 4; lookahead_idx ++)
                {
                    Vector3 cur_path = my_path[next_waypoint_idx + lookahead_idx] - my_path[next_waypoint_idx + lookahead_idx - 1];
                    Vector3 next_path = my_path[next_waypoint_idx + lookahead_idx + 1] - my_path[next_waypoint_idx + lookahead_idx];
                    if (Vector3.Angle(cur_path, next_path) > 70f) {
                        max_speed = Math.Min(max_speed, 5f);
                        just_hard_turn = true;
                        if (!just_hard_turn) {
                            after_hard_turn_time = Time.time;
                            just_hard_turn = true;
                        }
                        break;
                    }
                    else if (Vector3.Angle(cur_path, next_path) > 40f) {
                        max_speed = Math.Min(max_speed, 7f);
                        just_hard_turn = false;
                        break;
                    }
                }
            }

            max_speed = Math.Min(max_speed, max_speed_last_time + 0.05f);
            max_speed_last_time = max_speed;


            // this is how you access information about the terrain from a simulated laser range finder
            RaycastHit hit;
            // float longRange = 40f;
            float maxRange = 12f;
            List<Vector3> ray_list = new List<Vector3>();
            List<float> ray_dir_dist = new List<float>();
            ray_list.Add(new Vector3(0f, 0f, 1f));
            ray_list.Add(new Vector3(-0.5f, 0f, 0.85f));
            ray_list.Add(new Vector3(-0.7f, 0f, 0.7f));
            ray_list.Add(new Vector3(-0.85f, 0f, 0.5f));
            ray_list.Add(new Vector3(-1f, 0f, 0f));
            ray_list.Add(new Vector3(-0.85f, 0f, -0.5f));
            ray_list.Add(new Vector3(-0.7f, 0f, -0.7f));
            ray_list.Add(new Vector3(-0.5f, 0f, -0.85f));
            ray_list.Add(new Vector3(0f, 0f, -1f));
            ray_list.Add(new Vector3(0.5f, 0f, -0.85f));
            ray_list.Add(new Vector3(0.7f, 0f, -0.7f));
            ray_list.Add(new Vector3(0.85f, 0f, -0.5f));
            ray_list.Add(new Vector3(1f, 0f, 0f));
            ray_list.Add(new Vector3(0.85f, 0f, 0.5f));
            ray_list.Add(new Vector3(0.7f, 0f, 0.7f));
            ray_list.Add(new Vector3(0.5f, 0f, 0.85f));

            Vector3 sum_acc = new Vector3(0f, 0f, 0f);

            foreach (var ray in ray_list) {
                if (Physics.Raycast(transform.position + transform.up, ray, out hit, maxRange)) {
                    Vector3 closestObstacleDir = ray * hit.distance;
                    ray_dir_dist.Add(hit.distance);
                    Debug.DrawRay(transform.position, closestObstacleDir, Color.yellow);
                    float max_speed_factor = full_speed / max_speed;
                    float angle_factor = (180 - Vector3.Angle(m_Drone.velocity, ray)) / 90f;
                    float look_ahead_factor = (180 - Vector3.Angle((lookahead_position - transform.position), ray)) / 90f;
                    if ((ray[0] != 0) && (ray[2] != 0))
                        sum_acc += ray * (maxRange - hit.distance) * (-1f) * 0.1f * look_ahead_factor * 
                            max_speed_factor * angle_factor * (0.1f + 50f * (float)Math.Pow(1 - (hit.distance - 0.1f) / maxRange, 4f));
                    else
                        sum_acc += ray * (maxRange - hit.distance) * (-1f) * 0.15f * look_ahead_factor *
                            max_speed_factor * (0.1f + 80f * (float)Math.Pow(1 - (hit.distance - 0.1f) / maxRange, 4f));
                } else {
                    ray_dir_dist.Add(0f);
                }
            }

            Vector3 velocity_error = m_Drone.velocity * (max_speed - m_Drone.velocity.magnitude);
            Vector3 desired_acceleration = 3 * (1f * position_error + k_d * velocity_error);

            Vector3 final_acc = desired_acceleration + sum_acc;

            Debug.Log("Sum acc: " + (sum_acc[0]) + ", " + (sum_acc[2]));

            Debug.Log("Reg acc: " + (desired_acceleration[0]) + ", " + (desired_acceleration[2]));

            float horizontal = final_acc[0];
            float vertical = final_acc[2];

            // this is how you control the car
            Debug.Log("hor:" + horizontal + "  vertical:" + vertical + " Velo:" + m_Drone.velocity.magnitude + "max_speed:" + max_speed);




            // start to break when approaching target
            if (goal_distance < 10)
            {
                Debug.Log("Close enough, stopping");
                // m_Drone.Move(steering, 0f, 1f, 1f);
            }
            else
            {
                float acc_total = (float) Math.Sqrt(Math.Pow(horizontal, 2f) + Math.Pow(vertical, 2f));
                float reduce_factor = 2f / acc_total;
                m_Drone.Move(reduce_factor * horizontal, reduce_factor * vertical);
                // }
            }
            real_speed_last_time = m_Drone.velocity.magnitude;
        }
    }
}