﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


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
            // Replace the code below that makes a random path
            // ...

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;
            
            // RRT function
            Rrt(start_pos, goal_pos);
            
            // Replace the code below that makes a random path
            List<Vector3> my_path = new List<Vector3>();

            my_path.Add(start_pos);

            for (int i = 0; i < 3; i++)
            {
                Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
                my_path.Add(waypoint);
            }
            my_path.Add(goal_pos);
            
            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            /*foreach (var wp in my_path)
            {
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }*/

            
        }


        private void FixedUpdate()
        {
            // Execute your path here
            // ...

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

        public void Rrt(Vector3 start_pos, Vector3 goal_pos)
        {
            float xLow = terrain_manager.myInfo.x_low;
            float xHigh = terrain_manager.myInfo.x_high;
            float zLow = terrain_manager.myInfo.z_low;
            float zHigh = terrain_manager.myInfo.z_high;
            float distanceGoal;
            const int stepSize = 5;
            const int distanceThreshold = 10;
            bool pathFound = false;
            GameObject car = GameObject.Find("Car");
            Vector3 randomPoint = FindRandomPoint(xLow, xHigh, zLow, zHigh);
            Debug.DrawLine(start_pos, randomPoint, Color.red, 100f);
            /*while (pathFound == false)
            {
                Vector3 randomPoint = FindRandomPoint(xLow, xHigh, zLow, zHigh);
                Debug.DrawLine(start_pos, randomPoint, Color.red, 100f);
                
                distanceGoal = Vector3.Distance(car.transform.position, goal_pos);
                if (distanceGoal < distanceThreshold)
                {
                    pathFound = true;
                }
                
            }*/
        }

        public Vector3 FindRandomPoint(float xLow, float xHigh, float zLow, float zHigh)
        {
            int i;
            int j;
            Vector3 randomPoint = new Vector3(0,0,0);
            float obstacle = 1.0f;
            
            while (obstacle == 1.0f)
            {
                randomPoint = new Vector3(UnityEngine.Random.Range(xLow, xHigh), 0,
                    UnityEngine.Random.Range(zLow, zHigh));
                i = terrain_manager.myInfo.get_i_index(randomPoint.x);
                j = terrain_manager.myInfo.get_j_index(randomPoint.z);
                obstacle = terrain_manager.myInfo.traversability[i, j];
            }
            return randomPoint;
        }
    }
}
