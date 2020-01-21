using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        private List<Vector3> my_path;
        private const double max_steer_angle = (25f / 360f) * 2f * Math.PI;
        private float angle = 0;
        private double theta = Math.PI / 2f;
        private double v = 0;
        private double t;
        private const double L = 3;
        private double phi = 0;
        private double x = 0;
        private double z = 0;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

            RRT rrt = new RRT(terrain_manager_game_object);
            rrt.test(m_Car);
            my_path = rrt.testPath();

            // initialize the starting position for the model
            x = start_pos[0];
            z = start_pos[2];

            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                UnityEngine.Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }
        }

        private void FixedUpdate()
        {
            //TODO: determine the angle of the steering based on where the car sohuld go before applying the update on the model
            //      implement a variable acceleration, depending on the path
            //      -> both should be doable by looking at the distance to the next node and then act according to the angle between 
            //         the car's current theta and the vector leading to the node after the next one

            // the time between the fixed updates. used to scale the velocity, given in m/s
            t = Time.fixedDeltaTime;

            // angle in percentage
            angle = -0.2f;

            // velocity in m/s
            v = m_Car.CurrentSpeed / 2.23693629f;

            // angle of wheels in radians
            phi = -angle * max_steer_angle;

            // angle of car in radians
            theta += ((v * t) / L) * Math.Tan(phi);

            // update the predicted position
            x += v * Math.Cos(theta) * t;
            z += v * Math.Sin(theta) * t;

            /*
            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            UnityEngine.Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));
            */

            // used to check the accuracy of the model
            UnityEngine.Debug.Log(transform.position.x + " | " + transform.position.z);
            UnityEngine.Debug.Log(x + " # " + z);
            
            m_Car.Move(angle, 1f, 0f, 0f);

            // this is how you control the car
            /*
            Move(1, 2, 3, 4)
                1: steering (-1 = left, 0 = nothing, 1 = right)
                2: gas pedal
                3: break (-1 = backwards, 0 = nothing)
                4: ? handbreak ?
            */
        }

        private Vector3 closest(Vector3 current)
        {
            double best = Double.MaxValue;
            Vector3 result = new Vector3();
            foreach (Vector3 v in my_path)
            {
                if (Vector3.Distance(current, v) < best)
                {
                    result = v;
                }
            }

            return result;
        }
    }
}
