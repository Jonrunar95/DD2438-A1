using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System;
namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(DroneController))]
    public class DroneAI : MonoBehaviour
    {

        private DroneController m_Drone; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        private List<Vector3> my_path;
        private Vector3 drone_pos;
        private Vector3 v = new Vector3();
        private float t;
        private float x = 0;
        private float z = 0;
        // stores the index of the next node the car should head toward
        private int next;
        public int iter;
        public Vector3 start_pos;
        public Vector3 goal_pos;
        public bool Done = false;

        private void Start()
        {
            // get the drone controller
            m_Drone = GetComponent<DroneController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

            drone_pos = start_pos;

            // get test path for terrain C
            RRTS rrt = new RRTS(terrain_manager_game_object, null, m_Drone);
            //my_path = rrt.testPath();
            my_path = rrt.Run();

            // initialize the starting position for the model
            x = start_pos[0];
            z = start_pos[2];

            // set the next node in the path to be the one after the start node
            next = 1;

            // plot the path in the scene window
            Vector3 old_wp = start_pos;

            foreach (var wp in my_path)
            {

                UnityEngine.Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }
        }

        private Vector3 drone_pos_old = new Vector3();


        private void FixedUpdate()
        {
            drone_pos_old = drone_pos;

            // update drone position
            drone_pos = new Vector3(transform.position.x, 0, transform.position.z);

            // if the car is within a certain distance of the next node, increase the next index by one
            // the distance is squared (e.g. 25 = 5m distance from car to point)
            if (Mathf.Pow(drone_pos[0] - my_path[next][0], 2) + Mathf.Pow(drone_pos[2] - my_path[next][2], 2) < 25)
            {
                if (next < my_path.Count - 1)
                {
                    next++;
                }
            }

            // the time between the fixed updates
            t = Time.fixedDeltaTime;

            // velocity vector
            // get difference in positions
            v = drone_pos - drone_pos_old;

            if (drone_pos_old + v != drone_pos)
            {
                v = drone_pos_old - drone_pos;
            }

            // calculate velocity with delta_s and delta_t
            v[0] = v[0] / t;
            v[2] = v[2] / t;

            double[] result = calculateDesiredConfiguration();

            UnityEngine.Debug.DrawLine(drone_pos, my_path[next], Color.white, t);

            m_Drone.Move((float)result[0], (float)result[1]);
        }

        // I don't know why 500 is a good value, but it works
        private const float max_speed = 500;

        private double[] calculateDesiredConfiguration()
        {
            double[] result = new double[2];
            double[] delta_a = new double[2];

            // vector from drone to next point
            Vector3 drone_next = my_path[next] - drone_pos;

            if (drone_pos + drone_next != my_path[next])
            {
                drone_next = drone_pos - my_path[next];
            }

            // the resulting velocity vector when applying acceleration according to drone_next
            Vector3 v_prime = new Vector3(drone_next[0], 0, drone_next[2]);

            // cap the speed
            float max = Mathf.Max(Mathf.Abs(v_prime[0]), Mathf.Abs(v_prime[2]));
            if (max > max_speed * t)
            {
                v_prime[0] = (v_prime[0] / max) * max_speed * t;
                v_prime[2] = (v_prime[2] / max) * max_speed * t;
            }

            // the vector between v and v_prime, used to set the result values
            Vector3 v_v_prime = v_prime - v;

            if (v + v_v_prime != v_prime)
            {
                v_v_prime = v - v_prime;
            }

            result[0] = v_v_prime[0];
            result[1] = v_v_prime[2];

            return result;
        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}