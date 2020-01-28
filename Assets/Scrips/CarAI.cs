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
        private Vector3 car_pos;
        private const float max_steer_angle = (25f / 360f) * 2f * Mathf.PI;
        private float steering_angle = 0;
        private float theta = Mathf.PI / 2;
        private float v = 0;
        private float t;
        private const float L = 3f;
        private float phi = 0;
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
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            car_pos = start_pos;

            // get test path for terrain C
            RRTS rrt = new RRTS(terrain_manager_game_object);
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
		
		private void FixedUpdate123()
        {
            // stop driving once the goal node is reached
            /*if (next >= my_path.Count - 2)
            {
                m_Car.Move(0f, 0f, 0f, 0f);
                return;
            }
			*/
			// update car position
            car_pos = new Vector3(transform.position.x, 0, transform.position.z);

			
            // if the car is within a certain distance of the next node, increase the next index by one
            // the distance is squared (e.g. 25 = 5m distance from car to point)
            if (Mathf.Pow(car_pos[0] - my_path[next][0], 2) + Mathf.Pow(car_pos[2] - my_path[next][2], 2) < 1)
            {
                next++;
            }

			// the time between the fixed updates
            //t = Time.fixedDeltaTime;
			t = 1;

            // velocity in m/s
            v = 0.6f;

			float xDist = my_path[next][0] - car_pos[0];
			float zDist = my_path[next][2] - car_pos[2];

			float newDistance = Mathf.Sqrt(Mathf.Pow(xDist,2) + Mathf.Pow(zDist, 2));
			//float goalDistance = Mathf.Sqrt(Mathf.Pow(car_pos[0] - goal_pos[0],2) + Mathf.Pow(car_pos[2] - goal_pos[2], 2));

			//if(goalDistance < 0.1) {
				//return new Node2(nearest, newPos, 0, 0, theta);
			//}
			float vectorAngle = (float)Mathf.Atan(zDist / xDist);
			/*  TODO:
					If on x-axis
					If on y-axis
					Check if all quadrants work
			*/
			if ((xDist < 0 && zDist > 0) || ( xDist < 0 && zDist < 0)){// if car_next lies in the second or thrid quadrant
				vectorAngle += Mathf.PI;
			}
			if(
				(theta > vectorAngle) && ((xDist < 0 && zDist > 0) || ( xDist > 0 && zDist < 0))
				|| (theta < vectorAngle) && ((xDist < 0 && zDist < 0) || ( xDist > 0 && zDist > 0))
			) {

				theta += v/L*Mathf.Tan(vectorAngle);
			} else {
				theta -= v/L*Mathf.Tan(vectorAngle);
			}
			float x = car_pos[0] + v * Mathf.Cos(theta);
			float z = car_pos[2] + v * Mathf.Sin(theta);
			Vector3 updatedPos = new Vector3(x, 0, z);
			//UnityEngine.Debug.DrawLine(newPos, updatedPos, Color.blue, 100f);
			//UnityEngine.Debug.Log("Angle = " + vectorAngle);
			//UnityEngine.Debug.Log("Tan(Angle) = " + Mathf.Tan(vectorAngle));
			//UnityEngine.Debug.Log("Theta = " + theta);
			float phi_min = 0;
			float best = float.MaxValue;
			float it = -1f * max_steer_angle;
			float xx = 0;
			int number_of_iterations = 50;

			for (float i = -1; i <= 1; i += 2f / number_of_iterations)
			{
				xx = Mathf.Abs(theta + ((v*(float)t / L) * Mathf.Tan(it)) - vectorAngle);

				if (xx < best)
				{
					best = xx;
					phi_min = i;
				}

				it += (1f * 2 * max_steer_angle) / (number_of_iterations - 1);
			}
			steering_angle = -phi_min;
			m_Car.Move(steering_angle, v, 0f, 0f);
		}

		
        private void FixedUpdate()
        {
            // stop driving once the goal node is reached
            /*if (next >= my_path.Count - 1)
            {
				UnityEngine.Debug.Log("ASDFASDFASDGASDSDGDA");
                m_Car.Move(0f, 0f, 0f, 0f);
                return;
            }*/

            // update car position
            car_pos = new Vector3(transform.position.x, 0, transform.position.z);
			float goalDistance = Mathf.Sqrt(Mathf.Pow(car_pos[0] - goal_pos[0],2) + Mathf.Pow(car_pos[2] - goal_pos[2], 2));
			UnityEngine.Debug.Log("goalDistance " + goalDistance);
			if(goalDistance < 10) {
				Done = true;
			}

            // if the car is within a certain distance of the next node, increase the next index by one
            // the distance is squared (e.g. 25 = 5m distance from car to point)
            if (Mathf.Pow(car_pos[0] - my_path[next][0], 2) + Mathf.Pow(car_pos[2] - my_path[next][2], 2) < 100)
            {
				if(next < my_path.Count-2) {
					next++;
				}
            }

            // the time between the fixed updates
            t = Time.fixedDeltaTime;

            // velocity in m/s
            v = m_Car.CurrentSpeed / 2.23693629f;

            // get the speed and steering angle which leads the car towards the next point
            double[] result = calculateDesiredConfiguration();

            // steering angle [-1 - 1]
            steering_angle = (float)result[1];

            // angle of wheels in radians
            phi = -steering_angle * max_steer_angle;

            // angle of car in radians
            theta += (((v * t) / L) * Mathf.Tan(phi)) % (2 * Mathf.PI);

            
            // update the predicted position
            //x += v * Mathf.Cos(theta) * t;
            //z += v * Mathf.Sin(theta) * t;
            

            // used to check the accuracy of the model
            //UnityEngine.Debug.Log(transform.position.x + " | " + transform.position.z);
            //UnityEngine.Debug.Log(x + " # " + z);

            // accelerate or break, depending on the desired velocity
            //TODO: improve the dynamics of the speed
            float gas_pedal = 0.3f;
            float break_pedal = 0;
			/*
            if (v < result[0])
            {
                gas_pedal = 0.5f;
                break_pedal = 0f;
            }
            else if (v > result[0])
            {
                gas_pedal = 0f;
                break_pedal = -0.5f;
            }*/

            // this is how you control the car
            
            //Move(1, 2, 3, 4)
            //    1: steering (-1 = left, 0 = nothing, 1 = right)
            //    2: gas pedal (0 - 1)
            //    3: break (-1 = backwards, 0 = nothing)
            //    4: handbreak
			UnityEngine.Debug.Log(steering_angle + " " + gas_pedal + " " + break_pedal + " " + 0f);
			if(Done == true) {
				m_Car.Move(0f, 0f, 0f, 1f);
				UnityEngine.Debug.Log("DONE");
			} else {
				m_Car.Move(steering_angle, gas_pedal, break_pedal, 0f);
			}
            
        }
		

        /**
        returns a steering angle and speed, depending on the angle between the two points and the direction of the next point
        */
        private double[] calculateDesiredConfiguration()
        {
            // get the last, next and the one after the next point
            Vector3 point1 = my_path[next - 1];
            Vector3 point2 = my_path[next];
            Vector3 point3 = my_path[next + 1];

            // build the two vectors between the points and one between the car and the next point
            Vector3 vector1 = point2 - point1;
            Vector3 vector2 = point3 - point2;
            Vector3 car_next = point2 - car_pos;

            // get the angle between vector 1 & 2 in degrees
            double angle = Vector3.Angle(vector1, vector2);
            double[] result = new double[2];

            // speed (needs improvement)
            if (angle > 81)
            {
                result[0] = 0.3;
            }
            else
            {
                result[0] = 100 - ((angle * 100) / 90);
            }

            // only apply model dynamics when driving
            if (v > 0)
            {
                // get the angle of the car - next vector
                // Atan() returns the smallest angle (from -pi/2 to pi/2)
                float vectorAngle = Mathf.Atan(car_next[2] / car_next[0]);
                // TODO: test all edge cases
                if (
                    // if car_next lies in the second quadrant
                    (car_next[0] < 0 && car_next[2] > 0 && vectorAngle < 0)
                    // if car_next lies on the x axis and points to the left
                    || (car_next[0] < 0 && vectorAngle == 0)
                    // if car_next lies in the third quadrant
                    || (car_next[0] < 0 && car_next[2] < 0)
                )
                {
                    vectorAngle += Mathf.PI;
                }
                else if (
                  // if car_next lies in the fourth quadrant
                  car_next[0] > 0 && car_next[2] < 0
              	)
                {
                    vectorAngle += 2 * Mathf.PI;
                }

                // calculates the steering angle (phi) which results in a minimal difference between theta and the vector angle
                // increase the number of iterations in order to get a more precise result
                float phi_min = 0;
                float best = float.MaxValue;
                float iter = -1f * max_steer_angle;
                float x = 0;
                int number_of_iterations = 20;

                for (float i = -1; i <= 1; i += 2f / number_of_iterations)
                {
                    x = Mathf.Abs(theta + (((v * t) / L) * Mathf.Tan(iter)) - vectorAngle);

                    if (x < best)
                    {
                        best = x;
                        phi_min = i;
                    }

                    iter += (1f * 2 * max_steer_angle) / (number_of_iterations - 1);
                }
                result[1] = -phi_min;
            }
            else
            {
                result[1] = 0f;
            }

            return result;
        }
    }
}
