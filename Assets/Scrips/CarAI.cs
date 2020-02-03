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

        private List<Node2> my_path;
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
		public float theta2 = Mathf.PI / 2;
		public bool nextPoint = false;
		public float sinceLastChange = 0;


        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            car_pos = start_pos;

            // get test path for terrain C
            RRTS rrt = new RRTS(terrain_manager_game_object, m_Car.MaxSpeed, m_Car.m_MaximumSteerAngle);
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

                UnityEngine.Debug.DrawLine(old_wp, wp.pos, Color.red, 100f);
                old_wp = wp.pos;
            }
        }


        /*private void FixedUpdate()
        {
            // stop driving once the goal node is reached
            //if (next >= my_path.Count - 1)
            //{
            //    m_Car.Move(0f, 0f, 0f, 0f);
            //    return;
            //}

            // update car position
            car_pos = new Vector3(transform.position.x, 0, transform.position.z);
            float goalDistance = Mathf.Sqrt(Mathf.Pow(car_pos[0] - goal_pos[0], 2) + Mathf.Pow(car_pos[2] - goal_pos[2], 2));
            //UnityEngine.Debug.Log("goalDistance " + goalDistance);
            if (goalDistance < 5)
            {
                Done = true;
            }

            // if the car is within a certain distance of the next node, increase the next index by one
            // the distance is squared (e.g. 25 = 5m distance from car to point)
            if (Mathf.Pow(car_pos[0] - my_path[next][0], 2) + Mathf.Pow(car_pos[2] - my_path[next][2], 2) < 25)
            {
                if (next < my_path.Count - 1)
                {
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

            UnityEngine.Debug.DrawLine(car_pos, my_path[next], Color.white, t);


            // update the predicted position
            //x += v * Mathf.Cos(theta) * t;
            //z += v * Mathf.Sin(theta) * t;


            // used to check the accuracy of the model
            //UnityEngine.Debug.Log(transform.position.x + " | " + transform.position.z);
            //UnityEngine.Debug.Log(x + " # " + z);

            // accelerate or break, depending on the desired velocity
            //TODO: improve the dynamics of the speed
            float gas_pedal = 0.6f;
            float break_pedal = 0;

            if (v < result[0])
            {
                gas_pedal = 1f;
                break_pedal = 0f;
            }
            else if (v > result[0])
            {
                gas_pedal = 1f;
                break_pedal = 0f;
            }

            // this is how you control the car

            //Move(1, 2, 3, 4)
            //    1: steering (-1 = left, 0 = nothing, 1 = right)
            //    2: gas pedal (0 - 1)
            //    3: break (-1 = backwards, 0 = nothing)
            //    4: handbreak
            //UnityEngine.Debug.Log(steering_angle + " " + gas_pedal + " " + break_pedal + " " + 0f);
            if (Done == true)
            {
                m_Car.Move(steering_angle, gas_pedal, break_pedal, 0f);
                UnityEngine.Debug.Log("DONE");
            }
            else
            {
				float steer = steering_angle/max_steer_angle;
				UnityEngine.Debug.Log("steering_angle: " + steering_angle + " CurrentSteerAngle: " + m_Car.CurrentSteerAngle);
				UnityEngine.Debug.Log("steer: " + steer + " gas_pedal: " + gas_pedal + " break_pedal " + break_pedal);
				m_Car.Move(steer, gas_pedal, break_pedal, 0f);
            }

        }*/


        /**
        returns a steering angle and speed, depending on the angle between the two points and the direction of the next point
        */
        /*private double[] calculateDesiredConfiguration()
        {
            Vector3 point1 = new Vector3();
            Vector3 point2 = new Vector3();
            Vector3 point3 = new Vector3();

            Vector3 vector1 = new Vector3();
            Vector3 vector2 = new Vector3();
            Vector3 car_next = new Vector3();

            double[] result = new double[2];
            if (next != my_path.Count - 1)
            {


                // get the last, next and the one after the next point
                point1 = my_path[next - 1];
                point2 = my_path[next];
                point3 = my_path[next + 1];

                // build the two vectors between the points and one between the car and the next point
                vector1 = point2 - point1;
                vector2 = point3 - point2;
                car_next = point2 - car_pos;

                // get the angle between vector 1 & 2 in degrees
                double angle = Vector3.Angle(vector1, vector2);

                // speed (needs improvement)
                
                if (angle > 81)
                {
                    result[0] = m_Car.MaxSpeed * 0.2f;
                }
                else
                {
                    result[0] = (100 - ((angle * 100) / 90)) * m_Car.MaxSpeed * 0.2f;
                }
                
                result[0] = 10;
            }
            else
            {
                // get the last, next and the one after the next point
                point1 = my_path[next - 1];
                point2 = my_path[next];

                // build the two vectors between the points and one between the car and the next point
                vector1 = point2 - point1;
                car_next = point2 - car_pos;
                result[0] = 10;
            }

            // only apply model dynamics when driving
            if (v > 0)
            {
                //UnityEngine.Debug.Log(theta);

                // get the angle of the car - next vector
                // Atan() returns the smallest angle (from -pi/2 to pi/2)
                float vectorAngle = Mathf.Atan(car_next[2] / car_next[0]);
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

                // will be -1 for right and 1 for left
                int direction = 0;
                float beta = theta - vectorAngle;
                if (beta > 0 && beta < Mathf.PI)
                {
                    direction = -1;
                }
                else if (beta > 0 && beta >= Mathf.PI)
                {
                    direction = 1;
                }
                else if (beta < 0 && beta > -Mathf.PI)
                {
                    direction = 1;
                }
                else
                {
                    direction = -1;
                }

                float[] steering_angles = new float[11];
                for (int i = 0; i <= 10; i++)
                {
                    steering_angles[i] = (i / 10f) * direction;
                }

                // calculates the steering angle (phi) which results in a minimal difference between theta and the vector angle
                float phi_min = 0;
                float best = float.MaxValue;
                float x = 0;

                foreach (float angle in steering_angles)
                {
                    x = Mathf.Abs(theta + (((v * t) / L) * Mathf.Tan(angle * max_steer_angle)) - vectorAngle);

                    if (x < best)
                    {
                        best = x;
                        phi_min = angle;
                    }
                }
                result[1] = -phi_min;
            }
            else
            {
                result[1] = 0f;
            }

            return result;
        }*/

        private void FixedUpdate()
        {
			iter++;
			sinceLastChange += m_Car.CurrentSpeed*t;
			if(nextPoint) {
				next++;
				nextPoint = false;
			}
			if(Done) {
				m_Car.Move(0f, 0f ,0f, 1f);
			} else {
				//UnityEngine.Debug.Log("----------------------------- Iteration: " + iter + " -----------------------------");
				car_pos = new Vector3(transform.position.x, 0, transform.position.z);
				UnityEngine.Debug.Log(next + "/" + my_path.Count);
				float[] results = Steer(car_pos, my_path[next].pos);
				float steer = -results[1]/max_steer_angle;
				UnityEngine.Debug.Log(" Steer: " + steer + " velocity: " + m_Car.CurrentSpeed + " accel: " + results[0] + " Brake: " + results[2]);
				m_Car.Move(steer, results[0], 0f, results[2]);
			}			
        }

    	private float[] Steer(Vector3 curPos, Vector3 pos)
        {
			t = Time.fixedDeltaTime;
			float L = 3.0f;
			float Accel = 1f;
			float footbreak = 0f;
			
			theta = nfmod(-m_Car.transform.eulerAngles.y * (Mathf.PI/180.0f) + Mathf.PI / 2.0f, 2*Mathf.PI);
			float v = m_Car.CurrentSpeed;

			float xDist = pos[0] - curPos[0];
			float zDist = pos[2] - curPos[2];

			float newDistance = Mathf.Sqrt(Mathf.Pow(xDist,2) + Mathf.Pow(zDist, 2));
			float goalDistance = Mathf.Sqrt(Mathf.Pow(curPos[0] - goal_pos[0],2) + Mathf.Pow(curPos[2] - goal_pos[2], 2));
			if(newDistance < 5 && !nextPoint) {
				UnityEngine.Debug.Log("Changing to next point");
				nextPoint = true;
				sinceLastChange = 0;
			} else if(goalDistance < 10) {
				Done = true;
			}

			float vectorAngle = Mathf.Atan(zDist / xDist);
			//UnityEngine.Debug.Log("Vector angle before " + vectorAngle);
			if (
				(xDist < 0 && zDist > 0 && vectorAngle < 0) // if car_next lies in the second quadrant
				|| (xDist < 0 && vectorAngle == 0) // if car_next lies on the x axis and points to the left
				|| (xDist < 0 && zDist < 0) // if car_next lies in the third quadrant
			) {
				vectorAngle += Mathf.PI;
			}
			else if (xDist > 0 && zDist < 0)// if car_next lies in the fourth quadrant
			{
				vectorAngle += 2 * Mathf.PI;
			}
			//UnityEngine.Debug.Log("Vector angle after " + vectorAngle);
			float beta = 0;
			if (vectorAngle < Mathf.PI/2) {
				//UnityEngine.Debug.Log("Vector in the first quadrant");
				if(theta < Mathf.PI/2) {
					//UnityEngine.Debug.Log("Theta in the first quadrant");
					beta = vectorAngle - theta;
				} else if(theta < Mathf.PI) {
					//UnityEngine.Debug.Log("Theta in the second quadrant");
					beta = vectorAngle - theta;
				} else if(theta < 3*Mathf.PI/2) {
					//UnityEngine.Debug.Log("Theta in the third quadrant");
					beta = -vectorAngle;
				} else if(theta < 2*Mathf.PI) {
					//UnityEngine.Debug.Log("Theta in the fourth quadrant");
					beta = 2*Mathf.PI+vectorAngle-theta;
				}
			} else if (vectorAngle < Mathf.PI) {
				//UnityEngine.Debug.Log("Vector in the second quadrant");
				if(theta < Mathf.PI/2) {
					//UnityEngine.Debug.Log("Theta in the first quadrant");
					beta =vectorAngle  - theta;
				} else if(theta < Mathf.PI) {
					//UnityEngine.Debug.Log("Theta in the second quadrant");
					beta = vectorAngle - theta;
				} else if(theta < 3*Mathf.PI/2) {
					//UnityEngine.Debug.Log("Theta in the third quadrant");
					beta = vectorAngle - theta;
				} else if(theta < 2*Mathf.PI) {
					//UnityEngine.Debug.Log("Theta in the fourth quadrant");
					beta = -vectorAngle;
				}
			} else if (vectorAngle < 3*Mathf.PI/2) {
				//UnityEngine.Debug.Log("Vector in the third quadrant");
				if(theta < Mathf.PI/2) {
					//UnityEngine.Debug.Log("Theta in the first quadrant");
					beta = vectorAngle;
				} else if(theta < Mathf.PI) {
					//UnityEngine.Debug.Log("Theta in the second quadrant");
					beta = vectorAngle - theta;
				} else if(theta < 3*Mathf.PI/2) {
					//UnityEngine.Debug.Log("Theta in the third quadrant");
					beta = vectorAngle - theta;
				} else if(theta < 2*Mathf.PI) {
					//UnityEngine.Debug.Log("Theta in the fourth quadrant");
					beta = vectorAngle - theta;
				}
			} else if (vectorAngle < 2*Mathf.PI) {
				//UnityEngine.Debug.Log("Vector in the fourth quadrant");
				if(theta < Mathf.PI/2) {
					//UnityEngine.Debug.Log("Theta in the first quadrant");
					beta = -2*Mathf.PI - vectorAngle - theta;
				} else if(theta < Mathf.PI) {
					//UnityEngine.Debug.Log("Theta in the second quadrant");
					beta = -(Mathf.Sign(vectorAngle - Mathf.PI - theta));
				} else if(theta < 3*Mathf.PI/2) {
					//UnityEngine.Debug.Log("Theta in the third quadrant");
					beta = vectorAngle - theta;
				} else if(theta < 2*Mathf.PI) {
					//UnityEngine.Debug.Log("Theta in the fourth quadrant");
					beta = vectorAngle - theta;
				}
			} else {
				//UnityEngine.Debug.Log("None of the above. Should not happen.");
				beta = vectorAngle - theta;
			}
			UnityEngine.Debug.Log("Distance to next point: " + newDistance);
			if(next < my_path.Count-1) {
				if(newDistance < 30 && v > 30) {
					float xNext = pos[0] - my_path[next+1].pos[0];
					float zNext = pos[2] - my_path[next+1].pos[2];
					float nextAngle = Mathf.Atan(zNext/xNext);
					float nextBeta = 0;
					if (
						(xNext < 0 && zNext > 0 && nextAngle < 0) // if car_next lies in the second quadrant
						|| (xNext < 0 && nextAngle == 0) // if car_next lies on the x axis and points to the left
						|| (xNext < 0 && zNext < 0) // if car_next lies in the third quadrant
					) {
						nextAngle += Mathf.PI;
					}
					else if (xNext > 0 && zNext < 0)// if car_next lies in the fourth quadrant
					{
						nextAngle += 2 * Mathf.PI;
					}

					//nextBeta = theta - nfmod(nextBeta, 2*Mathf.PI);
					if (nextAngle < Mathf.PI/2) {
						//UnityEngine.Debug.Log("Vector in the first quadrant");
						if(theta < Mathf.PI/2) {
							//UnityEngine.Debug.Log("Theta in the first quadrant");
							nextBeta = nextAngle - theta;
						} else if(theta < Mathf.PI) {
							//UnityEngine.Debug.Log("Theta in the second quadrant");
							nextBeta = nextAngle - theta;
						} else if(theta < 3*Mathf.PI/2) {
							//UnityEngine.Debug.Log("Theta in the third quadrant");
							nextBeta = -nextAngle;
						} else if(theta < 2*Mathf.PI) {
							//UnityEngine.Debug.Log("Theta in the fourth quadrant");
							nextBeta = 2*Mathf.PI+nextAngle-theta;
						}
					} else if (nextAngle < Mathf.PI) {
						//UnityEngine.Debug.Log("Vector in the second quadrant");
						if(theta < Mathf.PI/2) {
							//UnityEngine.Debug.Log("Theta in the first quadrant");
							nextBeta =nextAngle  - theta;
						} else if(theta < Mathf.PI) {
							//UnityEngine.Debug.Log("Theta in the second quadrant");
							nextBeta = nextAngle - theta;
						} else if(theta < 3*Mathf.PI/2) {
							//UnityEngine.Debug.Log("Theta in the third quadrant");
							nextBeta = nextAngle - theta;
						} else if(theta < 2*Mathf.PI) {
							//UnityEngine.Debug.Log("Theta in the fourth quadrant");
							nextBeta = -nextAngle;
						}
					} else if (nextAngle < 3*Mathf.PI/2) {
						//UnityEngine.Debug.Log("Vector in the third quadrant");
						if(theta < Mathf.PI/2) {
							//UnityEngine.Debug.Log("Theta in the first quadrant");
							nextBeta = nextAngle;
						} else if(theta < Mathf.PI) {
							//UnityEngine.Debug.Log("Theta in the second quadrant");
							nextBeta = nextAngle - theta;
						} else if(theta < 3*Mathf.PI/2) {
							//UnityEngine.Debug.Log("Theta in the third quadrant");
							nextBeta = nextAngle - theta;
						} else if(theta < 2*Mathf.PI) {
							//UnityEngine.Debug.Log("Theta in the fourth quadrant");
							nextBeta = nextAngle - theta;
						}
					} else if (nextAngle < 2*Mathf.PI) {
						//UnityEngine.Debug.Log("Vector in the fourth quadrant");
						if(theta < Mathf.PI/2) {
							//UnityEngine.Debug.Log("Theta in the first quadrant");
							nextBeta = -2*Mathf.PI - nextAngle - theta;
						} else if(theta < Mathf.PI) {
							//UnityEngine.Debug.Log("Theta in the second quadrant");
							nextBeta = -(Mathf.Sign(nextAngle - Mathf.PI - theta));
						} else if(theta < 3*Mathf.PI/2) {
							//UnityEngine.Debug.Log("Theta in the third quadrant");
							nextBeta = nextAngle - theta;
						} else if(theta < 2*Mathf.PI) {
							//UnityEngine.Debug.Log("Theta in the fourth quadrant");
							nextBeta = nextAngle - theta;
						}
					} else {
						//UnityEngine.Debug.Log("None of the above. Should not happen.");
						nextBeta = nextAngle - theta;
					}
					nextBeta = nfmod(nextBeta, 2*Mathf.PI);
					UnityEngine.Debug.Log("Theta: " + theta + " nextBeta: " + nextBeta);
					if(nextBeta < Mathf.PI/4 && nextBeta > - Mathf.PI/4) {
						Accel = 1f;
						UnityEngine.Debug.Log("Next beta less than PI/4");
					} else if(nextBeta < Mathf.PI/2 && nextBeta > - Mathf.PI/2) {
						if(newDistance > 10) {
							Accel = 0;
						} else {
							Accel = 0;
							footbreak = 0.5f;
						}
						UnityEngine.Debug.Log("Next beta less than PI/2");
					} else if(nextBeta < 3*Mathf.PI/4 && nextBeta > - 3*Mathf.PI/4) {
						if(newDistance > 10) {
							Accel = 0;
						} else {
							Accel = 0;
							footbreak = 1f;
						}
						UnityEngine.Debug.Log("Next beta less than 3*PI/2");
						
					} else if(nextBeta < Mathf.PI && nextBeta > - Mathf.PI) {
						if(newDistance > 10) {
							Accel = 0;
							footbreak = 0.5f;
						} else {
							Accel = 0;
							footbreak = 1f;
						}
						UnityEngine.Debug.Log("Next beta less than PI");
					} else if(nextBeta < 2*Mathf.PI && nextBeta > - 2*Mathf.PI) {
						if(newDistance > 20) {
							Accel = 0;
							footbreak = 0.5f;
						} else {
							Accel = 0;
							footbreak = 1f;
						}
						UnityEngine.Debug.Log("Next beta more than PI");
					}
					UnityEngine.Debug.Log("Next angle: " + nextAngle + " Next beta: " + nextBeta + " Theta: " + theta);
				}
			}

			//UnityEngine.Debug.Log("Beta " + beta);

			//vectorAngle += theta;
			if(beta < -max_steer_angle) {
				beta = -max_steer_angle;
			} else if(beta > max_steer_angle) {
				beta = max_steer_angle;
			}
			float[] results = {Accel, beta, footbreak};
			return results;
        }
		public float nfmod(float a,float b)
		{
			return a - b * Mathf.Floor(a / b);
		}
	}
}