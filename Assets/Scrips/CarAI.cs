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
        private float theta = Mathf.PI / 2;
        private float t;

        private float x = 0;
        private float z = 0;
        // stores the index of the next node the car should head toward
        private int next;
		public int nextnext;
        public int iter;
        public Vector3 start_pos;
        public Vector3 goal_pos;
        public bool Done = false;
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
            my_path = rrt.Run();

            // initialize the starting position for the model
            x = start_pos[0];
            z = start_pos[2];

            // set the next node in the path to be the one after the start node
            next = 1;
			nextnext = 2;
        }

        private void FixedUpdate()
        {
			iter++;
			sinceLastChange += m_Car.CurrentSpeed*t;
			if(Done) {
				m_Car.Move(0f, 0f ,0f, 1f);
			} else {
				//UnityEngine.Debug.Log("----------------------------- Iteration: " + iter + " -----------------------------");
				if(nextPoint) {
					sinceLastChange += 1;
					if(sinceLastChange > 4) {
						next++;
						sinceLastChange = 0;
						nextPoint = false;
						if(nextnext <= next) {
							nextnext++;
						}
					}
				}
				car_pos = new Vector3(transform.position.x, 0, transform.position.z);
				//UnityEngine.Debug.Log(next + "/" + my_path.Count);
				float[] results = Steer(car_pos, my_path[next].pos);
				float steer = -results[1]/max_steer_angle;
				//UnityEngine.Debug.Log(" Steer: " + steer + " velocity: " + m_Car.CurrentSpeed + " accel: " + results[0] + " Brake: " + results[2]);
				m_Car.Move(steer, results[0], 0f, results[2]);
			}			
        }

    	private float[] Steer(Vector3 curPos, Vector3 pos)
        {
			t = Time.fixedDeltaTime;
			float Accel = 1f;
			float footbreak = 0f;
			
			theta = nfmod(-m_Car.transform.eulerAngles.y * (Mathf.PI/180.0f) + Mathf.PI / 2.0f, 2*Mathf.PI);
			float v = m_Car.CurrentSpeed;

			float xDist = pos[0] - curPos[0];
			float zDist = pos[2] - curPos[2];

			float newDistance = Mathf.Sqrt(Mathf.Pow(xDist,2) + Mathf.Pow(zDist, 2));
			float goalDistance = Mathf.Sqrt(Mathf.Pow(curPos[0] - goal_pos[0],2) + Mathf.Pow(curPos[2] - goal_pos[2], 2));
			if(newDistance < 5) {
				nextPoint = true;

			} if(newDistance < 1) {
				next++;
				nextPoint = false;
				sinceLastChange = 0;
				if(nextnext <= next) {
					nextnext = next +1;
				}
			}
			if(goalDistance < 10) {
				Done = true;
				float[] ret = {0f, 0f, 0f};
				return ret;
			}

			float vectorAngle = Mathf.Atan(zDist / xDist);
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

			if(nextnext < my_path.Count) {
				float nextnextX = my_path[nextnext].pos[0] - curPos[0];
				float nextnextZ = my_path[nextnext].pos[2] - curPos[2];
				float nextnextdistance = Mathf.Sqrt(Mathf.Pow(nextnextX, 2) + Mathf.Pow(nextnextZ, 2));
				if(nextnextdistance < 10) {
					nextnext++;
				}
				float xNext = my_path[nextnext].pos[0] - pos[0];
				float zNext = my_path[nextnext].pos[2] - pos[2];
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
				nextBeta = theta - nextAngle;

				if(v < 30) {
					Accel = 1f;
					footbreak = 0;
				}
				else if(nextBeta < Mathf.PI/4 && nextBeta > - Mathf.PI/4) {
					Accel = 1f;
					if(newDistance < 20) {
						Accel = 0f;
					}
					//UnityEngine.Debug.Log("Next beta less than PI/4");
				} else if(nextBeta < Mathf.PI/2 && nextBeta > - Mathf.PI/2) {
					if(newDistance > 30) {
						Accel = 0f;
					} else {
						Accel = 0f;
						footbreak = 1f;
					}
					//UnityEngine.Debug.Log("Next beta less than PI/2");
				} else if(nextBeta < 3*Mathf.PI/4 && nextBeta > - 3*Mathf.PI/4) {
					if(newDistance > 40) {
						Accel = 0;
					} else {
						Accel = 0;
						footbreak = 1f;
					}
					//UnityEngine.Debug.Log("Next beta more than PI/2");
				}
			}
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