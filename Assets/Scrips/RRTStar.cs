using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System;

public class Graph2
{
    public Node2 root;
    public Graph2(Vector3 pos)
    {
		List<Vector3> root_path = new List<Vector3>();
		root_path.Add(pos);
        this.root = new Node2(null, pos, 0, 0, Mathf.PI / 2, root_path);
    }
}

public class Node2
{
    public Node2 parent;
    public List<Node2> children;
    public Vector3 pos;
    public float cost;
    public float theta;
    public float velocity;
	public List<Vector3> node_path;
    public Node2(Node2 parent, Vector3 pos, float cost, float velocity, float theta, List<Vector3> node_path)
    {
        this.parent = parent;
        this.pos = pos;
        this.cost = cost;
        this.velocity = velocity;
        this.theta = theta;
        this.children = new List<Node2>();
		this.node_path = node_path;
    }
	    public Node2(Node2 parent, Vector3 pos, float cost, float velocity, float theta)
    {
        this.parent = parent;
        this.pos = pos;
        this.cost = cost;
        this.velocity = velocity;
        this.theta = theta;
        this.children = new List<Node2>();
    }
    public void addChild(Node2 node)
    {
        children.Add(node);
    }
    public void removeChild(Node2 node)
    {
        children.Remove(node);
    }
}

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
	[RequireComponent(typeof(DroneController))]
    public class RRTS
    {
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public Graph2 graph;
        Vector3 start_pos, goal_pos;
        public float x_min, x_max, z_min, z_max;
        int[,] x_free;
		Node2 node_best;
		public float maxSpeed;
		public float maxSteerAngle;
		public float t = 0.02f;
		public int maxIter = 40000;
		public int stepSize;
		public int nearRadius = 30;
		public List<Node2> near;
		public float carLength = 3f;
		public int buffer = 2;
		public System.Random r = new System.Random(DateTime.Now.Millisecond);
        public RRTS(GameObject terrain_manager_game_object, float m_maxSpeed, float m_MaximumSteerAngle)
        {
			stepSize = (int)(5/t);
			maxSpeed = m_maxSpeed;
			maxSteerAngle = m_MaximumSteerAngle * Mathf.PI/180;
			terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
			start_pos = terrain_manager.myInfo.start_pos;
			graph = new Graph2(start_pos);
			goal_pos = terrain_manager.myInfo.goal_pos;
			x_min = terrain_manager.myInfo.x_low;
			x_max = terrain_manager.myInfo.x_high;
			z_min = terrain_manager.myInfo.z_low;
			z_max = terrain_manager.myInfo.z_high;
			x_free = getXFree();
			//printXFree();
        }
        public int[,] getXFree()
        {
            float[,] x_int = terrain_manager.myInfo.traversability;
            int N = x_int.GetLength(0);
            int M = x_int.GetLength(1);
            float lenX = (x_max - x_min);
            float lenZ = (z_max - z_min);
            x_free = new int[(int)lenX, (int)lenZ];
            float n = lenX / N;
            float m = lenZ / M;
			int x_length = (int)(x_max-x_min);
			int z_length = (int)(z_max-z_min);
            for (int i = 0; i < lenX; i++)
            {
                for (int j = 0; j < lenZ; j++)
                {
                    int posX = (int)(i / n);
                    int posZ = (int)(j / m);
                    if (x_int[posX, posZ] == 0.0)
                    {
                        x_free[i, j] = 1;
                    }
                    else
                    {
                        x_free[i, j] = 0;
                        x_free[i, j] = 0;
					}
				}
			}			
			for (int i = 0; i < lenX; i++)
            {
                for (int j = 0; j < lenZ; j++)
                {
                    int posX = (int)(i / n);
                    int posZ = (int)(j / m);
                    if (x_int[posX, posZ] == 1.0)
                    {
                        x_free[i, j] = 0;
                        x_free[i, j] = 0;
			            int bufferXPlus = buffer;
                        int bufferXMinus = -buffer;
                        int bufferZPlus = buffer;
                        int bufferZMinus = -buffer;
                        if (bufferXMinus + i < 0) { bufferXMinus = 0; }
                        if (bufferXPlus + i > x_length - 1) { bufferXPlus = 0; }
                        if (bufferZMinus + j < 0) { bufferZMinus = 0; }
                        if (bufferZPlus + j > z_length - 1) { bufferZPlus = 0; }
                        for (int k = bufferXMinus; k <= bufferXPlus; k++)
                        {
                            for (int l = bufferZMinus; l <= bufferZPlus; l++)
                            {
                                x_free[k + i, l + j] = 0;
                            }
                        }
                    }
				}
			}
			for (int i = -5; i < 6; i++)
            {
                for (int j = -5; j < 6; j++)
                {
					x_free[(int)(start_pos[0]- x_min) + i -1, (int)(start_pos[2] - z_min) + j - 1] = 1;
				}
			}
            return x_free;
        }
        public void printXFree()
        {
            int N = x_free.GetLength(0);
            int M = x_free.GetLength(1);
			for(int i = 0; i < N; i++) {
				string s = i+x_min + ": ";
				for(int j = 0; j < M; j++) {
					s += j+z_min + " = " + x_free[i, j] + ", ";
				}
				UnityEngine.Debug.Log(s);
			}
		}
        public Vector3 SampleFree(int iter)
        {
			if(iter % 100 == 0) {
				int xPos = r.Next(-10, 10);
				int zPos = r.Next(-10, 10);
				Vector3 ret = new Vector3(goal_pos[0] + xPos, 0, goal_pos[2] + zPos);
				return ret;
			}
			else {
				int xStart = 0;
				int xStop = (int)(x_max - x_min-1);
				int zStart = 0;
				int zStop = (int)(z_max - z_min-1);
				bool notDone = true;
				int xInt = 0;
				int zInt = 0;
				while (notDone)
				{
					xInt = r.Next(xStart, xStop);
					zInt = r.Next(zStart, zStop);
					if (x_free[xInt, zInt] == 1)
					{
						notDone = false;
					}
				}
				Vector3 pos = new Vector3(xInt + x_min + 1, 0, zInt + z_min + 1);
				//UnityEngine.Debug.Log("Sampled position " + pos);
				return pos;
			}
		}
		public bool ObstacleFreeSteer(Vector3 pos) {
			int xCord = (int)(pos[0] - x_min - 1 );
			int zCord = (int)(pos[2] - z_min - 1);
			if(x_free[xCord, zCord] == 1) {
				return true;
			} else {
				return false;
			}
		}
		public bool ObstacleFree(Vector3 nearest, Vector3 pos)
        {
            float xDist = pos[0] - nearest[0];
            float zDist = pos[2] - nearest[2];
            float xAng = xDist + 0;
            float zAng = zDist + 0;
            if (zDist != 0)
            {
                xAng = xDist / Mathf.Abs(zDist);
            }
            if (xDist != 0)
            {
                zAng = zDist / Mathf.Abs(xDist);
            }
            if (Mathf.Abs(zAng) < Mathf.Abs(xAng))
            {
                xAng = 1 * Mathf.Sign(xDist);
            } else
            {
                zAng = 1 * Mathf.Sign(zDist);
            }
            bool obstacleFree = true;
            int length = (int)Mathf.Max(Mathf.Abs(zDist), Mathf.Abs(xDist));
			for(int i = 0; i < length; i++) {
				int xCord = (int)(nearest[0] - x_min - 1 + xAng*i);
				int zCord = (int)(nearest[2] - z_min - 1 + i*zAng);
				if(x_free[xCord, zCord] == 0) {
					obstacleFree = false;
					break;
				}
			}
            return obstacleFree;
        }

		private Node2 NearestRecursive(Node2 node, Vector3 pos, Node2 x_nearest, float minDistance)
		{
			float xDist = pos[0] - node.pos[0];
			float zDist = pos[2] - node.pos[2];
			float distance = Mathf.Sqrt(Mathf.Pow(xDist,2) + Mathf.Pow(zDist, 2));
			if(distance < minDistance) {
				x_nearest = node;
				minDistance = distance;
			}
			foreach (Node2 childNode in node.children) {
				x_nearest = NearestRecursive(childNode, pos, x_nearest, minDistance);
				xDist = pos[0] - x_nearest.pos[0];
				zDist = pos[2] - x_nearest.pos[2];
				minDistance = Mathf.Sqrt(Mathf.Pow(xDist,2) + Mathf.Pow(zDist, 2));
			} 
			return x_nearest;
		}
        private void NearRecursive(Node2 node, Node2 x_new, Node2 x_nearest, float minDistance)
        {
            Vector3 pos = x_new.pos;
            float xDist = pos[0] - node.pos[0];
            float zDist = pos[2] - node.pos[2];
            float distance = Mathf.Sqrt(Mathf.Pow(xDist, 2) + Mathf.Pow(zDist, 2));
			if(x_new == node) {
				//UnityEngine.Debug.Log("In near. Node is x_new");
			} else if(x_nearest == node) {
				//UnityEngine.Debug.Log("In near. Node is x_nearest");
			} else if (distance < minDistance) {
				//UnityEngine.Debug.Log("In near. adding node " + node.pos);
                near.Add(node);
            }
            foreach (Node2 childNode in node.children)
            {
                NearRecursive(childNode, x_new, x_nearest, minDistance);
            }
        }
        public float nfmod(float a, float b)
        {
            return a - b * Mathf.Floor(a / b);
        }
        private Node2 Steer(Node2 nearest, Vector3 pos)
        {
			float Accel = 1f;
			float v = nearest.velocity;
			float theta = nearest.theta;
			Vector3 newPos = nearest.pos;
			float cost = nearest.cost;
			List<Vector3> node_path = new List<Vector3>();
			node_path.Add(nearest.pos);
			for(int i = 1; i < stepSize+1; i++) {
				float xDist = pos[0] - newPos[0];
				float zDist = pos[2] - newPos[2];
				
				float goalDistance = Mathf.Pow(newPos[0] - goal_pos[0],2) + Mathf.Pow(newPos[2] - goal_pos[2], 2);
				if(goalDistance < 100) { return new Node2(null, newPos, cost, v, theta, node_path); }
				float newDistance = Mathf.Sqrt(Mathf.Pow(xDist, 2) + Mathf.Pow(zDist, 2));
				if(newDistance < 5) { return new Node2(null, newPos, cost, v, theta, node_path); }

                float vectorAngle = Mathf.Atan(zDist / xDist);
                if (
					(xDist < 0 && zDist > 0 && vectorAngle < 0) // if car_next lies in the second quadrant
                    || (xDist < 0 && vectorAngle == 0) // if car_next lies on the x axis and points to the left
                    || (xDist < 0 && zDist < 0)
				) // if car_next lies in the third quadrant
				{
                 vectorAngle += Mathf.PI; 
				}
                else if (xDist > 0 && zDist < 0)// if car_next lies in the fourth quadrant
                {
                    vectorAngle += 2 * Mathf.PI;
                }
                float beta = 0;
                if (vectorAngle < Mathf.PI / 2)
                {
                    //UnityEngine.Debug.Log("Vector in the first quadrant");
                    if (theta < Mathf.PI / 2)
                    {
                        //UnityEngine.Debug.Log("Theta in the first quadrant");
                        beta = vectorAngle - theta;
                    }
                    else if (theta < Mathf.PI)
                    {
                        //UnityEngine.Debug.Log("Theta in the second quadrant");
                        beta = vectorAngle - theta;
                    }
                    else if (theta < 3 * Mathf.PI / 2)
                    {
                        //UnityEngine.Debug.Log("Theta in the third quadrant");
                        beta = (Mathf.Sign(vectorAngle - Mathf.PI - theta));
                    }
                    else if (theta < 2 * Mathf.PI)
                    {
                        //UnityEngine.Debug.Log("Theta in the fourth quadrant");
                        beta = 2 * Mathf.PI + vectorAngle - theta;
                    }
                }
                else if (vectorAngle < Mathf.PI)
                {
                    //UnityEngine.Debug.Log("Vector in the second quadrant");
                    if (theta < Mathf.PI / 2)
                    {
                        //UnityEngine.Debug.Log("Theta in the first quadrant");
                        beta = vectorAngle - theta;
                    }
                    else if (theta < Mathf.PI)
                    {
                        //UnityEngine.Debug.Log("Theta in the second quadrant");
                        beta = vectorAngle - theta;
                    }
                    else if (theta < 3 * Mathf.PI / 2)
                    {
                        //UnityEngine.Debug.Log("Theta in the third quadrant");
                        beta = vectorAngle - theta;
                    }
                    else if (theta < 2 * Mathf.PI)
                    {
                        //UnityEngine.Debug.Log("Theta in the fourth quadrant");
                        beta = (Mathf.Sign(vectorAngle - Mathf.PI - theta));
                    }
                }
                else if (vectorAngle < 3 * Mathf.PI / 2)
                {
                    //UnityEngine.Debug.Log("Vector in the third quadrant");
                    if (theta < Mathf.PI / 2)
                    {
                        //UnityEngine.Debug.Log("Theta in the first quadrant");
                        beta = -(Mathf.Sign(vectorAngle - Mathf.PI - theta));
                    }
                    else if (theta < Mathf.PI)
                    {
                        //UnityEngine.Debug.Log("Theta in the second quadrant");
                        beta = vectorAngle - theta;
                    }
                    else if (theta < 3 * Mathf.PI / 2)
                    {
                        //UnityEngine.Debug.Log("Theta in the third quadrant");
                        beta = vectorAngle - theta;
                    }
                    else if (theta < 2 * Mathf.PI)
                    {
                        //UnityEngine.Debug.Log("Theta in the fourth quadrant");
                        beta = vectorAngle - theta;
                    }
                }
                else if (vectorAngle < 2 * Mathf.PI)
                {
                    //UnityEngine.Debug.Log("Vector in the fourth quadrant");
                    if (theta < Mathf.PI / 2)
                    {
                        //UnityEngine.Debug.Log("Theta in the first quadrant");
                        beta = -2 * Mathf.PI - vectorAngle - theta;
                    }
                    else if (theta < Mathf.PI)
                    {
                        //UnityEngine.Debug.Log("Theta in the second quadrant");
                        beta = -(Mathf.Sign(vectorAngle - Mathf.PI - theta));
                    }
                    else if (theta < 3 * Mathf.PI / 2)
                    {
                        //UnityEngine.Debug.Log("Theta in the third quadrant");
                        beta = vectorAngle - theta;
                    }
                    else if (theta < 2 * Mathf.PI)
                    {
                        //UnityEngine.Debug.Log("Theta in the fourth quadrant");
                        beta = vectorAngle - theta;
                    }
                }
                else
                {
                    //UnityEngine.Debug.Log("None of the above. Should not happen.");
                    beta = vectorAngle - theta;
                }
				if(beta < Mathf.PI/4 && beta > - Mathf.PI/4) {
					Accel = 1f;
				} else if(beta < Mathf.PI/2 && beta > - Mathf.PI/2) {
					Accel = 0.5f;
					if(v > 30) {
						v = 30;
					}
				} else if(beta < 3*Mathf.PI/4 && beta > - 3*Mathf.PI/4) {
					if(v > 20) {
						v = 20;
					}
					Accel = 0f;
				} else if(beta < Mathf.PI && beta > - Mathf.PI) {
					Accel = 0f;
					if(v > 20) {
						v = 20;
					}
				} else if(beta < 2*Mathf.PI && beta > - 2*Mathf.PI) {
					Accel = 0f;
					if(v > 20) {
						v = 20;
					}
				}
				if( v < 10) {
					Accel = 1f;
				}
				v += t*Accel;
				if(beta < -maxSteerAngle) {
					beta = -maxSteerAngle;
				} else if(beta > maxSteerAngle) {
					beta = maxSteerAngle;
				}
				float tanPsi = Mathf.Tan(beta);
				
				theta += t * v / carLength * tanPsi;
				theta = nfmod(theta, 2*Mathf.PI);

				cost++;
				float x = newPos[0] + t*v * Mathf.Cos(theta);
				float z = newPos[2] + t*v * Mathf.Sin(theta);
				Vector3 updatePos = new Vector3(x, 0, z);

				if(!ObstacleFreeSteer(updatePos)) {
					return null;
				}
				newPos = updatePos;
				node_path.Add(newPos);
            }
            Node2 newNode = new Node2(null, newPos, cost, v, theta, node_path);
            return newNode;
        }
        public List<Node2> Run()
        {
            int iter = 0;
            bool notConverged = true;
			while(notConverged) {
				iter++;
				Vector3 x_rand = SampleFree(iter);

				Node2 x_nearest = NearestRecursive(graph.root, x_rand, null, float.MaxValue);
				Node2 x_new = Steer(x_nearest, x_rand);
				//Vector3 dir = new Vector3(1, 1, 1);
				//UnityEngine.Debug.DrawRay(x_rand, dir, Color.black, 100f);
				if(x_new != null) {

					/*UnityEngine.Debug.Log("Obstacle Free");
					UnityEngine.Debug.Log("x_new pos " + x_new.pos);
					near = new List<Node2>();
					NearRecursive(graph.root, x_new, x_nearest, nearRadius);
					foreach (Node2 nearNode in near) {
						if(ObstacleFree(nearNode.pos, x_new.pos)) {
							Node2 x_temp = Steer(nearNode, x_new.pos);
							if(x_temp != null) {
								if(x_temp.cost < x_new.cost) {
									x_nearest = nearNode;
									x_new = x_temp;
								}
							}
						}
					}
					*/
					x_new.parent = x_nearest;
					x_nearest.addChild(x_new);
					/*
					foreach (Node2 nearNode in near) {
						if(nearNode == x_new || nearNode == x_nearest || nearNode == graph.root) {
						} else if(ObstacleFree(x_new.pos, nearNode.pos)){
							Node2 x_temp = Steer(x_new, nearNode.pos);
							if(x_temp!= null) {
								if(x_temp.cost < nearNode.cost) {
									nearNode.parent.removeChild(nearNode);
									nearNode.parent = x_new;
									x_new.addChild(nearNode);
								}
							}
						}
					}*/
					float distance = Mathf.Pow(x_new.pos[0] - goal_pos[0],2) + Mathf.Pow(x_new.pos[2] - goal_pos[2], 2);
					if(distance < 100) {
						notConverged = false;
						UnityEngine.Debug.Log("------------------------Converged on Iteration: " + iter + " ------------------------");
						node_best = x_new;
					}
				} else {
					//UnityEngine.Debug.Log("Not Obstacle Free :(");
				}
				if(iter % maxIter == 0) {
					if(node_best == null) {
						Node2 near = NearestRecursive(graph.root, goal_pos,  null, float.MaxValue);
						node_best = near;
					}
					notConverged = false;
					UnityEngine.Debug.Log("------------------------Stopped on Iteration: " + iter + " ------------------------");
				}
			}
			List<Vector3> myPath = new List<Vector3>();
        	List<Node2> nodePath = new List<Node2>();
			Node2 node = node_best;
			myPath.Add(node.pos);
			nodePath.Add(node);
			while(node.parent != null) {
				if(node.pos != node.parent.pos) {
					node = node.parent;
					myPath.Insert(0, node.pos);
					nodePath.Insert(0, node);
				} else {
					node = node.parent;
				}

			}			
			
			//drawAllPaths(graph.root);
			drawPath(nodePath);
			UnityEngine.Debug.Log("Number of nodes in path " + nodePath.Count);
			return nodePath;
        }

		private void drawPath(List<Node2> nodePath) {
			for(int i = 1; i < nodePath.Count; i++) {
				UnityEngine.Debug.Log("point " + i + " pos: " + nodePath[i].pos);
				UnityEngine.Debug.DrawLine(nodePath[i-1].pos, nodePath[i].pos, Color.blue, 100f);
				/*for(int j = 1; j < nodePath[i].node_path.Count; j++) {
					UnityEngine.Debug.DrawLine(nodePath[i].node_path[j-1], nodePath[i].node_path[j], Color.yellow, 100f);
				}*/
			}
		}
		private void drawAllPaths(Node2 node) {
			foreach(Node2 childNode in node.children) {
				UnityEngine.Debug.DrawLine(node.pos, childNode.pos, Color.red, 100f);
				/*for(int j = 1; j < node.node_path.Count; j++) {
					UnityEngine.Debug.DrawLine(node.node_path[j-1], node.node_path[j], Color.red, 100f);
				}*/
				drawAllPaths(childNode);
			}
		}
    }
}