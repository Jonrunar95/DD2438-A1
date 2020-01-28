using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Graph2
{
    public Node2 root;
    public Graph2(Vector3 pos)
    {
        this.root = new Node2(null, pos, 0, 0, Mathf.PI/2);
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
	public Node2(Node2 parent, Vector3 pos, float cost, float velocity, float theta) {
		this.parent = parent;
        this.pos = pos;
        this.cost = cost;
		this.velocity = velocity;
		this.theta = theta;
        this.children = new List<Node2>();
	}
    public Node2 getParent()
    {
        return parent;
    }
    public List<Node2> getChildren(int id)
    {
        return children;
    }
    public void addChild(Node2 node)
    {
        children.Add(node);
    }
}

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class RRTS
    {
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public Graph2 graph;
        Vector3 start_pos, goal_pos;
        public float x_min, x_max, z_min, z_max;
        int[,] x_free;
		//Node2 x_nearest;
		Node2 x_last;
		//float minDistance;
        public RRTS(GameObject terrain_manager_game_object)
        {
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            start_pos = terrain_manager.myInfo.start_pos;
			graph = new Graph2(start_pos);
            goal_pos = terrain_manager.myInfo.goal_pos;
            
            x_min = terrain_manager.myInfo.x_low;
            x_max = terrain_manager.myInfo.x_high;
            z_min = terrain_manager.myInfo.z_low;
            z_max = terrain_manager.myInfo.z_high;
            x_free = getXFree();
        }
        public int[,] getXFree()
        {
            float[,] x_int = terrain_manager.myInfo.traversability;
            int N = x_int.GetLength(0);
            int M = x_int.GetLength(1);
            int lenX = (int)(x_max - x_min);
            int lenZ = (int)(z_max - z_min);
            x_free = new int[lenX, lenZ];
            int n = lenX / N;
            int m = lenZ / M;
            int buffer = 3;
			int x_length = (int)(x_max-x_min);
			int z_length = (int)(z_max-z_min);
            for (int i = 0; i < lenX; i++)
            {
                for (int j = 0; j < lenZ; j++)
                {
                    int posX = i / n;
                    int posZ = j / m;
                    if (x_int[posX, posZ] == 0.0)
                    {
                        x_free[i, j] = 1;
                        x_free[i, j] = 1;
                    }
                    else
                    {
						x_free[i, j] = 0;
                        x_free[i, j] = 0;
						int bufferXPlus = buffer;
						int bufferXMinus = -buffer;
						int bufferZPlus = buffer;
						int bufferZMinus = -buffer;
						if(bufferXMinus+i < 0) { bufferXMinus = 0; }
						if(bufferXPlus+i > x_length-1) { bufferXPlus = 0; }
						if(bufferZMinus+j < 0) { bufferZMinus = 0; }
						if(bufferZPlus+j > z_length-1) { bufferZPlus = 0; }
                        for (int k = bufferXMinus; k <= bufferXPlus; k++)
                        {
                            for (int l = bufferZMinus; l <= bufferZPlus; l++)
                            {
                                x_free[k+i, l+j] = 0;
                            }
                        }
                    }
                }
            }
            return x_free;
        }
		public void printXFree() {
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
			/*
			if(iter == 1) {
				//Vector3 newPos = new Vector3(200, 0, 167); //first quadrant
				Vector3 newPos = new Vector3(165, 0, 167); //second quadrant
				//Vector3 newPos = new Vector3(155, 0, 107); //third quadrant
				//Vector3 newPos = new Vector3(200, 0, 107); //fourth quadrant
				return newPos;
			}
			else if(iter  == 2) {
				//Vector3 newPos = new Vector3(200, 0, 167); //first quadrant
				Vector3 newPos = new Vector3(150, 0, 162); //second quadrant
				//Vector3 newPos = new Vector3(15, 0, 107); //third quadrant
				//Vector3 newPos = new Vector3(200, 0, 107); //fourth quadrant
				return newPos;
			} 
			else if(iter == 3) {
				//Vector3 newPos = new Vector3(200, 0, 167); //first quadrant
				Vector3 newPos = new Vector3(120, 0, 162); //second quadrant
				//Vector3 newPos = new Vector3(15, 0, 107); //third quadrant
				//Vector3 newPos = new Vector3(200, 0, 107); //fourth quadrant
				return newPos;
			} else if(iter  == 4) {
				//Vector3 newPos = new Vector3(200, 0, 167); //first quadrant
				Vector3 newPos = new Vector3(124, 0, 134); //second quadrant
				//Vector3 newPos = new Vector3(15, 0, 107); //third quadrant
				//Vector3 newPos = new Vector3(200, 0, 107); //fourth quadrant
				return newPos;
			}
			*/
			if(iter % 100 == 0) {
				return goal_pos;
			}
			else {
				System.Random r = new System.Random();
				bool notDone = true;
				int xInt = 0;
				int zInt = 0;
				while (notDone)
				{
					xInt = r.Next(0, (int)(x_max - x_min-1));
					zInt = r.Next(0, (int)(z_max - z_min-1));
					if (x_free[xInt, zInt] == 1)
					{
						notDone = false;
					}
				}
				Vector3 pos = new Vector3(xInt+x_min+1, 0, zInt+z_min+1);
				return pos;
			}

        }
        /*public bool ObstacleFree(Node2 nearest, Vector3 pos)
        {
			printXFree();
			float xDist = pos[0] - nearest.pos[0];
			float zDist = pos[2] - nearest.pos[2];
			float xAng= 1;
			float zAng = 1;

			if(zDist != 0) {
				xAng = xDist/Mathf.Abs(zDist);
			}
			if(xDist != 0) {
				zAng = zDist/Mathf.Abs(xDist);
			}
			float zSign, xSign = 0;
			if(xAng < 0) {
				xSign = -1;
			} else {
				xSign = 1;
			}
			if(zAng < 0) {
				zSign = -1;
			} else {
				zSign = 1;
			}
			if(Mathf.Abs(zAng) < Mathf.Abs(xAng)) {
				xAng = 1*xSign;
			} else {
				zAng = 1*zSign;
			}
			bool obstacleFree = true;
			if(zDist < 0) {
				zAng= -1;
			} else {
				zAng = 1;
			}
			int length = (int)Mathf.Max(Mathf.Abs(zDist), Mathf.Abs(xDist));
			UnityEngine.Debug.Log("Checking if obstacle free");
			UnityEngine.Debug.Log("Postitions = " + nearest.pos + pos);
			for(int i = 0; i < length; i++) {
				int xCord = (int)(nearest.pos[0] - x_min - 1 + xAng*i);
				int zCord = (int)(nearest.pos[2] - z_min - 1 + i*zAng);
				UnityEngine.Debug.Log("xCord = " + xCord + " zCord " + zCord);
				if(x_free[xCord, zCord] == 0) {
					UnityEngine.Debug.Log("Obstalce :(");
					obstacleFree = false;
					break;
				}
			}
            return obstacleFree;
        }*/

		public bool ObstacleFree(Vector3 pos) {
			int xCord = (int)(pos[0] - x_min- 1 );
			int zCord = (int)(pos[2] - z_min - 1);
			if(x_free[xCord, zCord] == 1) {
				return true;
			} else {
				return false;
			}
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
		/*
        private List<Node2> Near()
        {
			List<Node2> near = new List<Node2>();
			foreach (Node2 node in graph.Vertices) {
				float distance = Mathf.Pow(near.pos[0]*node.pos[0] + near.pos[2]*node.pos[2], 2);
				if(distance < nearLength) {
					near.Add(node);
				}
			}
			return near;
        }
		*/
		private Node2 Steer(Node2 nearest, Vector3 pos)
        {
			//UnityEngine.Debug.DrawLine(nearest.pos, pos, Color.yellow, 100f);
			float L = 3.0f;
			float v = 0.3f;
			//float theta = nearest.theta;
			float theta = nearest.theta;
			Vector3 newPos = nearest.pos;
			//UnityEngine.Debug.Log("Beginning Pos: X = " + newPos[0] + " Z = " + newPos[2]);
			//UnityEngine.Debug.Log("Goal Pos: X = " + pos[0] + " Z = " + pos[2]);
			//UnityEngine.Debug.Log("Beginning theta: " + theta);
			float oldDistance = Mathf.Sqrt(Mathf.Pow(pos[0] - newPos[0],2) + Mathf.Pow(pos[2] - newPos[2], 2));
			for(int i = 1; i < 100; i++) {
				float xDist = pos[0] - newPos[0];
				float zDist = pos[2] - newPos[2];
				float newDistance = Mathf.Sqrt(Mathf.Pow(xDist,2) + Mathf.Pow(zDist, 2));
				oldDistance = newDistance;
				float goalDistance = Mathf.Sqrt(Mathf.Pow(newPos[0] - goal_pos[0],2) + Mathf.Pow(newPos[2] - goal_pos[2], 2));
				//UnityEngine.Debug.Log("New distance " + newDistance);
				//UnityEngine.Debug.Log("Distance to goal " + goalDistance);
				if(newDistance < 1) {
					return new Node2(nearest, newPos, 0, 0, theta);
				}
				if(goalDistance < 0.1) {
					return new Node2(nearest, newPos, 0, 0, theta);
				}
				float vectorAngle = (float)Math.Atan(zDist / xDist);
				/*  TODO:
						If on x-axis
						If on y-axis
						Check if all quadrants work
				*/
				if ((xDist < 0 && zDist > 0) || ( xDist < 0 && zDist < 0)){// if car_next lies in the second or thrid quadrant
                    vectorAngle += Mathf.PI;
                }
				//UnityEngine.Debug.Log("xDist = " + xDist + ", zDist = " + zDist + ", Angle = " + vectorAngle + " Theta = " + theta);
				if(
					(theta > vectorAngle) && ((xDist < 0 && zDist > 0) || ( xDist > 0 && zDist < 0))
					|| (theta < vectorAngle) && ((xDist < 0 && zDist < 0) || ( xDist > 0 && zDist > 0))
				) {
					//UnityEngine.Debug.Log("Plus");
					theta += v/L*Mathf.Tan(vectorAngle);
				} else {
					//UnityEngine.Debug.Log("Minus");
					theta -= v/L*Mathf.Tan(vectorAngle);
				}
				//UnityEngine.Debug.Log("Updated theta " + theta);

				newPos[0] += v * Mathf.Cos(theta);
            	newPos[2] += v * Mathf.Sin(theta);
				if(!ObstacleFree(newPos)) {
					//UnityEngine.Debug.Log("Obstacle at: " + newPos);
					return null;
				}
			}
			//UnityEngine.Debug.Log("New node pos = [" + newPos[0] +", " + newPos[2] + "]");
			Node2 newNode = new Node2(nearest, newPos, 0, 0, theta);
            return newNode;
        }
        public List<Vector3> Run()
        {
			int iter = 0;
			bool notConverged = true;

			while(notConverged) {
				iter++;
				//UnityEngine.Debug.Log("------------------------Iteration: " + iter + " ------------------------");
				Vector3 x_rand = SampleFree(iter);

				Node2 x_nearest = NearestRecursive(graph.root, x_rand, null, float.MaxValue);
				
				Node2 x_new = Steer(x_nearest, x_rand);
				
				if(x_new != null) {
					//UnityEngine.Debug.Log("Obstacle Free");
					x_nearest.addChild(x_new);
					float distance = Mathf.Sqrt(Mathf.Pow(x_new.pos[0] - goal_pos[0],2) + Mathf.Pow(x_new.pos[2] - goal_pos[2], 2));
					if(distance < 1) {
						UnityEngine.Debug.Log("Converged in " + iter + " iterations");
						notConverged = false;
					}
					x_last = x_new;
				} else {
					//UnityEngine.Debug.Log("Not Obstacle Free");
				}
				if(iter % 5000 == 0) { notConverged = false; }
			}
			

			//List<Vector3> myPath = testPath();

			
			//drawPaths(graph.root);

			List<Vector3> myPath = new List<Vector3>();
			List<Node2> nodePath = new List<Node2>();

			Node2 node = x_last;
			myPath.Add(node.pos);
			nodePath.Add(node);
			while(node.parent != null) {
				node = node.parent;
				myPath.Insert(0, node.pos);
				nodePath.Insert(0, node);
			}
			driveThroughPoints(nodePath);
			return myPath;
        }

		public void drive(Node2 start, Node2 finish) {
			Node2 nearest = start;
			Vector3 pos = finish.pos;
			float L = 3.0f;
			float v = 0.3f;
			//float theta = nearest.theta;
			float theta = nearest.theta;
			Vector3 newPos = nearest.pos;
			for(int i = 1; i < 100; i++) {
				float xDist = pos[0] - newPos[0];
				float zDist = pos[2] - newPos[2];
				float newDistance = Mathf.Sqrt(Mathf.Pow(xDist,2) + Mathf.Pow(zDist, 2));
				float goalDistance = Mathf.Sqrt(Mathf.Pow(newPos[0] - goal_pos[0],2) + Mathf.Pow(newPos[2] - goal_pos[2], 2));
								
				if(newDistance < 1) {
					return;
				}
				if(goalDistance < 0.1) {
					return;
				}
				float vectorAngle = (float)Math.Atan(zDist / xDist);
				/*  TODO:
						If on x-axis
						If on y-axis
						Check if all quadrants work
				*/
				if ((xDist < 0 && zDist > 0) || ( xDist < 0 && zDist < 0)){// if car_next lies in the second quadrant
					vectorAngle += Mathf.PI;
				}

				if(
					(theta > vectorAngle) && ((xDist < 0 && zDist > 0) || ( xDist > 0 && zDist < 0))
					|| (theta < vectorAngle) && ((xDist < 0 && zDist < 0) || ( xDist > 0 && zDist > 0))
					) {
					theta += v/L*Mathf.Tan(vectorAngle);
				} else{
					theta -= v/L*Mathf.Tan(vectorAngle);
				}

				float x = newPos[0] + v * (float)Math.Cos(theta);
				float z = newPos[2] + v * (float)Math.Sin(theta);
				Vector3 updatedPos = new Vector3(x, 0, z);
				UnityEngine.Debug.DrawLine(newPos, updatedPos, Color.blue, 100f);
				newPos[0] += v * (float)Math.Cos(theta);
				newPos[2] += v * (float)Math.Sin(theta);
			}
		}

		private void driveThroughPoints(List<Node2> myPath) {
			for(int i = 1; i < myPath.Count; i++) {
				drive(myPath[i-1], myPath[i]);
			}
		}

		private void drawPaths(Node2 node)
		{
			foreach(Node2 childNode in node.children) {
				UnityEngine.Debug.DrawLine(childNode.pos, node.pos, Color.yellow, 100f);
                drawPaths(childNode);
			}
		}
        public List<Vector3> testPath()
        {
            List<Vector3> list = new List<Vector3>();
            list.Add(terrain_manager.myInfo.start_pos);
            list.Add(new Vector3(180f, 0f, 180f));
            list.Add(new Vector3(110f, 0f, 180f));
            list.Add(terrain_manager.myInfo.goal_pos);

            return list;
        }
    }
}