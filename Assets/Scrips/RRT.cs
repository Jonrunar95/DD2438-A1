using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Node
{
    public Node parent;
    public List<Node> children;
    public Vector3 pos;
    public float cost;
    public Node(Node parent, Vector3 pos, float cost)
    {
        this.parent = parent;
        this.pos = pos;
        this.cost = cost;
        this.children = new List<Node>();
    }
    public Node getParent()
    {
        return parent;
    }
    public List<Node> getChildren(int id)
    {
        return children;
    }
    public void addChild(Node node)
    {
        children.Add(node);
    }
}
public class Graph
{
    public Node root;
    public Graph(Vector3 pos)
    {
        this.root = new Node(null, pos, 0);
    }
}

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class RRT
    {
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        List<Vector3> my_path;
        public Graph graph;
        Vector3 start_pos, goal_pos;
        public float x_min, x_max, z_min, z_max;
        int[,] x_free;
		//Node x_nearest;
		Node x_last;
		//float minDistance;
        public RRT(GameObject terrain_manager_game_object)
        {
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            start_pos = terrain_manager.myInfo.start_pos;
			graph = new Graph(start_pos);
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
			if(iter % 10 == 0) {
				return goal_pos;
			} else {
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
        public bool ObstacleFree(Node nearest, Vector3 pos)
        {
			float xDist = pos[0] - nearest.pos[0];
			float zDist = pos[2] - nearest.pos[2];
			UnityEngine.Debug.Log("X dist " + xDist + " Z dist " + zDist);
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
			UnityEngine.Debug.Log("X Angle " + xAng);
			bool obstacleFree = true;
			if(zDist < 0) {
				zAng= -1;
			} else {
				zAng = 1;
			}
			UnityEngine.Debug.Log("Z Angle " + zAng);
			int length = (int)Mathf.Max(Mathf.Abs(zDist), Mathf.Abs(xDist));

			for(int i = 0; i < length; i++) {
				int xCord = (int)(nearest.pos[0] - x_min - 1 + xAng*i);
				int zCord = (int)(nearest.pos[2] - z_min - 1 + i*zAng);
				UnityEngine.Debug.Log("X cord " + (xCord+x_min) + " Z cord " + (zCord+z_min));
				if(x_free[xCord, zCord] == 0) {
					UnityEngine.Debug.Log("Obstacle");
					obstacleFree = false;
					break;
				}
			}
            return obstacleFree;
        }
        private Node Steer(Node nearest, Vector3 pos)
        {

            return null;
        }

		private Node NearestRecursive(Node node, Vector3 pos, Node x_nearest, float minDistance)
		{
			float xDist = pos[0] - node.pos[0];
			float zDist = pos[2] - node.pos[0];
			float distance = Mathf.Sqrt(Mathf.Pow(xDist,2) + Mathf.Pow(zDist, 2));
			//UnityEngine.Debug.Log("Point: " + node.pos[0] + ", " + node.pos[2]);
			//UnityEngine.Debug.Log("Min Dist; " + minDistance + "New Dist: " + distance);
			if(distance < minDistance) {
				x_nearest = node;
				minDistance = distance;
			}
			//UnityEngine.Debug.Log("New Min Dist: " + minDistance);
			//UnityEngine.Debug.Log("Number of children: " + node.children.Count);
			int i = 0;
			foreach (Node childNode in node.children) {
				//UnityEngine.Debug.Log("Child " + i + " children: " + childNode.children.Count);
				x_nearest = NearestRecursive(childNode, pos, x_nearest, minDistance);
				xDist = pos[0] - x_nearest.pos[0];
				zDist = pos[2] - x_nearest.pos[0];
				minDistance = Mathf.Sqrt(Mathf.Pow(xDist,2) + Mathf.Pow(zDist, 2));
			} 
			return x_nearest;
		}
		/*
		private void NearestRecursive(Node node, Vector3 pos)
		{
			float distance = Mathf.Sqrt(Mathf.Pow(pos[0]*node.pos[0],2) + Mathf.Pow(pos[2]*node.pos[2], 2));
			UnityEngine.Debug.Log("Min Dist; " + minDistance + "New Dist: " + distance);
			if(distance < minDistance) {
				x_nearest = node;
				minDistance = distance;
			}
			UnityEngine.Debug.Log("New Min Dist: " + minDistance);
			UnityEngine.Debug.Log("Number of children: " + node.children.Count);
			if(node.children.Count != 0) {
				foreach (Node childNode in node.children) {
					UnityEngine.Debug.Log("Child: " +childNode.children.Count);
					NearestRecursive(childNode, pos);
				} 
			}
			return;
		}
		
        private Node Nearest(Graph graph, Vector3 pos)
        {
			Node nearest = graph.root;
			float minDistance = float.MaxValue;
			foreach (Node node in graph.Vertices) {
				float distance = Mathf.Sqrt(Mathf.Pow(pos[0]*node.pos[0],2) + Mathf.Pow(pos[2]*node.pos[2], 2));
				if(distance < minDistance) {
					nearest = node;
					minDistance = distance;
				}
			}
			return nearest;
        }
		*/
		/*
        private List<Node> Near()
        {
			List<Node> near = new List<Node>();
			foreach (Node node in graph.Vertices) {
				float distance = Mathf.Pow(near.pos[0]*node.pos[0] + near.pos[2]*node.pos[2], 2);
				if(distance < nearLength) {
					near.Add(node);
				}
			}
			return near;
        }
		*/
        public List<Vector3> Run()
        {
			UnityEngine.Debug.Log("Starting position       X: " + start_pos[0] + " Z: " + start_pos[2]);
			int iter = 0;
			bool notConverged = true;

			while(notConverged) {
				iter++;
				UnityEngine.Debug.Log("-----------------------------------------------------------------------------------------------------------------------------Iteration:" + iter);
				Vector3 x_rand = SampleFree(iter);
				UnityEngine.Debug.Log("Sampled point        X: " + x_rand[0] + " Z: " + x_rand[2]);

				Node x_nearest = NearestRecursive(graph.root, x_rand, null, float.MaxValue);
				
				UnityEngine.Debug.Log("Node nearest         X: " + x_nearest.pos[0] + " Z: " + x_nearest.pos[2]);
				//Node x_new = Steer(x_nearest, x_rand);
				
				if(ObstacleFree(x_nearest, x_rand)) { // replace x_rand with x_new when steer is ready
					UnityEngine.Debug.Log("Obstacle Free");	

					Node x_new = new Node(x_nearest, x_rand, 0);
					x_nearest.addChild(x_new);
					
					//if(x_new.pos[0] == goal_pos[0] && x_new.pos[2] == goal_pos[2]) {
					if(x_rand[0] == goal_pos[0] && x_rand[2] == goal_pos[2]) {
						UnityEngine.Debug.Log("CONVERGED");
						notConverged = false;
					}
					x_last = x_new;
				}
				else {
					UnityEngine.Debug.Log("Not Obstacle Free");
				}
			}
			drawPaths(graph.root);

			//List<Vector3> myPath = testPath();

			List<Vector3> myPath = new List<Vector3>();

			Node node = x_last;
			//UnityEngine.Debug.Log("Node  pos " + node.pos);
			myPath.Add(node.pos);
			//UnityEngine.Debug.Log("Node parents pos " + node.parent.pos);
			while(node.parent != null) {
				//UnityEngine.Debug.Log("Creating path");
				node = node.parent;
				myPath.Insert(0, node.pos);
			}
			printXFree();
			for(int i = 0; i < myPath.Count; i++) {
				//UnityEngine.Debug.Log("point " + i + " = " + myPath[i][0] + " " + myPath[i][2]);
			}
			return myPath;
        }

		private void drawPaths(Node node)
		{
			foreach(Node childNode in node.children) {
				UnityEngine.Debug.DrawLine(childNode.pos, node.pos, Color.red, 100f);
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