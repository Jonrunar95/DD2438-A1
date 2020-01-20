using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Node
{
    public Node parent;
    public List<Node> children;
    public float pos;
    public float cost;
    public Node(Node parent, float pos, float cost)
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
    public List<Node> Vertices;
    public Graph(float pos)
    {
        this.root = new Node(null, pos, 0);
        Vertices = new List<Node>();
    }
}
namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class RRT
    {
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        List<Vector3> my_path = new List<Vector3>();
        public Graph graph;
        Vector3 start_pos, goal_pos;
        public float x_min, x_max, z_min, z_max;
        float[,,] x_free;
        public RRT(GameObject terrain_manager_game_object)
        {
            this.graph = new Graph(0);
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            /*
            x_min = terrain_manager.myInfo.x_min;
            x_max = terrain_manager.myInfo.x_max;
            z_min = terrain_manager.myInfo.z_min;
            z_max = terrain_manager.myInfo.z_max;
            */
            x_min = 0;
            x_max = 0;
            z_min = 0;
            z_max = 0;
            x_free = getXFree();
        }
        public float[,,] getXFree()
        {
            float[,] x_int = terrain_manager.myInfo.traversability;
            int N = x_int.GetLength(0);
            int M = x_int.GetLength(1);
            int lenX = (int)(x_max - x_min);
            int lenZ = (int)(z_max - z_min);
            x_free = new float[lenX, lenZ, 2];
            int n = lenX / N;
            int m = lenZ / M;
            int buffer = 5;
            int length = 0;
            for (int i = 0; i < length; i++)
            {
                for (int j = 0; j < length; j++)
                {
                    int posX = i / n;
                    int posZ = j / m;
                    if (x_int[posX, posZ] == 1)
                    {
                        x_free[i, j, 0] = i + x_min;
                        x_free[i, j, 1] = j + z_min;
                    }
                    else
                    {
                        for (int k = -buffer; k < buffer + 1; k++)
                        {
                            for (int l = -buffer; l < buffer + 1; l++)
                            {
                                x_free[k, l, 0] = 0;
                                x_free[k, l, 1] = 0;
                            }
                        }
                    }
                }
            }
            return x_free;
        }
        public float SampleFree()
        {
            System.Random r = new System.Random();
            bool notDone = true;
            int xInt = 0;
            int zInt = 0;
            while (notDone)
            {
                xInt = r.Next(0, (int)(x_max - x_min));
                zInt = r.Next(0, (int)(z_max - z_min));
                if (x_free[xInt, zInt, 0] != 0)
                {
                    notDone = false;
                }
            }
            return x_free[xInt, zInt, 2];
        }
        private bool ObstacleFree(Node x_nearest, Node x_new)
        {
            return false;
        }
        private Node Steer(Node x_nearest, float x)
        {
            return null;
        }
        private Node Nearest(Graph graph, float x)
        {
            return null;
        }
        private void Near()
        {
        }
        private void run()
        {
            float x_rand = SampleFree();
            Node x_nearest = Nearest(graph, 0);
            Node x_new = Steer(x_nearest, 0);
            if (ObstacleFree(x_nearest, x_new))
            {
                x_nearest.addChild(x_new);
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