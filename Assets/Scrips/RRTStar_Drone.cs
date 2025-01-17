using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System;


public class RRTSDrone
{
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    public Graph2 graph;
    Vector3 start_pos, goal_pos;
    public float x_min, x_max, z_min, z_max;
    int[,] x_free;
    //Node2 x_nearest;
    Node2 x_best;
    //float minDistance;
    public int stepSize = 5;
    public float maxSpeed;
    public float maxSteerAngle;
    private const float max_steer_angle = (25f / 360f) * 2f * Mathf.PI;
    public RRTSDrone(GameObject terrain_manager_game_object)
    {
        maxSpeed = 200;
        maxSteerAngle = 25 * Mathf.PI / 180;
        UnityEngine.Debug.Log(max_steer_angle + " " + maxSteerAngle);
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
        int buffer = 2;
        int x_length = (int)(x_max - x_min);
        int z_length = (int)(z_max - z_min);
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
    public void printXFree()
    {
        int N = x_free.GetLength(0);
        int M = x_free.GetLength(1);
        for (int i = 0; i < N; i++)
        {
            string s = i + x_min + ": ";
            for (int j = 0; j < M; j++)
            {
                s += j + z_min + " = " + x_free[i, j] + ", ";
            }
            //UnityEngine.Debug.Log(s);
        }
    }
    public Vector3 SampleFree(int iter)
    { /*
			if(iter == 1) {
				Vector3 newPos = new Vector3(195, 0, 137);
				return newPos;
			}
			else if(iter  == 2) {
				Vector3 newPos = new Vector3(195, 0, 157);
				return newPos;
			} 
			else if(iter == 3) {
				Vector3 newPos = new Vector3(185, 0, 157);
				return newPos;
			} else if(iter  == 4) {
				Vector3 newPos = new Vector3(175, 0, 157);
				return newPos;
			}  else if(iter == 5) {
				Vector3 newPos = new Vector3(175, 0, 137);
				return newPos;
			} else if(iter == 6) {
				Vector3 newPos = new Vector3(175, 0, 117);
				return newPos;
			} else if(iter == 7) {
				Vector3 newPos = new Vector3(185, 0, 117);
				return newPos;
			} else if(iter == 8) {
				Vector3 newPos = new Vector3(195, 0, 117);
				return newPos;
			} else if(iter == 9) {
				Vector3 newPos = new Vector3(110, 0, 135);
				return newPos;
			}*/

        if (iter % 10 == 0)
        {
            return goal_pos;
        }
        else
        {
            int xStart, xStop, zStart, zStop;
            /*if(iter < 50){
                xStart = (int)Math.Max(start_pos[0] -x_min - 20, 0);
                xStop = (int)Math.Min(start_pos[0] -x_min + 20, (x_max - x_min-1));
                zStart = (int)Math.Max(start_pos[2] - z_min - 20, 0);
                zStop = (int)Math.Min(start_pos[2] - z_min + 20, (int)(z_max - z_min-1));
            } else if(iter < 100) {
                xStart = (int)Math.Max(start_pos[0] -x_min - 40, 0);
                xStop = (int)Math.Min(start_pos[0] -x_min + 40, (x_max - x_min-1));
                zStart = (int)Math.Max(start_pos[2] - z_min - 40, 0);
                zStop = (int)Math.Min(start_pos[2] - z_min + 40, (int)(z_max - z_min-1));
            } else if(iter < 150) {
                xStart = (int)Math.Max(start_pos[0] -x_min - 60, 0);
                xStop = (int)Math.Min(start_pos[0] -x_min + 60, (x_max - x_min-1));
                zStart = (int)Math.Max(start_pos[2] - z_min - 60, 0);
                zStop = (int)Math.Min(start_pos[2] - z_min + 60, (int)(z_max - z_min-1));
            } else if(iter < 200) {
                xStart = (int)Math.Max(start_pos[0] -x_min - 80, 0);
                xStop = (int)Math.Min(start_pos[0] -x_min + 80, (x_max - x_min-1));
                zStart = (int)Math.Max(start_pos[2] - z_min - 80, 0);
                zStop = (int)Math.Min(start_pos[2] - z_min + 80, (int)(z_max - z_min-1));
            } else {*/
            xStart = 0;
            xStop = (int)(x_max - x_min - 1);
            zStart = 0;
            zStop = (int)(z_max - z_min - 1);
            //}
            System.Random r = new System.Random();
            bool notDone = true;
            int xInt = 0;
            int zInt = 0;
            while (notDone)
            {
                xInt = r.Next(xStart, xStop);
                zInt = r.Next(zStart, zStop);
                //UnityEngine.Debug.Log("Starting position" + start_pos);
                //UnityEngine.Debug.Log((xStart+x_min) + " " + (xStop+x_min) + " " + (zStart+x_min) + " " + (zStop+x_min));
                if (x_free[xInt, zInt] == 1)
                {
                    notDone = false;
                }
            }
            Vector3 pos = new Vector3(xInt + x_min + 1, 0, zInt + z_min + 1);
            //UnityEngine.Debug.Log("Sampled position" + pos);
            return pos;
        }

    }

    public bool ObstacleFree(Vector3 pos)
    {
        int xCord = (int)(pos[0] - x_min - 1);
        int zCord = (int)(pos[2] - z_min - 1);
        if (x_free[xCord, zCord] == 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    private Node2 NearestRecursive(Node2 node, Vector3 pos, Node2 x_nearest, float minDistance)
    {
        float xDist = pos[0] - node.pos[0];
        float zDist = pos[2] - node.pos[2];
        float distance = Mathf.Sqrt(Mathf.Pow(xDist, 2) + Mathf.Pow(zDist, 2));
        if (distance < minDistance)
        {
            x_nearest = node;
            minDistance = distance;
        }
        foreach (Node2 childNode in node.children)
        {
            x_nearest = NearestRecursive(childNode, pos, x_nearest, minDistance);
            xDist = pos[0] - x_nearest.pos[0];
            zDist = pos[2] - x_nearest.pos[2];
            minDistance = Mathf.Sqrt(Mathf.Pow(xDist, 2) + Mathf.Pow(zDist, 2));
        }
        return x_nearest;
    }

    private List<Node2> NearRecursive(Node2 node, Node2 x_new, List<Node2> near, float minDistance)
    {
        Vector3 pos = x_new.pos;
        float xDist = pos[0] - node.pos[0];
        float zDist = pos[2] - node.pos[2];
        float distance = Mathf.Sqrt(Mathf.Pow(xDist, 2) + Mathf.Pow(zDist, 2));

        if ((distance < minDistance) && (x_new != node) && (x_new.parent != node))
        {
            near.Add(node);
        }
        foreach (Node2 childNode in node.children)
        {
            near = NearRecursive(childNode, x_new, near, minDistance);
        }
        return near;
    }
    public float nfmod(float a, float b)
    {
        return a - b * Mathf.Floor(a / b);
    }
    private Node2 Steer(Node2 nearest, Vector3 pos, bool plottingPath)
    {
        float L = 3.0f;
        float Accel = 1f;
        float v = nearest.velocity;
        float theta = nearest.theta;
        Vector3 newPos = nearest.pos;
        float cost = nearest.cost;
        for (int i = 1; i < stepSize; i++)
        {
            UnityEngine.Debug.Log("------------------- Steer Iteration " + i + " ------------------- ");
            float xDist = pos[0] - newPos[0];
            float zDist = pos[2] - newPos[2];

            float newDistance = Mathf.Sqrt(Mathf.Pow(xDist, 2) + Mathf.Pow(zDist, 2));
            cost += newDistance;
            float goalDistance = Mathf.Sqrt(Mathf.Pow(newPos[0] - goal_pos[0], 2) + Mathf.Pow(newPos[2] - goal_pos[2], 2));
            //UnityEngine.Debug.Log("Distance " + newDistance);
            if (newDistance < 5)
            {
                return new Node2(null, newPos, cost, v, theta);
            }
            else if (goalDistance < 10)
            {
                return new Node2(null, newPos, cost, v, theta);
            }

            float vectorAngle = Mathf.Atan(zDist / xDist);
            //UnityEngine.Debug.Log("Vector angle before " + vectorAngle);
            if (
                (xDist < 0 && zDist > 0 && vectorAngle < 0) // if car_next lies in the second quadrant
                || (xDist < 0 && vectorAngle == 0) // if car_next lies on the x axis and points to the left
                || (xDist < 0 && zDist < 0) // if car_next lies in the third quadrant
            )
            {
                vectorAngle += Mathf.PI;
            }
            else if (xDist > 0 && zDist < 0)// if car_next lies in the fourth quadrant
            {
                vectorAngle += 2 * Mathf.PI;
            }
            //UnityEngine.Debug.Log("Vector angle after " + vectorAngle);
            float beta = 0;
            if (vectorAngle < Mathf.PI / 2)
            {
                UnityEngine.Debug.Log("Vector in the first quadrant");
                if (theta < Mathf.PI / 2)
                {
                    UnityEngine.Debug.Log("Theta in the first quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < Mathf.PI)
                {
                    UnityEngine.Debug.Log("Theta in the second quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 3 * Mathf.PI / 2)
                {
                    UnityEngine.Debug.Log("Theta in the third quadrant");
                    beta = -vectorAngle;
                }
                else if (theta < 2 * Mathf.PI)
                {
                    UnityEngine.Debug.Log("Theta in the fourth quadrant");
                    beta = 2 * Mathf.PI + vectorAngle - theta;
                }
            }
            else if (vectorAngle < Mathf.PI)
            {
                UnityEngine.Debug.Log("Vector in the second quadrant");
                if (theta < Mathf.PI / 2)
                {
                    UnityEngine.Debug.Log("Theta in the first quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < Mathf.PI)
                {
                    UnityEngine.Debug.Log("Theta in the second quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 3 * Mathf.PI / 2)
                {
                    UnityEngine.Debug.Log("Theta in the third quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 2 * Mathf.PI)
                {
                    UnityEngine.Debug.Log("Theta in the fourth quadrant");
                    beta = -vectorAngle;
                }
            }
            else if (vectorAngle < 3 * Mathf.PI / 2)
            {
                UnityEngine.Debug.Log("Vector in the third quadrant");
                if (theta < Mathf.PI / 2)
                {
                    UnityEngine.Debug.Log("Theta in the first quadrant");
                    beta = vectorAngle;
                }
                else if (theta < Mathf.PI)
                {
                    UnityEngine.Debug.Log("Theta in the second quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 3 * Mathf.PI / 2)
                {
                    UnityEngine.Debug.Log("Theta in the third quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 2 * Mathf.PI)
                {
                    UnityEngine.Debug.Log("Theta in the fourth quadrant");
                    beta = vectorAngle - theta;
                }
            }
            else if (vectorAngle < 2 * Mathf.PI)
            {
                UnityEngine.Debug.Log("Vector in the fourth quadrant");
                if (theta < Mathf.PI / 2)
                {
                    UnityEngine.Debug.Log("Theta in the first quadrant");
                    beta = -2 * Mathf.PI - vectorAngle - theta;
                }
                else if (theta < Mathf.PI)
                {
                    UnityEngine.Debug.Log("Theta in the second quadrant");
                    beta = -(Mathf.Sign(vectorAngle - Mathf.PI - theta));
                }
                else if (theta < 3 * Mathf.PI / 2)
                {
                    UnityEngine.Debug.Log("Theta in the third quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 2 * Mathf.PI)
                {
                    UnityEngine.Debug.Log("Theta in the fourth quadrant");
                    beta = vectorAngle - theta;
                }
            }
            else
            {
                UnityEngine.Debug.Log("None of the above. Should not happen.");
                beta = vectorAngle - theta;
            }
            //UnityEngine.Debug.Log("Beta " + beta);

            //vectorAngle += theta;
            if (beta < -maxSteerAngle)
            {
                beta = -maxSteerAngle;
            }
            else if (beta > maxSteerAngle)
            {
                beta = maxSteerAngle;
            }
            //UnityEngine.Debug.Log("Final beta" + beta);
            //UnityEngine.Debug.Log("Old theta "+ theta);
            float tanPsi = Mathf.Tan(beta);
            v += Accel;
            if (v > 5)
            {
                v = 5;
            }
            theta += v / L * tanPsi;
            theta = nfmod(theta, 2 * Mathf.PI);

            //UnityEngine.Debug.Log("New theta "+ theta);

            //UnityEngine.Debug.Log("xDist: " + xDist + " zDist: " + zDist + " Psi: "+ beta + " tanPsi " + tanPsi);
            Vector3 updatePos = new Vector3(newPos[0] + v * Mathf.Cos(theta), 0, newPos[2] + v * Mathf.Sin(theta));
            if (i % 2 == 0) { UnityEngine.Debug.DrawLine(newPos, updatePos, Color.blue, 100f); }
            else { UnityEngine.Debug.DrawLine(newPos, updatePos, Color.black, 1000f); }

            newPos = updatePos;
            //UnityEngine.Debug.Log("New Pos: " + newPos + " Velocity: " + v + " Accel: " + Accel);
            if (!ObstacleFree(newPos))
            {
                return null;
            }
        }
        Node2 newNode = new Node2(null, newPos, cost, v, theta);
        return newNode;
    }
    public List<Vector3> Run()
    {
        int iter = 0;
        bool notConverged = true;

        while (notConverged)
        {
            iter++;
            UnityEngine.Debug.Log("------------------------Iteration: " + iter + " ------------------------");
            Vector3 x_rand = SampleFree(iter);
            UnityEngine.Debug.Log("Sampled point" + x_rand);
            Node2 x_nearest = NearestRecursive(graph.root, x_rand, null, float.MaxValue);
            //UnityEngine.Debug.DrawLine(x_nearest.pos, x_rand, Color.red, 100f);
            UnityEngine.Debug.Log("Nearest point" + x_nearest.pos);
            Node2 x_new = Steer(x_nearest, x_rand, false);
            if (x_new != null)
            {
                UnityEngine.Debug.Log("New point" + x_new.pos);
                UnityEngine.Debug.Log("Obstacle Free");
                x_new.parent = x_nearest;
                x_nearest.addChild(x_new);

                List<Node2> near = new List<Node2>();
                near = NearRecursive(graph.root, x_new, near, 50);
                Node2 newParent = null;
                Node2 newChild = null;
                float newMinDistance = x_new.cost;
                //UnityEngine.Debug.Log("Number of near = " + near.Count);
                foreach (Node2 nearNode in near)
                {
                    //UnityEngine.Debug.Log("SteerNear");
                    Node2 x_new2 = Steer(nearNode, x_new.pos, false);
                    //UnityEngine.Debug.Log("x_new = " + x_new);
                    if (x_new2 != null)
                    {
                        if (x_new2.cost < newMinDistance)
                        {
                            //UnityEngine.Debug.Log("x_new2.cost " + x_new2.cost);
                            //UnityEngine.Debug.Log("newMinDistance " + newMinDistance);							
                            newMinDistance = x_new2.cost;
                            newChild = x_new2;
                            newParent = nearNode;
                        }
                    }
                }
                if (newParent != null)
                {
                    //UnityEngine.Debug.Log("Changing Parents. New min distance = " + newChild.cost);
                    x_nearest.removeChild(x_new);
                    newChild.parent = newParent;
                    newParent.addChild(newChild);
                    x_new = newChild;
                }
                float distance = Mathf.Sqrt(Mathf.Pow(x_new.pos[0] - goal_pos[0], 2) + Mathf.Pow(x_new.pos[2] - goal_pos[2], 2));
                if (distance < 10)
                {
                    UnityEngine.Debug.Log("Converged in " + iter + " iterations");
                    //notConverged = false;
                    if (x_best != null)
                    {
                        if (x_new.cost < x_best.cost)
                        {
                            x_best = x_new;
                        }
                    }
                    else
                    {
                        x_best = x_new;
                    }

                }
            }
            else
            {
                UnityEngine.Debug.Log("Not Obstacle Free :(");
            }
            if (iter % 250 == 0)
            {
                Node2 near = NearestRecursive(graph.root, goal_pos, null, float.MaxValue);
                x_best = near;
                notConverged = false;
            }
        }

        //List<Vector3> myPath = testPath();

        List<Vector3> myPath = new List<Vector3>();
        List<Node2> nodePath = new List<Node2>();

        Node2 node = x_best;
        myPath.Add(node.pos);
        //nodePath.Add(node);
        while (node.parent != null)
        {
            node = node.parent;
            myPath.Insert(0, node.pos);
            //nodePath.Insert(0, node);
        }

        myPath.Add(goal_pos);
        return myPath;
    }
}
