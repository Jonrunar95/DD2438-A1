public class Node
{
	public Node parent;
	public Node[] children;
	public float pos;
	public float cost;
	public Node(Node parent, float pos, float cost)
	{
		this.parent = parent;
		this.pos = pos;
		this.cost = cost;
		this.children = new Node[];
	}
	public Node getParent()
	{
		return parent;
	}
	public Node[] getChildren(int id)
	{
		return children;
	}
	public Node addChild(Node node)
	{
		children.add(node);
	}
}
public class Graph
{
	public Node root;
	public Node[] Vertices;
	public Graph(float pos)
	{
		this.root = new Node(null, pos);
		Vertices = new Node[];
	}
}
public class RRT
{
	public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
	Vector3[] my_path = new Vector3[];
	public Graph graph;
	Vector3 start_pos, goal_pos;
	public float x_min, x_max, z_min, z_max;
	float[, ,] x_free;
	public RRT()
	{
		this.graph = new Graph();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
		start_pos = terrain_manager.myInfo.start_pos;
        goal_pos = terrain_manager.myInfo.goal_pos;
		x_min = terrain_manager.myInfo.x_min;
		x_max = terrain_manager.myInfo.x_max;
		z_min = terrain_manager.myInfo.z_min;
		z_max = terrain_manager.myInfo.z_max;
		x_free = getXFree();
	}
	public void getXFree()
	{
		x_int = terrain_manager.myInfo.traversability;
		int N = x_int.GetLength(0);
		int M = x_int.GetLength(1);
		lenX = x_max-x_min;
		lenZ = z_max-z_min;
		x_free = new float[lenX, lenZ, 2];
		int n = lenX/N;
		int m = lenz/M;
		int buffer = 5;
		for(int i = 0; i < length; i++)
		{
			for(int j = 0; j < length; j++)
			{
				int posX = i/n;
				int posZ = j/m;
				if(x_int[posX, posZ] == 1) {
					x_free[i, j, 0] = i + x_min;
					x_free[i, j, 1] = j + z_min;
				} else {
					for(int k = -buffer; k < buffer+1; k++) {
						for(int l = -buffer; l <buffer+1; l++) {
							x_free[k, l, 0] = 0;
							x_free[k, l, 1] = 0;	
						}
					}
				}
			}
		}
		return x_free;
	}
	public void SampleFree()
	{
		Random r = new Random();
		bool notDone = true;
		int xInt, zInt;
		while(notDone) {
			xInt = r.Next(0, x_max-x_min);
			zInt = r.Next(0, z_max-z_min);
			if(x_free[xInt, zInt, 0] != 0) {
				notDone = false;
			}
		}
		return x_free[xInt, zInt];
	}
	private void ObstacleFree(float x_nearest, float x_new)
	{
		return 0;
	}
	private void Steer(float x_nearest, float x)
	{
		return 0;
	}
	private void Nearest(Graph graph, float x)
	{
		return 0;
	}
	private void Near()
	{
		return 0;
	}
	private void run()
	{
		float x_rand = SampleFree();
		Node x_nearest = Nearest(graph, x);
		Node x_new = Steer(x_nearest, x);
		if(ObstacleFree(x_nearest, x_new))
		{
			x_nearest.addChild(x_new);
		}
	}
}