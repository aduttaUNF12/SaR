package generateData;

import static org.graphstream.ui.graphicGraph.GraphPosLengthUtils.nodePosition;

import java.awt.Point;
import java.io.File;
import java.io.FileNotFoundException;
//import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.Scanner;
import java.util.stream.Stream;

import org.graphstream.algorithm.AStar;
import org.graphstream.algorithm.ConnectedComponents;
import org.graphstream.algorithm.Toolkit;
import org.graphstream.graph.DepthFirstIterator;
import org.graphstream.graph.Edge;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.Path;
import org.graphstream.graph.implementations.SingleGraph;

import com.google.ortools.*;

public class search_rescue {

	private static String fileAG = "coverageOutput.txt";
	protected static ArrayList<Point> chargers;
	private static double eSize;
	private static Graph g;
	public static ArrayList<Node> TSPnodes;
	public static ArrayList<Node> astarP;
	public static ArrayList<Node> astarSameCostP;
	
	static String styleSheet =
			"node.charger {" +
					"fill-color: black;"+
					"size: 12px;"+
					"text-size: 50;"+ "text-alignment: at-left;"+ "text-color: green;" +
					"}" +
					"node.astarSameCost {" +
					"fill-color: yellow;" + "size: 10px;"+ "text-size: 50;"+
					"}"+
					"edge.dfs {" +
					"fill-color: red;" + "size: 10px;"+ "text-size: 20;"+
					"}"+
					"node.tsp {" +
					"fill-color: green;"+
					"size: 12px;"+
					"text-size: 50;"+ "text-alignment: at-left;"+
					"}"+
					"edge.astar {" +
					"fill-color: orange;"+
					"size: 5px;"+
					"text-size: 50;"+ "text-alignment: at-left;"+
					"}";

	public search_rescue(int chargeCount) {
		// TODO Auto-generated constructor stub
		chargers = new ArrayList<Point>();
		g = new SingleGraph("final graph2");
		eSize=10;
		astarP = new ArrayList<Node>();
		astarSameCostP = new ArrayList<Node>();
		TSPnodes = new ArrayList<Node>();
	}



	public static void main(String[] args) {
		// TODO Auto-generated method stub
		boolean pathFound = false;
		System.setProperty("org.graphstream.ui", "swing");
		while(!pathFound) {
			int chargeCount =20; 
			double budget = 6;
			Point start = new Point(); start.setLocation(0.0, 0.0);
			search_rescue sar = new search_rescue(chargeCount);
			sar.chargers=generateChargePoints(chargeCount);
			ArrayList<Point> cov_points = sar.readAGOutput(6);
			double[][] DistanceMatrix = sar.createDistMat(cov_points, start);

			TSP tspInstance = new TSP(DistanceMatrix);
			ArrayList<Integer> tour = tspInstance.getSolution();
			System.out.println("TSP tour: " + tspInstance.getSolution());
			sar.g=createFinalGraph(tour, cov_points, budget);
			for (Node node : sar.g) {
		        node.setAttribute("ui.label", node.getId());
		    }
			sar.g.setAttribute("ui.stylesheet", styleSheet);
			double xy1[]= {0,0};
			double xy2[]={0,0};
			pathFound = true;
			for(int i=0;i<tour.size()-1;i++) {
//				xy1[0] = cov_points.get(tour.get(i)).getX();
//				xy1[1] = cov_points.get(tour.get(i)).getY();
//				xy2[0] = cov_points.get(tour.get(i+1)).getX();
//				xy2[1] = cov_points.get(tour.get(i+1)).getY();
				Graph gr = sar.planPath(sar.g.getNode(TSPnodes.get(i).getId()), sar.g.getNode(TSPnodes.get(i+1).getId()), budget);
				if(gr==null) {
					pathFound=false;
					break;
				}
			}
			if(pathFound!=false) {
				System.out.println(astarP.toString());
				sar.g.display();
			}
			
		}
		System.out.println("Done with this run.");
	}
	
	static Graph createFinalGraph(ArrayList<Integer> tour, ArrayList<Point> cov_points, double budget) {
		// TODO Auto-generated method stub
				int chargeCount = chargers.size();
				Graph g=new SingleGraph("final graph");
				Random rand = new Random();
				//generate charge locations and add them to G.
				for(int i=0; i< chargeCount; i++) {
					g.addNode(String.valueOf(i));
					double x = chargers.get(i).getX();
					double y = chargers.get(i).getY();
					g.getNode(String.valueOf(i)).setAttribute("xyz", x, y, 0);
					g.getNode(String.valueOf(i)).setAttribute("ui.class", "charger");
				}
				//generate TSP locations and add them to G -- for now they are random, but we should read their location from a file.
				for(int i=0; i< tour.size()-1; i++) {
					g.addNode(String.valueOf(chargeCount+i));
					double x = cov_points.get(tour.get(i)).getX();
					double y = cov_points.get(tour.get(i)).getY();
					g.getNode(String.valueOf(chargeCount+i)).setAttribute("xyz", x, y, 0);
					g.getNode(String.valueOf(chargeCount+i)).setAttribute("ui.class", "tsp");
					TSPnodes.add(g.getNode(String.valueOf(chargeCount+i)));
				}
				TSPnodes.add(g.getNode(String.valueOf(chargeCount)));
				// create the edges in the graph
				int edgeCount=0;
				for(Node n1: g) {
					for(Node n2: g) {
						if(!n1.getId().equals(n2.getId())) {
							double d = calculateHeuristicEuclid(n1, n2);
							if(!g.getNode(n1.getId()).hasEdgeBetween(g.getNode(n2.getId())) && d<budget) {
								g.addEdge(String.valueOf(edgeCount), n1.getId(), n2.getId());
								edgeCount++;
							}
						}
					}
				}
				return g;
	}


	static ArrayList<Point> generateChargePoints(int chargeCount) {
		Random rand = new Random();
		ArrayList<Point> pp = new ArrayList<Point>();
		rand.setSeed(System.currentTimeMillis());
		//generate random charge locations and add them to G.
		for(int i=0; i<chargeCount;i++) {
			double x = rand.nextDouble()*eSize;
			double y = rand.nextDouble()*eSize;
			Point p = new Point();
			p.setLocation(x, y);
			pp.add(p);
		}
		return pp;
	}

	static Graph planPath(Node start, Node target, double budget) {
		int chargeCount = chargers.size();
		boolean connectedG = false;
		//Graph g = null;
		//while(!connectedG) {
		//g = generateGraph4ECA(budget, xy1, xy2);
		ConnectedComponents cc = new ConnectedComponents();
		cc.init(g);
		if(cc.getConnectedComponentsCount()==1) {
			connectedG=true;
		}
		else {
			System.out.println("No solution for this TSP pair could be found.");
			return null;
		}
		//}
		System.out.println("Generated a connected graph with Charging stations and TSP waypoints as nodes.");
		//g.display();
		// find the A* path through the charging stations
		Path astarPath = findMinCostPath(g.getNode(start.getId()), g.getNode(target.getId()));
		astarP.addAll(astarPath.getNodePath());
		//System.out.println("# charging stations visited with A* = "+(astarPath.size()-2));

		// find the A* - same cost - path through the charging stations
		Path sameCostPath=findMinHopPath(g.getNode(start.getId()), g.getNode(target.getId()));
		astarSameCostP.addAll(sameCostPath.getNodePath());
		//System.out.println("# charging stations visited with A* (unit cost edges) = "+(sameCostPath.size()-2));

		//ArrayList<Node> dfsPath = findMinHopPathDFS(g, g.getNode(chargeCount), g.getNode(chargeCount+1));
		//System.out.println("# charging stations visited with DFS = "+(dfsPath.size()-2));
		//g.display();
		return g;
	}

	// read the output of Anirban's coverage algorithm
	public static ArrayList<Point> readAGOutput() {
		ArrayList<Point> coverPoints = new ArrayList<Point>();
		File file = new File(fileAG);
		Scanner in = null;
		try {
			in = new Scanner(file);
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		while(in.hasNextLine()) {
			String xy[] = in.nextLine().split(" ");
			//System.out.println(in.nextLine());
			Point p = new Point(); 
			p.setLocation(Double.parseDouble(xy[0]), Double.parseDouble(xy[1]));
			coverPoints.add(p);
		}
		in.close();
		return coverPoints;
	}
	
		// read the output of Anirban's coverage algorithm -- this is overloaded random generation function
		public static ArrayList<Point> readAGOutput(int count) {
			Random rand = new Random();
			rand.setSeed(System.currentTimeMillis());
			ArrayList<Point> coverPoints = new ArrayList<Point>();
			//generate random covering locations and add them to G.
			for(int i=0; i<count;i++) {
				double x = rand.nextDouble()*eSize;
				double y = rand.nextDouble()*eSize;
				Point p = new Point();
				p.setLocation(x, y);
				coverPoints.add(p);
			}
			return coverPoints;
		}

	// create a distance matrix from Anirban's coverage algorithm's output and the start point
	static double[][] createDistMat(ArrayList<Point> p, Point start) {
		p.add(0, start);// start has to be at index 0.
		double[][] mat = new double[p.size()][p.size()];
		for(int i=0;i<p.size();i++) {
			for(int j=0;j<p.size();j++) {
				mat[i][j] = p.get(i).distance(p.get(j));
			}
		}
		return mat;
	}


	private static Path findMinCostPath(Node start, Node goal) {
		// TODO Auto-generated method stub
		Path astarPath;
		AStar astar = new AStar(g);
		astar.setCosts(new DistanceCosts());
		astar.compute(start.getId(), goal.getId());
		astarPath=astar.getShortestPath();
		List<Node> nodes = astarPath.getNodePath();
		for(int i=0; i<nodes.size()-1;i++) {
			Node a = nodes.get(i);
			Node b = nodes.get(i+1);
			//if(a.getId()!=start.getId() && a.getId()!=goal.getId()) {
			//g.getNode(a.getId()).setAttribute("ui.class", "astar");
			g.getNode(a.getId()).getEdgeBetween(g.getNode(b.getId())).setAttribute("ui.class", "astar");
			//}
		}
		return astarPath;
	}

	private static Path findMinHopPath(Node start, Node goal)  {
		// TODO Auto-generated method stub
		Path astarPath;
		AStar astar = new AStar(g);
		astar.compute(start.getId(), goal.getId());
		astarPath=astar.getShortestPath();
		List<Node> nodes = astarPath.getNodePath();
		for(int i=0; i<nodes.size()-1;i++) {
			Node a = nodes.get(i);
			Node b = nodes.get(i+1);
			if(a.getId()!=start.getId() && a.getId()!=goal.getId())
				g.getNode(a.getId()).setAttribute("ui.class", "astarSameCost");
			//g.getNode(a.getId()).getEdgeBetween(g.getNode(b.getId())).setAttribute("ui.class", "astarSameCost");
			//double weight = (Double) g.getNode(a.getId()).getEdgeBetween(g.getNode(b.getId())).getAttribute("weight");
			//System.out.println("This edge weight is: "+g.getNode(a.getId()).getEdgeBetween(g.getNode(b.getId())).hasNumber("weight"));
		}
		return astarPath;
	}

	private static ArrayList<Node> findMinHopPathDFS(Graph g, Node start, Node goal)  {
		// TODO Auto-generated method stub
		ArrayList<Node> path = new ArrayList<Node>();
		Iterator<Node> dfs = g.getNode(start.getId()).getBreadthFirstIterator();
		Node previous = start;
		while(dfs.hasNext()) {
			Node n = dfs.next();
			g.getNode(n.getId()).setAttribute("parent", previous.getId());
			Iterator<Edge> ee = n.edges().iterator();
			previous = n;
			if(n.getId() == goal.getId()) {
				break;
			}
			//			while(ee.hasNext()) {
			//				Edge e = ee.next();
			//				g.getNode(e.getTargetNode().getId()).setAttribute("parent", n.getId());
			//			}
		}
		g.getNode(start.getId()).setAttribute("parent", "voila");
		//System.out.println("Found the DFS target");
		Node current = g.getNode(goal.getId());
		//System.out.println("Start is: "+start.toString()+" Current is: "+current.toString()+" and parent= "+g.getNode((String) current.getAttribute("parent")).toString());
		path.add(current);
		while(g.getNode(current.getId()).getAttribute("parent")!="voila") {
			current = g.getNode((String) current.getAttribute("parent"));
			//System.out.println("Current is: "+current.toString()+" and parent is "+g.getNode(current.getId()).getAttribute("parent"));
			if(current.getId()!=start.getId() && current.getId()!=goal.getId())
				g.getNode(current.getId()).setAttribute("ui.class", "dfs");
			path.add(g.getNode(current.getId()));
		}
		//System.out.println();
		return path;
	}

	/* POI set generation for CGAL --> Brian's code
	 * use this POI set for AG's code
	 * AG's code spits out service points
	 * Next, Brian's code uses this set and the start point to get the TSP tour.
	 * Generate a graph with these TSP points and the charging points
	 * Run ECA* on every consecutive pair in this TSP tour. 
	 * */
	private static Graph generateGraph4ECA(double budget, double[] xy1, double[] xy2) {
		// TODO Auto-generated method stub
		int chargeCount = chargers.size();
		Graph g=new SingleGraph("ECA graph");
		System.setProperty("org.graphstream.ui", "swing");
		Random rand = new Random();
		//generate random charge locations and add them to G.
		for(int i=1; i<= chargeCount; i++) {
			g.addNode(String.valueOf(i-1));
			double x = chargers.get(i-1).getX();
			double y = chargers.get(i-1).getY();
			g.getNode(String.valueOf(i-1)).setAttribute("xyz", x, y, 0);
			g.getNode(String.valueOf(i-1)).setAttribute("ui.class", "charger");
		}
		//generate TSP locations and add them to G -- for now they are random, but we should read their location
		// from a file.
		//		for(int i=1; i<= tsplength; i++) {
		//			g.addNode(String.valueOf(chargeCount+i-1));
		//			double x = rand.nextDouble()*10.00;
		//			double y = rand.nextDouble()*10.00;
		//			g.getNode(String.valueOf(chargeCount+i-1)).setAttribute("xyz", x, y, 0);
		//			g.getNode(String.valueOf(chargeCount+i-1)).setAttribute("ui.class", "tsp");
		//		}
		g.addNode(String.valueOf(chargeCount));
		g.getNode(String.valueOf(chargeCount)).setAttribute("xyz", xy1[0], xy1[1], 0);
		g.getNode(String.valueOf(chargeCount)).setAttribute("ui.class", "tsp");
		g.addNode(String.valueOf(chargeCount+1));
		g.getNode(String.valueOf(chargeCount+1)).setAttribute("xyz", xy2[0], xy2[1], 0);
		g.getNode(String.valueOf(chargeCount+1)).setAttribute("ui.class", "tsp");
		// create the edges in the graph
		int edgeCount=0;
		for(Node n1: g) {
			for(Node n2: g) {
				if(!n1.getId().equals(n2.getId())) {
					double d = calculateHeuristicEuclid(n1, n2);
					if(!g.getNode(n1.getId()).hasEdgeBetween(g.getNode(n2.getId())) && d<budget) {
						g.addEdge(String.valueOf(edgeCount), n1.getId(), n2.getId());
						edgeCount++;
					}
				}
			}
		}
		return g;
	}

	static double calculateHeuristicEuclid(Node node, Node target) {
		double xy1[] = Toolkit.nodePosition(node); 
		double xy2[] = Toolkit.nodePosition(target); 
		double dx = Math.abs(xy2[0] - xy1[0]); 
		double dy = Math.abs(xy2[1] - xy1[1]); 
		double H = Math.pow(dx, 2) + Math.pow(dy, 2);
		//System.out.println(xy1[0] +" and "+ xy1[1]);
		return Math.sqrt(H);
	}



}

/**
 * An implementation of the Costs interface that assume that the weight of 
 * edges is an Euclidean distance in 2D or 3D. No weight attribute is used. 
 * Instead, for the G value, the edge weights are used. For the H value the 
 * Euclidean distance in 2D or 3D between the current node and the target 
 * node is used. For this Costs implementation to work, the graph nodes must 
 * have a position (either individual "x", "y" and "z" attribute, or "xy" 
 * attribute or even "xyz" attributes. If there are only "x" and "y" or "xy" 
 * attribute this works in 2D, else the third coordinate is taken into 
 * account. 
 */ 
class DistanceCosts implements AStar.Costs { 
	public double heuristic(Node node, Node target) { // Cherysev Distance
		int D = 1;
		int D2 =1;
		double xy1[] = nodePosition(node); 
		double xy2[] = nodePosition(target); 

		double dx = Math.abs(xy2[0] - xy1[0]); 
		double dy = Math.abs(xy2[1] - xy1[1]); 
		//int dx = Math.abs(Start.GetCol() - End.GetCol());
		//int dy = Math.abs(Start.GetRow() - End.GetRow());
		//double H = D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
		//H = Math.max(Math.abs(End.GetCol() - Start.GetCol()), Math.abs(End.GetRow() - Start.GetRow()));
		return Math.hypot(dx, dy);
	} 

	public double cost(Node parent, Edge edge, Node next) { 
		//return edgeLength(edge);// parent.getEdgeToward( next.getId() ) );
		//return 1;
		//return heuristic(parent, next);
		//System.out.print("Using this as the cost function... \n");

		//Using Euclidean distance 
		double xy1[] = Toolkit.nodePosition(parent); 
		double xy2[] = Toolkit.nodePosition(next); 
		double dx = Math.abs(xy2[0] - xy1[0]); 
		double dy = Math.abs(xy2[1] - xy1[1]); 
		return Math.hypot(dx, dy);
	} 
} 

