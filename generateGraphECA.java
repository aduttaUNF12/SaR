package generateData;

import static org.graphstream.ui.graphicGraph.GraphPosLengthUtils.nodePosition;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
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

public class generateGraphECA {

	static String styleSheet =
			"node.charger {" +
					"fill-color: black;"+
					"size: 12px;"+
					"text-size: 50;"+ "text-alignment: at-left;"+
					"}" +
					"node.astarSameCost {" +
					"fill-color: red;" + "size: 10px;"+ "text-size: 20;"+
					"}"+
					"node.dfs {" +
					"fill-color: red;" + "size: 10px;"+ "text-size: 20;"+
					"}"+
					"node.tsp {" +
					"fill-color: green;"+
					"size: 12px;"+
					"text-size: 50;"+ "text-alignment: at-left;"+
					"}"+
					"edge.astar {" +
					"fill-color: orange;"+
					"size: 3px;"+
					"text-size: 50;"+ "text-alignment: at-left;"+
					"}";

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		int chargeCount =20; 
		int tsplength = 2;//sending two consecutive TSP points
		double budget = 2.5;
		double xy1[] = {0,0};//need to read from Brian's code
		double xy2[] = {6,6};//need to read from Brian's code
		boolean connectedG = false;
		Graph g = null;
		while(!connectedG) {
			g = generateGraph4ECA(chargeCount,tsplength, budget, xy1, xy2);
			ConnectedComponents cc = new ConnectedComponents();
			cc.init(g);
			if(cc.getConnectedComponentsCount()==1) {
				connectedG=true;
			}
			else {
				//System.out.println("No solution for this TSP pair could be found.");
				//return;
			}
		}
		System.out.println("Generated a connected graph with Charging stations and TSP waypoints as nodes.");
		g.setAttribute("ui.stylesheet", styleSheet);
		// find the A* path through the charging stations
		Path astarPath = findMinCostPath(g, g.getNode(chargeCount), g.getNode(chargeCount+1));
		System.out.println("# charging stations visited with A* = "+(astarPath.size()-2));

		// find the A* - same cost - path through the charging stations
		//Path sameCostPath=findMinHopPath(g, g.getNode(chargeCount), g.getNode(chargeCount+1));
		
		ArrayList<Node> dfsPath = findMinHopPathDFS(g, g.getNode(chargeCount), g.getNode(chargeCount+1));
		System.out.println("# charging stations visited with DFS = "+(dfsPath.size()-2));
		g.display();
	}

	private static Path findMinCostPath(Graph g, Node start, Node goal) {
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

	private static Path findMinHopPath(Graph g, Node start, Node goal)  {
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

		}
		return astarPath;
	}
	
	private static ArrayList<Node> findMinHopPathDFS(Graph g, Node start, Node goal)  {
		// TODO Auto-generated method stub
		ArrayList<Node> path = new ArrayList<Node>();
		Iterator<Node> dfs = g.getNode(start.getId()).getDepthFirstIterator();
		int count = 0;
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
	private static Graph generateGraph4ECA(int chargeCount, int tsplength, double budget, double[] xy1, double[] xy2) {
		// TODO Auto-generated method stub
		Graph g=new SingleGraph("ECA graph");
		System.setProperty("org.graphstream.ui", "swing");
		Random rand = new Random();
		//generate random charge locations and add them to G.
		for(int i=1; i<= chargeCount; i++) {
			g.addNode(String.valueOf(i-1));
			double x = rand.nextDouble()*10.00;
			double y = rand.nextDouble()*10.00;
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

