package generateData;

//import static org.graphstream.ui.graphicGraph.GraphPosLengthUtils.nodePosition;
import static org.graphstream.algorithm.Toolkit.*;
import java.awt.Point;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
//import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashSet;
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
import org.graphstream.graph.implementations.Graphs;
import org.graphstream.graph.implementations.SingleGraph;

import com.google.ortools.*;

public class search_rescue {

	//private static String fileAG = "coverageOutput.txt";
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
					"text-size: 25;"+ "text-alignment: at-left;"+ "text-color: black;" +
					"}" +
					"node.start {" +
					"fill-color: green;"+
					"shape: diamond;"+
					"size: 20;"+
					"stroke-mode: plain;" + 
					"stroke-color: black;"+
					"stroke-width: 2;"+
					"text-size: 25;"+ "text-alignment: at-left;"+ "text-color: black;" +
					"}"+
					"node.astarSameCost {" +
					"fill-color: yellow;" + "size: 10px;"+ "text-size: 50;"+
					"}"+
					"edge.dfs {" +
					"fill-color: red;" + "size: 10px;"+ "text-size: 20;"+
					"}"+
					"node.tsp {" +
					"fill-color: green;"+
					"size: 12px;"+
					"text-size: 25;"+ "text-alignment: at-left;"+
					"}"+
					"edge.astar {" +
					"fill-color: red;"+
					"size: 5px;"+
					"text-size: 25;"+ "text-alignment: at-left;"+
					"}";

	public search_rescue() {
		// TODO Auto-generated constructor stub
		chargers = new ArrayList<Point>();
		g = new SingleGraph("final graph");
		eSize=8;
		astarP = new ArrayList<Node>();
		astarSameCostP = new ArrayList<Node>();
		TSPnodes = new ArrayList<Node>();
	}



	public static void main(String[] args) {
		// TODO Auto-generated method stub
		System.setProperty("org.graphstream.ui", "swing");
		boolean display = false;
		int box = 50; 
		int[] chargeP = {5,7};
		int[] budgetP = {8,10};
		for(int envSize=100; envSize<=1000; envSize+=100) {
			for(int bp=0; bp<budgetP.length; bp++) {
				for(int cp=0; cp<chargeP.length; cp++) {
					String fileResults = new String("Results/Data_n"+envSize+"_RC_"+cp+"p_B_"+bp+".txt");
					BufferedWriter pw=null;
					try {
						pw =  new BufferedWriter(new FileWriter(fileResults));
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					for(int run=0; run<10; run++) {
						// AG's local file name format --> "COVER_" + to_string(sizeOfSquare)  + "_" + to_string(n) + "_" + to_string(i) + ".txt";
						String fileAG = new String("AGdata/COVER_env50_n"+envSize+"_run"+run+".txt");
						double budget = (int) Math.ceil(box * Math.sqrt(2) * budgetP[bp]/100.00);
						int chargeCount =(int) Math.ceil(box*box*chargeP[cp]/100.00);
						boolean pathFound = false;
						while(!pathFound) {
							double startT = System.currentTimeMillis();
							//int chargeCount =20; 
							//double budget = 2.5;
							Point start = new Point(); start.setLocation(0.0, 0.0);
							search_rescue sar = new search_rescue();
							sar.chargers=sar.generateChargePoints(chargeCount,box);
							ArrayList<Point> cov_points = sar.readAGOutput(fileAG);
							double[][] DistanceMatrix = sar.createDistMat(cov_points, start);

							TSP tspInstance = new TSP(DistanceMatrix);
							ArrayList<Integer> tour = tspInstance.getSolution();
							//System.out.println("TSP tour: " + tspInstance.getSolution());
							sar.g=sar.createFinalGraph(tour, cov_points, budget);
							//sar.g.display();
							if(display) {
								for (Node node : sar.g) {
									node.setAttribute("ui.label", node.getId());
								}
								sar.g.setAttribute("ui.stylesheet", styleSheet);
								sar.g.getNode(String.valueOf(chargeCount)).setAttribute("ui.class", "start");
							}
							g=sar.g;
							double xy1[]= {0,0};
							double xy2[]={0,0};
							pathFound = true;
							double cost = 0;
							for(int i=0;i<tour.size()-1;i++) {
//								xy1[0] = cov_points.get(tour.get(i)).getX();
//								xy1[1] = cov_points.get(tour.get(i)).getY();
//								xy2[0] = cov_points.get(tour.get(i+1)).getX();
//								xy2[1] = cov_points.get(tour.get(i+1)).getY();
								Path path = sar.planPath(sar.g.getNode(TSPnodes.get(i).getId()), sar.g.getNode(TSPnodes.get(i+1).getId()), budget);
								if(path==null) {
									pathFound=false;
									break;
								}
								List<Node> pathNodes = path.getNodePath();
								for(int p=0; p<pathNodes.size()-1;p++) {
									cost = cost + sar.calculateHeuristicEuclid(sar.g.getNode(pathNodes.get(p).getId()), sar.g.getNode(pathNodes.get(p+1).getId()));
								}
							}
							double endT = System.currentTimeMillis();
							//System.out.println("Cost is: "+cost+ " and elapsed time is: "+(endT-startT));
							int visitedCS = sar.getVisitedCSCount(astarP,chargers);
							if(pathFound!=false) {
								//System.out.println(astarP.toString());
								writeData2File(endT-startT, cost, envSize, visitedCS, cp, bp,pw);
								if(display)
									sar.g.display();
							}
							clearAllData(sar);
						}// end of 1 valid run with this setting
					}// end of ALL VALID runs for this setting
					try {
						pw.close();
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					System.out.printf("Done with Size: %d, bp: %d, cp: %d, and all runs.\n",envSize,bp,cp);
				}// end of cp loops
			}// end of bp loops
		}// end of env size loops
		System.out.println("*** DATA COLLECTION IS COMPLETE NOW ***");
	}
	
	private static int getVisitedCSCount(ArrayList<Node> path, ArrayList<Point> cs) {
		// TODO Auto-generated method stub
		int count=0;
		HashSet<Node> pathSet = new HashSet<>(path);
//		for(Node n: pathSet) {
//			double xy[] = nodePosition(n);
//			for(Point c: cs) {
//				if(c.getX()==xy[0] && c.getY()==xy[1]) {
//					count++;
//				}
//			}
//		}
		for(Node n: pathSet) {
			if(Integer.parseInt(n.getId())<cs.size())
				count++;
		}
		return count;
	}



	private static void clearAllData(search_rescue sar) {
		// TODO Auto-generated method stub
		sar.chargers.clear();
		sar.g.clear();
		sar.astarP.clear();
		sar.astarSameCostP.clear();
		sar.TSPnodes.clear();
	}



	private static void writeData2File(double l, double cost, int size, int stationCount, int cp, int bp, BufferedWriter pw) {
		// TODO Auto-generated method stub
		try {
			//System.out.println(results[0]+","+results[1]+","+results[2]);
			pw.write(l+","+cost+","+size+","+stationCount+","+cp+","+bp+"\n");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
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
				//add the start node
//				g.addNode(String.valueOf(chargeCount));
//				g.getNode(String.valueOf(chargeCount)).setAttribute("xyz", 0, 0, 0);
//				g.getNode(String.valueOf(chargeCount)).setAttribute("ui.class", "tsp");
//				TSPnodes.add(g.getNode(String.valueOf(chargeCount)));
				//generate TSP locations and add them to G -- for now they are random, but we should read their location from a file.
				for(int i=0; i< tour.size()-1; i++) {
					g.addNode(String.valueOf(chargeCount+i));
					double x = cov_points.get(tour.get(i)).getX();
					double y = cov_points.get(tour.get(i)).getY();
					g.getNode(String.valueOf(chargeCount+i)).setAttribute("xyz", x, y, 0);
					g.getNode(String.valueOf(chargeCount+i)).setAttribute("ui.class", "tsp");
					TSPnodes.add(g.getNode(String.valueOf(chargeCount+i)));
					//System.out.println("TSP tour node "+tour.get(i)+" = Graph node "+(chargeCount+i));
				}
				//for the start node to be added at the end to complete the TSP route
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
							if(d>budget) {
								//System.out.printf("d=%f\n",d);
							}
						}
					}
				}
				return g;
	}


	static ArrayList<Point> generateChargePoints(int chargeCount, int box) {
		Random rand = new Random();
		ArrayList<Point> pp = new ArrayList<Point>();
		rand.setSeed(System.currentTimeMillis());
		//generate random charge locations and add them to G.
		for(int i=0; i<chargeCount;i++) {
			double x = rand.nextDouble()*box - box/2;
			double y = rand.nextDouble()*box - box/2;
			Point p = new Point();
			p.setLocation(x, y);
			pp.add(p);
		}
		return pp;
	}

	static Path planPath(Node start, Node target, double budget) {
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
			System.out.println("The graph is disconnected.");
			return null;
		}
		//}
		//System.out.println("Generated a connected graph with Charging stations and TSP waypoints as nodes. Found path between: "+start.getId()+" and "+target.getId());
		//g.display();
		// find the A* path through the charging stations
		Path astarPath = findMinCostPath(g.getNode(start.getId()), g.getNode(target.getId()));
		if(astarPath==null) {
			System.out.println("No solution for this TSP pair could be found.");
			return null;
		}
		astarP.addAll(astarPath.getNodePath());
		//System.out.println("# charging stations visited with A* = "+(astarPath.size()-2));

		// find the A* - same cost - path through the charging stations
		//Path sameCostPath=findMinHopPath(g.getNode(start.getId()), g.getNode(target.getId()));
		//astarSameCostP.addAll(sameCostPath.getNodePath());
		//System.out.println("# charging stations visited with A* (unit cost edges) = "+(sameCostPath.size()-2));

		//ArrayList<Node> dfsPath = findMinHopPathDFS(g, g.getNode(chargeCount), g.getNode(chargeCount+1));
		//System.out.println("# charging stations visited with DFS = "+(dfsPath.size()-2));
		//g.display();
		return astarPath;
	}

	// read the output of Anirban's coverage algorithm
	public static ArrayList<Point> readAGOutput(String fileAG) {
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
	
		// read the output of Anirban's coverage algorithm -- this is overloaded -- random generation function
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
		Graph subG = Graphs.clone(g);
		subG = extractSubG4TSPpair(subG, start, goal);
		AStar astar = new AStar(subG);
		astar.setCosts(new DistanceCosts());
		astar.compute(subG.getNode(start.getId()).getId(), subG.getNode(goal.getId()).getId());
		astarPath=astar.getShortestPath();
		if(astarPath==null) {
			return null;
		}
		List<Node> nodes = astarPath.getNodePath();
		for(int i=0; i<nodes.size()-1;i++) {
			Node a = nodes.get(i);
			Node b = nodes.get(i+1);
			//if(a.getId()!=start.getId() && a.getId()!=goal.getId()) {
			//g.getNode(a.getId()).setAttribute("ui.class", "astar");
			g.getNode(a.getId()).getEdgeBetween(g.getNode(b.getId())).setAttribute("ui.class", "astar");
			//}
		}
		subG.clear();
		return astarPath;
	}

	private static Graph extractSubG4TSPpair(Graph subg, Node start, Node goal) {
		// TODO Auto-generated method stub
		Iterator<Node> it = g.nodes().iterator();
		
		while(it.hasNext()) {
			Node n = it.next();
			if(n==null)
				continue;
			//System.out.println("Charger size is: "+chargers.size()+" N id is: "+n.getId()+" Start id is: "+start.getId()+" target id is: "+goal.getId()+".");
			//System.out.println(" N id is: "+n.getId());
			if(Integer.parseInt(n.getId()) < chargers.size() || n.getId()==start.getId() || n.getId()==goal.getId()) {
				//continue;
			}
			else {
				//if(subg.getNode(n.getId())!=null)
				//System.out.println("Removed node is: "+n.getId());
				subg.removeNode(n.getId());
			}
		}
		return subg;
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
					if(d>budget) {
						System.out.println("Not connected"+n1.toString()+" and "+n2.toString());
					}
				}
			}
		}
		return g;
	}

	static double calculateHeuristicEuclid(Node node, Node target) {
		double xy1[] = nodePosition(node); 
		double xy2[] = nodePosition(target); 
		//double xy1[] = {0,0,0};
		//double xy2[] = {0,0,0};
		//xy1 = (double[]) node.getAttribute("xyz");
		//System.out.println(node.getAttribute("xyz"));
		//xy1[1] = (double) node.getAttribute("y");
		//xy2 = (double[]) target.getAttribute("xyz");
		//xy2[1] = (double) target.getAttribute("y");
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

