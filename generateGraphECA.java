package generateData;

import java.util.Random;

import org.graphstream.algorithm.ConnectedComponents;
import org.graphstream.algorithm.Toolkit;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.SingleGraph;

public class generateGraphECA {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		int chargeCount =10; int tsplength = 4;
		double budget = 2.5;
		boolean connectedG = false;
		Graph g = null;
		while(!connectedG) {
			g = generateGraph4ECA(chargeCount,tsplength, budget);
			ConnectedComponents cc = new ConnectedComponents();
			cc.init(g);
			if(cc.getConnectedComponentsCount()==1) {
				connectedG=true;
			}
		}
		System.out.println("Generated a connected graph with Charging stations and TSP waypoints as nodes.");
		g.display();
	}

	private static Graph generateGraph4ECA(int chargeCount, int tsplength, double budget) {
		// TODO Auto-generated method stub
		Graph g=new SingleGraph("ECA graph");
		System.setProperty("org.graphstream.ui", "swing");
		Random rand = new Random();
		//generate random charge locations and add them to G.
		for(int i=1; i<= chargeCount; i++) {
			g.addNode(String.valueOf(i-1));
			double x = rand.nextDouble()*5.00;
			double y = rand.nextDouble()*5.00;
			g.getNode(String.valueOf(i-1)).setAttribute("xyz", x, y, 0);
		}
		//generate TSP locations and add them to G -- for now they are random, but we should read their location
		// from a file.
		for(int i=1; i<= tsplength; i++) {
			g.addNode(String.valueOf(chargeCount+i-1));
			double x = rand.nextDouble()*10.00;
			double y = rand.nextDouble()*10.00;
			g.getNode(String.valueOf(chargeCount+i-1)).setAttribute("xyz", x, y, 0);
		}
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
