import java.util.Random;
import java.util.Stack;

import org.graphstream.algorithm.ConnectedComponents;
import org.graphstream.algorithm.Toolkit;
import org.graphstream.algorithm.generator.Generator;
import org.graphstream.algorithm.generator.RandomEuclideanGenerator;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.SingleGraph;
//Add variable usage for RC Percentage
//Create loop for graph generation using Connected Components code
public class RoomLayout {
	//public static Graph graph = new SingleGraph("random euclidean");
	public static double[][] Main() {
		System.setProperty("org.graphstream.ui", "swing");
		Generator gen = new RandomEuclideanGenerator();
		((RandomEuclideanGenerator) gen).setThreshold(ECA.EdgeThreshold);
		Random rand = new Random();
		rand.setSeed(System.currentTimeMillis());
		int charge_count = (int) Math.ceil((ECA.RCpercentage*Math.pow(ECA.NodeCount-1, 2)/100));//# charging stations
		double [][] DistanceMatrix = new double[ECA.NodeCount][ECA.NodeCount];
		ConnectedComponents cc = new ConnectedComponents();
		do {
			ECA.graph.clear();
			ECA.graph.clearSinks();
			gen.addSink(ECA.graph);
			cc.init(ECA.graph);
			gen.begin();
			for(int i=1; i< ECA.NodeCount; i++) {
				gen.nextEvents();
			}
			gen.end();
		}while(cc.getConnectedComponentsCount() != 1);
		Stack <Node> GL = new Stack<>();
		ECA.graph.nodes().forEach(n ->{
			n.setAttribute("ID", 0);
			n.setAttribute("Cost", 0);
			n.setAttribute("Fn", 0);
			n.setAttribute("Hn", 0);
			n.setAttribute("Gn", 0);
			n.setAttribute("rcH", 0);
			n.setAttribute("Parent", null);
			n.setAttribute("ui.label", n.getId());
			GL.push(n);
		});
		Node Current;
		Node DistTemp;
		for(int i = 0; i < GL.size(); i++) {
			Current = GL.get(i);
			
		}
		for(int i = 0; i < ECA.NodeCount; i++)
			for(int j = 0; j < ECA.NodeCount; j++) {
				Current = GL.get(i);
				DistTemp = GL.get(j);
				DistanceMatrix[i][j] = EucDist(Current, DistTemp);
			}
		try {
			Thread.sleep(50);
		} catch( InterruptedException e) {
			e.printStackTrace();
		}
		
		int max = ECA.graph.getNodeCount();
		Node Start = ECA.graph.getNode(0);
		Node End = ECA.graph.getNode(max-1);
		Start.setAttribute("ui.style", "fill-color: rgb(0,255,0);");
		int k = 0;
		while(k < charge_count) {
			int n = rand.nextInt(ECA.NodeCount);
			int m = (int) ECA.graph.getNode(n).getAttribute("ID");
			if(m != 1 && m != 2) {
				ECA.graph.getNode(n).setAttribute("ID", 3);
				ECA.graph.getNode(n).setAttribute("ui.style", "fill-color: rgb(255,165,0);");
				k++;
			}
		}
		ECA.graph.display(false);
		try {
			Thread.sleep(2500);
		} catch( InterruptedException e) {
			e.printStackTrace();
		}
		return DistanceMatrix;
	}
	public static void PrintMatrix(double DistanceMatrix[][]) {
		for(int i = 0; i < ECA.NodeCount; i++) {
			System.out.print("[");
			for(int j = 0; j < ECA.NodeCount; j++) {
				System.out.print(DistanceMatrix[i][j]+", ");
			}
			System.out.println("]");
		}
	}
	private static double EucDist(Node x, Node y) {//Method for Setting Heuristic Value
		Node Current = x;
		Node End = y;
		double[] Endrelative = Toolkit.nodePosition(End);
		double[] Currentrelative = Toolkit.nodePosition(Current);
		double dx = Math.abs(Currentrelative[0] - Endrelative[0]);
		double dy = Math.abs(Currentrelative[0] - Endrelative[0]);
		//return D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
		return Math.hypot(dx, dy);
	}

}