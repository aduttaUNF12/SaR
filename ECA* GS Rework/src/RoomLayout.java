import java.util.Random;

import org.graphstream.algorithm.ConnectedComponents;
import org.graphstream.algorithm.generator.Generator;
import org.graphstream.algorithm.generator.RandomEuclideanGenerator;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.SingleGraph;
//Add variable usage for RC Percentage
//Create loop for graph generation using Connected Components code
public class RoomLayout {
	//public static Graph graph = new SingleGraph("random euclidean");
	public static void Main() {
		System.setProperty("org.graphstream.ui", "swing");
		Generator gen = new RandomEuclideanGenerator();
		((RandomEuclideanGenerator) gen).setThreshold(.4);
		Random rand = new Random();
		rand.setSeed(System.currentTimeMillis());
		int charge_count = (int) Math.ceil((Main.RCpercentage*Math.pow(Main.NodeCount-1, 2)/100));//# charging stations
		ConnectedComponents cc = new ConnectedComponents();
		do {
			Main.graph.clear();
			Main.graph.clearSinks();
			gen.addSink(Main.graph);
			cc.init(Main.graph);
			gen.begin();
			for(int i=1; i< Main.NodeCount; i++) {
				gen.nextEvents();
			}
			gen.end();
		}while(cc.getConnectedComponentsCount() != 1);
		Main.graph.nodes().forEach(n ->{
			n.setAttribute("ID", 0);
			n.setAttribute("Cost", 0);
			n.setAttribute("Fn", 0);
			n.setAttribute("Hn", 0);
			n.setAttribute("Gn", 0);
			n.setAttribute("rcH", 0);
			n.setAttribute("Parent", null);
		});
		
		
		try {
			Thread.sleep(50);
		} catch( InterruptedException e) {
			e.printStackTrace();
		}
		
		int max = Main.graph.getNodeCount();
		Node Start = Main.graph.getNode(0);
		Node End = Main.graph.getNode(max-1);
		Start.setAttribute("ID", 1);
		End.setAttribute("ID", 2);
		Start.setAttribute("ui.style", "fill-color: rgb(0,255,0);");
		End.setAttribute("ui.style", "fill-color: rgb(255,0,0);");
		int k = 0;
		while(k < charge_count) {
			int n = rand.nextInt(Main.NodeCount);
			int m = (int) Main.graph.getNode(n).getAttribute("ID");
			if(m != 1 && m != 2) {
				Main.graph.getNode(n).setAttribute("ID", 3);
				Main.graph.getNode(n).setAttribute("ui.style", "fill-color: rgb(255,165,0);");
				k++;
			}
		}
		Main.graph.display(false);
		try {
			Thread.sleep(2500);
		} catch( InterruptedException e) {
			e.printStackTrace();
		}
	}

}
