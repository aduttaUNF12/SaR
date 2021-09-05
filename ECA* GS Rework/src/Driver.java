import java.util.ArrayList;
import java.util.Scanner;

import org.graphstream.graph.Graph;
import org.graphstream.graph.implementations.SingleGraph;

public class Driver {
	public static Graph graph = new SingleGraph("random euclidean");
	public static void main(String[] args) {
		Scanner scan = new Scanner(System.in);
		System.out.print("Enter number of Nodes to be generated: ");
		ECA.NodeCount = scan.nextInt();
		double [][] DistanceMatrix = RoomLayout.Main();
		//RoomLayout.PrintMatrix(DistanceMatrix);
		
		TSP tspInstance = new TSP(DistanceMatrix);
		ArrayList<Integer> tour = tspInstance.getSolution();
		System.out.println("TSP tour: " + tspInstance.getSolution());
		
		for(int i=0; i< tour.size() - 1;i++){
			ECA.main(tour.get(i), tour.get(i+1));
		}
		
		System.out.println("Finished");
		System.out.println("Press Enter to quit");
		scan.next();
		scan.close();
		System.exit(0);
	}
}
