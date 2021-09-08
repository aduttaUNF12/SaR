package generateData;

import java.util.ArrayList;
import com.google.ortools.Loader;
import com.google.ortools.constraintsolver.*;

public class TSP {
	int[][] distanceMatrix;
	int n, depot = 0;
	double cost = 0.0;

	public TSP(double[][] locations) {
		System.out.println("/*********************************************/");
		System.out.println("/* Beware! the start point must have index 0 */");
		System.out.println("/*********************************************/");

		this.n = locations.length;

		distanceMatrix = new int[n][n];
		for(int i = 0; i < n; i++)
			for(int j = 0; j < n; j++)
				distanceMatrix[i][j] = (int)(locations[i][j] * 100);
	}

	public ArrayList<Integer> getSolution() {
		ArrayList<Integer> tour = new ArrayList<Integer>(n+1);

		Loader.loadNativeLibraries();
		RoutingIndexManager manager = new RoutingIndexManager(n, 1, depot); 
		RoutingModel routing = new RoutingModel(manager);

		final int transitCallbackIndex =
				routing.registerTransitCallback((long fromIndex, long toIndex) -> {
					int fromNode = manager.indexToNode(fromIndex);
					int toNode = manager.indexToNode(toIndex);
					return distanceMatrix[fromNode][toNode];
				});

		routing.setArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

		RoutingSearchParameters searchParameters =
				main.defaultRoutingSearchParameters()
				.toBuilder()
				.setFirstSolutionStrategy(FirstSolutionStrategy.Value.CHRISTOFIDES)   
				.build();

		Assignment solution = routing.solveWithParameters(searchParameters);

		long index = routing.start(0);

		while (!routing.isEnd(index)) {
			tour.add( manager.indexToNode(index) );
			long previousIndex = index;
			index = solution.value(routing.nextVar(index));
			cost += routing.getArcCostForVehicle(previousIndex, index, 0);
		}
		tour.add(manager.indexToNode(routing.end(0)));
		cost = solution.objectiveValue();

		return tour;
	}

	public double getCost() {
		return cost/100;
	}
}
