import java.util.*;

import org.graphstream.graph.Element;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.SingleGraph;
import org.graphstream.algorithm.Toolkit;
import java.util.Stack;
/*
 * Currently have a slight pause after the graph generates so that there is time to see starting point and end point.
 * 
 * Current Issues:
 * -Once a RC station has been given, path finding to the End point doesn't work. Doesn't recognize End point and explores over it.
 * -Will still randomly freeze, not certain it is an infinite loop as I have tested them all. 
 * -Not path finding optimally currently. Still working on the evaluation of the heuristics to work on this.
 */
public class Main implements Runnable{
	static int NodeCount;
	public static Stack <Node> Open = new Stack<>();
	public static Stack <Node> Closed = new Stack<>();
	public static Stack <Node> BestPath = new Stack<>();
	public static Stack <Node> RC = new Stack<>();
	public static Stack <Node> RCP = new Stack <>();
	public static double FullTank = .5;
	public static double RCpercentage = .5;
	public static int Trials = 1;
	public static double Cost = 0;
	public static double timeStart = 0;
	public static double timeEnd = 0;
	public static double timeRun = 0;
	public static int RCindex = 0;
	public static int StationCounter = 0;
	public static int TrialCounter = 0;
	public static double B = FullTank;
	public static int k = 0;
	
	public static Graph graph = new SingleGraph("random euclidean");
	public static void main(String[] args) {
		System.setProperty("org.graphstream.ui", "swing");
		new Thread (new Main()).start();
	}//End of main
	public static Node NextStation(Node RCp, Node End) {//Method for finding the next RC point
		Node rc;
		RCp.setAttribute("Hn", SetHn(RCp, End));
		if(HDist(RCp, End) <= FullTank) {
			double RCpValue = RCp.getNumber("Hn");
			System.out.println("RC Node Hit is End Point: "+RCpValue);
			rc = graph.getNode(End.getId());
		} else {
			if(RC.size() == 0) {
				System.out.println("No more Recharge Stations");
				return null;
			}
			Node Temp = graph.getNode(RC.get(0).getId());
			Temp.setAttribute("rcH", HDist(RCp, RC.get(0)) + 2*HDist(RC.get(0), End) - HDist(RCp, End));
			for(int i = 0; i < RC.size(); i++) {
				RC.get(i).setAttribute("rcH", HDist(RCp, RC.get(i)) + 2*HDist(RC.get(i), End) - HDist(RCp, End));
				//System.out.println(RC.get(i).GetrcH());
				double TemprcH = Temp.getNumber("rch");
				double RCrch = RC.get(i).getNumber("rch");
				if(TemprcH >= RCrch && HDist(RCp, RC.get(i)) <= B) {
					Temp = graph.getNode(RC.get(i).getId());
					RCindex = i;
				}
			}
			rc = graph.getNode(Temp.getId());
			double RCpValue =  RCp.getNumber("Hn");
			System.out.println("RC Node Hit: "+RCpValue);
		}
		rc.setAttribute("ui.style", "fill-color: rgb(255,203,182);");
		return rc;
	}
	public static double HDist(Node Current, Node End) {//Method for Finding distance between two Nodes
		double H;
		int D = 1;
		int D2 =1;
		double[] Endrelative = Toolkit.nodePosition(End);
		double[] Currentrelative = Toolkit.nodePosition(Current);
		double dx = Math.abs(Currentrelative[0] - Endrelative[0]);
		double dy = Math.abs(Currentrelative[0] - Endrelative[0]);
		H = D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
		//H = Math.max(Math.abs(End.GetCol() - Start.GetCol()), Math.abs(End.GetRow() - Start.GetRow()));
		//return H;
		return Math.hypot(dx, dy);
	}
	public static Node PathFind(Node Current, Node End) { //Method for Path finding from Current to RC
		int TempIndex = 0;
		Node Temp = null;
		Node Initial = graph.getNode(Current.getId());
		Open.push(Current);
		Current.setAttribute("Cost", 0);
		while(Open.empty() == false) {//Start of A*
			if(B < 0){
				System.out.println("No solution under cost.");
				return null;
			}
			TempIndex = 0;
			Current = graph.getNode(Open.get(0).getId());
			for(int i = 0; i < Open.size(); i++) {
				Open.get(i).setAttribute("Hn", SetHn(Open.get(i), End));
				Open.get(i).setAttribute("Fn", SetFn(Open.get(i)));
				Current.setAttribute("Hn", SetHn(Current, End));
				Current.setAttribute("Fn", SetFn(Current));
				System.out.println("This the Fn back in the loop: "+ Current.getNumber("Fn"));
				double a = Open.get(i).getNumber("Fn");
				double b = Current.getNumber("Fn");
				if(a < b) {
					Current = graph.getNode(Open.get(i).getId());
					TempIndex = i;
				}
			}
			if (graph.getNode(Current.getId()) == graph.getNode(End.getId())) {
				Temp = graph.getNode(Current.getId());
				BestPath.push(Current);
				graph.getNode(End.getId()).setAttribute("Gn", Current.getNumber("Gn") + HDist(Current, End));
				while(Current.getAttribute("Previous") != Initial) {
					Node Previous = graph.getNode(((Node) Current.getAttribute("Previous")).getId());
					Cost = Cost + Current.getNumber("Hn");
					BestPath.push(Previous);
					graph.getNode(Current.getId()).getEdgeBetween(graph.getNode(Previous.getId())).setAttribute("ui.style", " fill-color: rgb(0,0,255);");
					Previous.setAttribute("ui.style", " fill-color: rgb(0,0,255);");
					Current = graph.getNode(Previous.getId());
				}
				return Temp;
			}
			if(Current.getAttribute("Previous") != null) {
				Node Previous = graph.getNode(((Node) Current.getAttribute("Previous")).getId());
				double Pcost = Previous.getNumber("Cost");
				Current.setAttribute("Cost", Pcost+ HDist(Current, Previous));
			}else {
				Current.setAttribute("Cost", 0);
			}
			if(RC.contains(Current)) {
				B = FullTank;
			}else {
				B = B - Current.getNumber("Cost");
			}
			System.out.println("Fuel: "+(B));
			Temp = graph.getNode(Open.get(TempIndex).getId());
			Open.remove(Current);
			Closed.push(Current);
			int NeighborCount = Current.getDegree();
			Node Neighbor;
			for(int i = 0; i < NeighborCount; i++){
				if(Current.getEdge(i) == null) continue;
				Neighbor = graph.getNode(Current.getEdge(i).getOpposite(Current).getId());
				graph.getNode(Neighbor.getId()).setAttribute("Previous", Current);
				double Gn = Current.getNumber("Gn");
				graph.getNode(Neighbor.getId()).setAttribute("Gn", Gn + HDist(Current, Neighbor));
				if (Open.contains(Neighbor)) {
						continue;
				}else {
					if(Closed.contains(Neighbor)) {
						continue;
					}
					Neighbor.setAttribute("ui.style", "fill-color: rgb(0,255,0);");
					Open.push(Neighbor);
				}
			}
			Closed.push(Current);
			/*try {
				TimeUnit.MILLISECONDS.sleep(60);
			} catch (InterruptedException e) {
				e.printStackTrace();
	        	}*/
		}//end of Search
		System.out.println("PathFind Failed");
		return null;
	}
	private static double SetHn(Node Current, Node End) {//Method for Setting Heuristic Value
		int D = 1;
		int D2 =1;
		double[] Endrelative = Toolkit.nodePosition(End);
		double[] Currentrelative = Toolkit.nodePosition(Current);
		double dx = Math.abs(Currentrelative[0] - Endrelative[0]);
		double dy = Math.abs(Currentrelative[0] - Endrelative[0]);
		//return D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
		return Math.hypot(dx, dy);
	}
	private static double SetFn(Node Current) {//Method for Setting F Value
		double Hn = Current.getNumber("Hn");
		System.out.println("This is the Hn in the method Fn "+ Current.getNumber("Hn"));
		double Gn = Current.getNumber("Gn");
		System.out.println("This is the Gn in the method Fn "+ Current.getNumber("Gn"));
		System.out.println("This is the Fn in the method Fn "+ (Hn + Gn));
		return Hn + Gn;
	}
	public void run(){//Main Method
		Scanner scan = new Scanner(System.in);
		while(true) {
			Open.clear();
			Closed.clear();
			BestPath.clear();
				System.out.print("Enter number of Nodes to be generated: ");
				NodeCount = scan.nextInt();
				RoomLayout.Main();
				int z = 0;
				while(z < Trials){
					timeStart = System.currentTimeMillis();
					Node Start = graph.getNode(0);
					Node End = graph.getNode(NodeCount-1);
					Node RCp = null;
					Boolean EndFound = false;
					Start.setAttribute("Hn", SetHn(Start, End));
					Open.push(Start);
					Open.push(graph.getNode(0));
					graph.nodes().forEach(n ->{
						graph.getNode(n.getId()).setAttribute("Hn", SetHn(n, End));//Sets Heuristic Values
						if(n.getNumber("ID") == 3) {
							RC.push(graph.getNode(n.getId()));
						}
					});
					RCp = graph.getNode(0);
					Node Current = graph.getNode(0);
					Node rc = null;
					B = FullTank;
					outer:
						while(true) {//Main Loop to run through
							System.out.println("Loop");
							rc = graph.getNode(NextStation(RCp, End).getId());
							if(rc == null) {
								EndFound = false;
								break outer;
							}
							graph.nodes().forEach(n ->{
								n.setAttribute("Hn", SetHn(n, End));//Sets Heuristic Values
							});
							Current.setAttribute("Cost", 0);
							Current = graph.getNode(PathFind(Current, rc).getId()); //Gets the Next RC point
							if(Current == null) {
								EndFound = false;
								break outer;
							}
							RCP.push(graph.getNode(RCp.getId()));
							if(graph.getNode(rc.getId()) == graph.getNode(End.getId())) {//Printing results section
								Start.setAttribute("ui.style", " fill-color: rgb(0,0,255);");
								End.setAttribute("ui.style", " fill-color: rgb(255,0,0);");
								timeEnd = System.currentTimeMillis();
								timeRun = timeEnd - timeStart;
								System.out.println("Found End");
								//System.out.println("Best Path Cost  = "+ BestPath.size());
								System.out.println("Best Path Cost  = "+ End.getNumber("Gn"));
								System.out.println("Run Time = " + timeRun);
								System.out.println("Stations Visted = "+StationCounter);
								EndFound = true;
								/*try { //For writing to data file
									//System.out.println(results[0]+","+results[1]+","+results[2]);
									pw.write(results[0]+","+results[1]+","+results[2]+"\n");
								} catch (IOException e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}*/
								break outer;
							}
							RCp = graph.getNode(rc.getId());
							Current = graph.getNode(rc.getId());
							StationCounter++;
							RC.remove(RCindex);
							B = FullTank;
							Open.clear();
							Closed.clear();
						}
					RC.clear();
					Closed.clear();
					Open.clear();
					RCP.clear();
					BestPath.clear();
					timeEnd = 0;
					timeEnd = 0;
					timeRun = 0;
					StationCounter = 0;
					TrialCounter++;
					if(EndFound == false) {
						System.out.println("No Solution");
						z++;
					}else {
						//RoomLayot.FileClose();
						z++;
					}
					/*for(int i = 0; i < Open.size(); i++) {
					Open.pop();
					}*/
				}
				/*try {
					pw.close();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}*/
			//}
			System.out.println("Enter 0 to quit or anything else to run it again");
			int quit = scan.nextInt();
			if (quit == 0) {
				scan.close();
				System.exit(0);
			}
		}
	}
}