import java.util.*;

import org.graphstream.graph.Element;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.SingleGraph;
import org.graphstream.algorithm.Toolkit;
import java.util.Stack;
import java.util.stream.Stream;
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
	public static double FullTank = 1;
	public static double RCpercentage = .7;
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
	
	
	public static Node NextStation(Node x, Node y) {//Method for finding the next RC point
		Node RCp = graph.getNode(x.getId());
		Node End = graph.getNode(y.getId());
		Node rc;
		graph.getNode(RCp.getId()).setAttribute("Hn", SetHn(graph.getNode(RCp.getId()), graph.getNode(End.getId())));
		if(HDist(graph.getNode(RCp.getId()), graph.getNode(End.getId())) <= FullTank) {
			double RCpValue = graph.getNode(RCp.getId()).getNumber("Hn");
			System.out.println("RC Node Hit is End Point: "+RCpValue);
			rc = graph.getNode(End.getId());
		} else {
			if(RC.size() == 0) {
				System.out.println("No more Recharge Stations");
				return null;
			}
			Node Temp = graph.getNode(RC.get(0).getId());
			graph.getNode(Temp.getId()).setAttribute("rcH", HDist(graph.getNode(RCp.getId()), graph.getNode(RC.get(0).getId())) 
					+ 2*HDist(graph.getNode(RC.get(0).getId()), graph.getNode(End.getId())) 
					- HDist(graph.getNode(RCp.getId()), graph.getNode(End.getId())));
			for(int i = 0; i < RC.size(); i++) {
				graph.getNode(RC.get(i).getId()).setAttribute("rcH", HDist(graph.getNode(RCp.getId()), graph.getNode(RC.get(i).getId())) 
						+ 2*HDist(graph.getNode(RC.get(i).getId()), graph.getNode(End.getId())) 
						- HDist(graph.getNode(RCp.getId()), graph.getNode(End.getId())));
				//System.out.println(RC.get(i).GetrcH());
				double TemprcH = graph.getNode(Temp.getId()).getNumber("rch");
				double RCrch = graph.getNode(RC.get(i).getId()).getNumber("rch");
				if(TemprcH >= RCrch && HDist(graph.getNode(RCp.getId()), graph.getNode(RC.get(i).getId())) <= B) {
					Temp = graph.getNode(RC.get(i).getId());
					RCindex = i;
				}
			}
			rc = graph.getNode(Temp.getId());
			double RCpValue =  graph.getNode(RCp.getId()).getNumber("Hn");
			System.out.println("RC Node Hit: "+RCpValue);
		}
		graph.getNode(rc.getId()).setAttribute("ui.style", "fill-color: rgb(255,203,182);");
		return graph.getNode(rc.getId());
	}
	
	
	public static double HDist(Node x, Node y) {//Method for Finding distance between two Nodes
		double H;
		Node Current = graph.getNode(x.getId());
		Node End = graph.getNode(y.getId());
		int D = 1;
		int D2 =1;
		double[] Endrelative = Toolkit.nodePosition(graph.getNode(End.getId()));
		double[] Currentrelative = Toolkit.nodePosition(graph.getNode(Current.getId()));
		double dx = Math.abs(Currentrelative[0] - Endrelative[0]);
		double dy = Math.abs(Currentrelative[0] - Endrelative[0]);
		H = D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
		//H = Math.max(Math.abs(End.GetCol() - Start.GetCol()), Math.abs(End.GetRow() - Start.GetRow()));
		//return H;
		return Math.hypot(dx, dy);
	}
	
	public static Node PathFind(Node x, Node y) { //Method for Path finding from Current to RC
		int TempIndex = 0;
		Node Current = graph.getNode(x.getId());
		Node End = graph.getNode(y.getId());
		Node Temp = null;
		Node Initial = graph.getNode(Current.getId());
		Open.push(graph.getNode(Current.getId()));
		graph.getNode(Current.getId()).setAttribute("Cost", 0);
		while(Open.empty() == false) {//Start of A*
			if(B < 0){
				System.out.println("No solution under cost.");
				return null;
			}
			TempIndex = 0;
			Current = graph.getNode(Open.get(0).getId());
			//finds the least FN node in OPEN
			for(int i = 1; i < Open.size(); i++) {
				graph.getNode(Open.get(i).getId()).setAttribute("Hn", SetHn(graph.getNode(Open.get(i).getId()), graph.getNode((End).getId())));
				graph.getNode(Open.get(i).getId()).setAttribute("Fn", SetFn(graph.getNode(Open.get(i).getId())));
				graph.getNode(Current.getId()).setAttribute("Hn", SetHn(graph.getNode(Current.getId()), graph.getNode(End.getId())));
				graph.getNode(Current.getId()).setAttribute("Fn", SetFn(graph.getNode(Current.getId())));
				//System.out.println("This the Fn back in the loop: "+ graph.getNode(Current.getId()).getNumber("Fn"));
				double a = graph.getNode(Open.get(i).getId()).getNumber("Fn");
				double b = graph.getNode(Current.getId()).getNumber("Fn");
				if(a < b) {
					Current = graph.getNode(Open.get(i).getId());
					TempIndex = i;
				}
			}
			//checks if the minimum FN node in OPEN the goal node or not
			if (graph.getNode(Current.getId()).getId() == graph.getNode(End.getId()).getId()) {
				//Initial = graph.getNode(x.getId());
				System.out.println("End node is "+ graph.getNode(End.getId()).getId());
				Temp = graph.getNode(Current.getId());
				BestPath.push(graph.getNode(Current.getId()));
				// AD: this next line seems to be wrong as the HDist between current and End is 0 -- they are the SAME NODES
				graph.getNode(End.getId()).setAttribute("Gn", graph.getNode(Current.getId()).getNumber("Gn") 
						+ HDist(graph.getNode(Current.getId()), graph.getNode(End.getId())));
				//Node Previous = graph.getNode(((Node) graph.getNode(Current.getId()).getAttribute("Previous")).getId());
				Node Previous =  graph.getNode(Current.getId());
				//while(!graph.getNode(((Node) graph.getNode(Current.getId()).getAttribute("Previous")).getId()).equals(graph.getNode(Initial.getId()))) {//Infinite Loop is here. Not always recognizing initial for some reason
				//while(graph.getNode(Previous.getId()).getId() != graph.getNode(Initial.getId()).getId()){	
				while(Previous.getAttribute("Parent") != Initial.getId()) {
					//System.out.println("test "+ graph.getNode(Previous.getId()).getId() +" and "+ graph.getNode(Initial.getId()).getId());
					Cost = Cost + graph.getNode(Previous.getId()).getNumber("Hn");
					BestPath.push(graph.getNode(Previous.getId()));
					//graph.getNode(Current.getId()).getEdgeBetween(graph.getNode(Previous.getId())).setAttribute("ui.style", " fill-color: rgb(0,0,255);");
					graph.getNode(Previous.getId()).setAttribute("ui.style", " fill-color: rgb(0,0,255);");
					//Previous =  graph.getNode(((Node) Previous.getAttribute("Previous")).getId());
					Previous = graph.getNode((String) Previous.getAttribute("Parent"));
					//System.out.println("Previous node is: "+Previous.getId());
					//Current = graph.getNode(Previous.getId());
//					if(Previous == null) {
//						System.out.println("Previous is null");
//						return graph.getNode(Temp.getId());
//					}
				}
				return graph.getNode(Temp.getId());
			}
			/*
			if(graph.getNode(Current.getId()).getAttribute("Previous") != null) {
				Node Previous = graph.getNode(((Node) graph.getNode(Current.getId()).getAttribute("Previous")).getId());
				double Pcost = graph.getNode(Previous.getId()).getNumber("Cost");
				graph.getNode(Current.getId()).setAttribute("Cost", Pcost+ HDist(graph.getNode(Current.getId()), graph.getNode(Previous.getId())));
			}else {
				graph.getNode(Current.getId()).setAttribute("Cost", 0);
			}
			*/
			if(RC.contains(graph.getNode(Current.getId()))) {
				B = FullTank;
			}else {
				B = B - graph.getNode(Current.getId()).getNumber("Cost");
			}
			System.out.println("Fuel: "+(B));
			Temp = graph.getNode(Open.get(TempIndex).getId());
			Open.remove(graph.getNode(Current.getId()));
			Closed.push(graph.getNode(Current.getId()));
			//int NeighborCount = graph.getNode(Current.getId()).getDegree();
			Node Neighbor;
			Stream<Node> Neighborlist = graph.getNode(Current.getId()).neighborNodes();//Look into converting Stream to ArrayList
			Iterator<Node> IT = Neighborlist.iterator();
			while(IT.hasNext()){
				//if(graph.getNode(Current.getId()).getEdge(i) == null) continue;
				//Neighbor = graph.getNode(graph.getNode(Current.getId()).getEdge(i).getOpposite(graph.getNode(Current.getId())).getId());
				Neighbor = IT.next();
				if(Neighbor == null || Closed.contains(graph.getNode(Neighbor.getId()))) continue;
				graph.getNode(Neighbor.getId()).setAttribute("Parent", Current.getId());
				System.out.println("parent of Node "+Neighbor.getId()+" is "+graph.getNode(Neighbor.getId()).getAttribute("Parent"));
				double Gn = graph.getNode(Current.getId()).getNumber("Gn");
				graph.getNode(Neighbor.getId()).setAttribute("Gn", Gn + HDist(graph.getNode(Current.getId()), graph.getNode(Neighbor.getId())));
				if (Open.contains(graph.getNode(Neighbor.getId()))) {
						continue;
						// AD: we should be updating the cost here of neighbor if a better cost path is found, right?
				}else {
				//					if(Closed.contains(graph.getNode(Neighbor.getId()))) {
				//						continue;
				//					}
					graph.getNode(Neighbor.getId()).setAttribute("ui.style", "fill-color: rgb(0,255,0);");
					Open.push(graph.getNode(Neighbor.getId()));
				}
			}
			Closed.push(graph.getNode(Current.getId()));
			/*try {
				TimeUnit.MILLISECONDS.sleep(60);
			} catch (InterruptedException e) {
				e.printStackTrace();
	        	}*/
		}//end of Search
		System.out.println("PathFind Failed");
		return null;
	}
	
	private static double SetHn(Node x, Node y) {//Method for Setting Heuristic Value
		Node Current = graph.getNode(x.getId());
		Node End = graph.getNode(y.getId());
		int D = 1;
		int D2 =1;
		double[] Endrelative = Toolkit.nodePosition(graph.getNode(End.getId()));
		double[] Currentrelative = Toolkit.nodePosition(graph.getNode(Current.getId()));
		double dx = Math.abs(Currentrelative[0] - Endrelative[0]);
		double dy = Math.abs(Currentrelative[0] - Endrelative[0]);
		//return D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
		return Math.hypot(dx, dy);
	}
	
	private static double SetFn(Node x) {//Method for Setting F Value
		Node Current = graph.getNode(x.getId());
		double Hn = graph.getNode(Current.getId()).getNumber("Hn");
		//System.out.println("This is the Hn in the method Fn "+ Current.getNumber("Hn"));
		double Gn = graph.getNode(Current.getId()).getNumber("Gn");
		//System.out.println("This is the Gn in the method Fn "+ Current.getNumber("Gn"));
		//System.out.println("This is the Fn in the method Fn "+ (Hn + Gn));
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
					Start.setAttribute("Hn", SetHn(graph.getNode(Start.getId()), graph.getNode(End.getId())));
					Open.push(graph.getNode(Start.getId()));
					//Open.push(graph.getNode(0));
					graph.nodes().forEach(n ->{
						graph.getNode(n.getId()).setAttribute("Hn", SetHn(graph.getNode(n.getId()), graph.getNode(End.getId())));//Sets Heuristic Values
						if(graph.getNode(n.getId()).getNumber("ID") == 3) {
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
							rc = graph.getNode(NextStation(graph.getNode(RCp.getId()), graph.getNode(End.getId())).getId());
							if(graph.getNode(rc.getId()) == null) {
								EndFound = false;
								break outer;
							}
							graph.nodes().forEach(n ->{
								n.setAttribute("Hn", SetHn(graph.getNode(n.getId()), graph.getNode(End.getId())));//Sets Heuristic Values
							});
							graph.getNode(Current.getId()).setAttribute("Cost", 0);
							Current = graph.getNode(PathFind(graph.getNode(Current.getId()), graph.getNode(rc.getId())).getId()); //Gets the Next RC point
							if(graph.getNode(Current.getId()) == null) {
								EndFound = false;
								break outer;
							}
							RCP.push(graph.getNode(RCp.getId()));
							if(graph.getNode(rc.getId()).equals(graph.getNode(End.getId()))) {//Printing results section
								graph.getNode(Start.getId()).setAttribute("ui.style", " fill-color: rgb(0,0,255);");
								graph.getNode(End.getId()).setAttribute("ui.style", " fill-color: rgb(255,0,0);");
								timeEnd = System.currentTimeMillis();
								timeRun = timeEnd - timeStart;
								System.out.println("Found End");
								//System.out.println("Best Path Cost  = "+ BestPath.size());
								System.out.println("Best Path Cost  = "+ graph.getNode(End.getId()).getNumber("Gn"));
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
							System.out.println("End of Main loop test");
							try {
								Thread.sleep(100);
							} catch( InterruptedException e) {
								e.printStackTrace();
							}
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