/*
*	A* and Adaptive A* search algorithms to solve a navigation problem of a game character in a grid world
*
*   @author: Sampat Biswas
*   Copyright 2011 Sampat Biswas
*
*	This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*	
*
*/
package cs561.biswas;

import java.io.*;
import java.util.Collections;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.ArrayList;

public class Astar {

	static final int MAX_ROW = 10;
	static final int MAX_COL = 10;
	
	static int Ai=-1, Aj=-1, Ti=-1, Tj=-1;
	
	static int a[][]= new int[MAX_ROW][MAX_COL];
	static int cost_grid[][] = new int [MAX_ROW][MAX_COL];
	static int fringe[][] = new int[MAX_ROW][MAX_COL];
	
	static Node nodes[][] = new Node[MAX_ROW][MAX_COL];
	
	static int rows=0, cols=0, nodesVisitedCount=0;
	static String fileName="";
	static boolean isForward, isBackward;
	static boolean isSmallerGvalue, isLargerGvalue;
	static boolean isAdaptiveSearch;
	
	static ArrayList<String> firstPresumedPath  = new ArrayList<String>();
	static ArrayList<String> presumedPath  = new ArrayList<String>();
	static ArrayList<String> newPresumedPath  = new ArrayList<String>();
	static ArrayList<String> walkPath  = new ArrayList<String>();
	static ArrayList<String> blocked_cells  = new ArrayList<String>();
	static ArrayList<Node> open = new ArrayList<Node>();
	static ArrayList<Node> closed = new ArrayList<Node>();
	static ArrayList<Node> parent = new ArrayList<Node>();
	static ArrayList<Node> successors = new ArrayList<Node>();
	
	public static void main(String[] args) {
		
		isForward=isBackward=isSmallerGvalue=isLargerGvalue=isAdaptiveSearch=false;
		
		// An input file name argument must be passed in the command line
		if(args.length == 0) {
			System.err.println("Error: No input file name passed in command line.");
			return;
		}
		
		int l=0;
		
		while( l < args.length ) {
			if(args[l].equalsIgnoreCase("-i")) {
				fileName = args[l+1];
			}
			if(args[l].equalsIgnoreCase("-d")) {
				if(args[l+1].equalsIgnoreCase("f"))
					isForward = true;
				else if(args[l+1].equalsIgnoreCase("b"))
					isBackward = true;
			}
			if(args[l].equalsIgnoreCase("-t")) {
				if(args[l+1].equalsIgnoreCase("s"))
					isSmallerGvalue = true;
				else if(args[l+1].equalsIgnoreCase("l"))
					isLargerGvalue = true;
			}
			if(args[l].equalsIgnoreCase("-A")) {
				isAdaptiveSearch = true;
			}
			l = l+2;
		}
		
		//fileName = "inputfile2.txt";
		
		try{
			FileInputStream fstream = new FileInputStream(fileName);
			DataInputStream in = new DataInputStream(fstream);
			BufferedReader br = new BufferedReader(new InputStreamReader(in));
			String strLine;
			int i=0,j=0, n=0, count=0;
			while ((strLine = br.readLine()) != null)   {
				if(count == 0) {
					// WIDTH
					String tokens[] = strLine.split(" ");
					cols = Integer.parseInt(tokens[1]);
					count++;
				}
				else if(count == 1) { 
					// HEIGHT
					String tokens[] = strLine.split(" ");
					rows = Integer.parseInt(tokens[1]);
					count++;
				}
				else if(count == 2) {
					// agent location
					String tokens[] = strLine.split(" ");
					String numTokens [] = tokens[1].split(",");
					Ai = Integer.parseInt(numTokens[0]);
					Aj = Integer.parseInt(numTokens[1]);
					count++;
				}
				else if(count == 3) {
					// target location
					String tokens[] = strLine.split(" ");
					String numTokens [] = tokens[1].split(",");
					Ti = Integer.parseInt(numTokens[0]);
					Tj = Integer.parseInt(numTokens[1]);
					count++;
				}
				else if(count == 4) {
					// configuration
					count++;
				}
				else if(count == 5) {
			      String tokens[] = strLine.split("\t");
			      for(n=0,j=0; n<tokens.length; n++, j++) {
					a[i][j] = Integer.parseInt(tokens[n]);
			      }
			      i++;
				  if(i == rows)
					break;
				}
			}
			in.close();
		}catch (Exception e){
			System.err.println("Error: " + e.getMessage());
			return;
		}
		
		int p,q,r,s;
		
		for(p=rows-1, r=0; p>=0; p--, r++) {
			for(q=0, s=0; q<cols; q++, s++) {
				fringe[q][p] = 1;
			}
		}
		//isBackward = true;
		//isAdaptiveSearch = true;
		//isLargerGvalue = true;
		
		if(isBackward) {
			// Interchange Agent and Target locations
			int temp;
			temp = Ai;
			Ai = Ti;
			Ti = temp;
			temp = Aj;
			Aj = Tj;
			Tj = temp;
		}
		
		System.out.println("Ai=" + Ai + " Aj=" + Aj + " Ti=" + Ti + " Tj=" + Tj);
		
		int Lii=-1, Ljj=-1, Rii=-1, Rjj=-1, Uii=-1, Ujj=-1, Dii=-1, Djj=-1;
		
		Lii=Ai-1; Ljj=Aj;
		Rii=Ai+1; Rjj=Aj;
		Uii=Ai; Ujj=Aj-1;
		Dii=Ai; Djj=Aj+1;
		
		if(Lii<rows && Lii>-1 && Ljj<cols && Ljj>-1) {
			if(a[cols-1-Ljj][Lii] < 0) {
				fringe[Lii][Ljj] = 0;
				blocked_cells.add(Lii+","+Ljj);
			}
			else
				fringe[Lii][Ljj] = a[cols-1-Ljj][Lii];
		}
		
		if(Rii<rows && Rii>-1 && Rjj<cols && Rjj>-1) {
			if(a[cols-1-Rjj][Rii] < 0) {
				fringe[Rii][Rjj] = 0;
				blocked_cells.add(Rii+","+Rjj);
			}
			else
				fringe[Rii][Rjj] = a[cols-1-Rjj][Rii];
		}
		
		if(Uii<rows && Uii>-1 && Ujj<cols && Ujj>-1) {
			if(a[cols-1-Ujj][Uii] < 0) {
				fringe[Uii][Ujj] = 0;
				blocked_cells.add(Uii+","+Ujj);
			}
			else
				fringe[Uii][Ujj] = a[cols-1-Ujj][Uii];
		}
		
		if(Dii<rows && Dii>-1 && Djj<cols && Djj>-1) {
			if(a[cols-1-Djj][Dii]<0) {
				fringe[Dii][Djj] = 0;
				blocked_cells.add(Dii+","+Djj);
			}
			else
				fringe[Dii][Djj] = a[cols-1-Djj][Dii];
		}
		
		for(p=rows-1, r=0; p>=0; p--, r++) {
			for(q=0, s=0; q<cols; q++, s++) {
				nodes[q][p] = new Node(q, p, fringe[q][p]);
				if(a[r][s] == -1)
					nodes[q][p].heuristic = 500;
				else
					nodes[q][p].heuristic = h(q,p,Ti,Tj);
			}
		}
		
		nodes[Ai][Aj].parent = nodes[Ai][Aj];
		
		
		/////////////////////////////////////////////////////////////////////
		
		nodes[Ai][Aj].gcost = 0;
		nodes[Ai][Aj].fcost = nodes[Ai][Aj].gcost + nodes[Ai][Aj].heuristic;
		
		open.add(new Node(	nodes[Ai][Aj].x, 
							nodes[Ai][Aj].y, 
							nodes[Ai][Aj].gcost, 
							nodes[Ai][Aj].heuristic, 
							nodes[Ai][Aj].fcost,
							nodes[Ai][Aj].ncost));
		
		long start = System.currentTimeMillis();
		
		
		firstPresumedPath = AstarSearch(Ai, Aj);
		
		if(firstPresumedPath == null) {
			System.out.println("No path found from agent to target.");
			return;
		}
		
		for(int b=firstPresumedPath.size()-1; b>=0; b--) {
			//System.out.print(firstPresumedPath.get(b)+" ");
			String coordinates[] = firstPresumedPath.get(b).toString().split(",");
			cost_grid[Integer.parseInt(coordinates[0])][Integer.parseInt(coordinates[1])] = -10;
		}
		
		System.out.println("\nThe grid known to the agent with the shortest path after the search");
		System.out.print(" ");
		for(p=0; p<cols; p++)
			System.out.print(p);
		System.out.println();
		for(p=rows-1; p>=0; p--) {
			System.out.print(p);
			for(q=0; q<cols; q++) {
				if(q==Ai && p==Aj) {
						System.out.print("A");
				}
				else if(q==Ti && p==Tj) {
						System.out.print("T");
				}
				else if(cost_grid[q][p] == -10) {
					System.out.print(".");
				}
				else if(blocked_cells.contains(q+","+p)) {
					System.out.print("X");
				}
				else {
					System.out.print(" ");
				}
			}
			System.out.println();
		}
	
		for(int b=firstPresumedPath.size()-2; b>=0; b--) {
			presumedPath.add(firstPresumedPath.get(b));
		}
		
		
		walkPath.add(Ai + "," + Aj);
		
		
		/* Call A star method iteratively */
		
		while(presumedPath.size()!=0) {
			
			String coordinates[] = presumedPath.get(0).split(",");
			walkPath.add(presumedPath.get(0));
			presumedPath.remove(0);
			
			
			int Ax = Integer.parseInt(coordinates[0]), Ay = Integer.parseInt(coordinates[1]);
			int Lx=-1, Ly=-1, Rx=-1, Ry=-1, Ux=-1, Uy=-1, Dx=-1, Dy=-1;
			
			Lx=Ax-1; Ly=Ay;
			Rx=Ax+1; Ry=Ay;
			Ux=Ax; Uy=Ay-1;
			Dx=Ax; Dy=Ay+1;
			
			if(Lx<rows && Lx>-1 && Ly<cols && Ly>-1) {
				if(a[cols-1-Ly][Lx] < 0) {
					fringe[Lx][Ly] = 0;
					blocked_cells.add(Lx+","+Ly);
				}
				else
					fringe[Lx][Ly] = a[cols-1-Ly][Lx];
			}
			
			if(Rx<rows && Rx>-1 && Ry<cols && Ry>-1) {
				if(a[cols-1-Ry][Rx] < 0) {
					fringe[Rx][Ry] = 0;
					blocked_cells.add(Rx+","+Ry);
				}
				else
					fringe[Rx][Ry] = a[cols-1-Ry][Rx];
			}
			
			if(Ux<rows && Ux>-1 && Uy<cols && Uy>-1) {
				if(a[cols-1-Uy][Ux] < 0) {
					fringe[Ux][Uy] = 0;
					blocked_cells.add(Ux+","+Uy);
				}
				else
					fringe[Ux][Uy] = a[cols-1-Uy][Ux];
			}
			
			if(Dx<rows && Dx>-1 && Dy<cols && Dy>-1) {
				if(a[cols-1-Dy][Dx] < 0) {
					fringe[Dx][Dy] = 0;
					blocked_cells.add(Dx+","+Dy);
				}
				else 
					fringe[Dx][Dy] = a[cols-1-Dy][Dx];
			}
			
			
			for(p=rows-1, r=0; p>=0; p--, r++) {
				for(q=0, s=0; q<cols; q++, s++) {
					//nodes[q][p] = new Node(q, p, a[r][s]);
					nodes[q][p].ncost = fringe[q][p];
					nodes[q][p].parent = null;
					nodes[q][p].fcost = 0;
					//nodes[q][p].gcost = 0;
				}
			}
			
			if(isAdaptiveSearch) {							////////////////////////// Adaptive A*
				for(int n=0; n<closed.size(); n++) {
					Node t = closed.get(n);
					nodes[t.x][t.y].heuristic = nodes[Ti][Tj].gcost - nodes[t.x][t.y].heuristic;
				}
			}
			
			nodes[Ax][Ay].gcost = 0;
			nodes[Ax][Ay].fcost = nodes[Ax][Ay].gcost + nodes[Ax][Ay].heuristic;
			
			/*for(p=rows-1; p>=0; p--) {
				for(q=0, s=0; q<cols; q++, s++) {
					System.out.print(fringe[q][p] + "\t");
				}
				System.out.println();
			}*/
			
			open.clear();
			open.add(new Node(	nodes[Ax][Ay].x, 
								nodes[Ax][Ay].y, 
								nodes[Ax][Ay].gcost, 
								nodes[Ax][Ay].heuristic, 
								nodes[Ax][Ay].fcost,
								nodes[Ax][Ay].ncost));
			
			
			
			closed.clear();
			successors.clear();
			
			//System.out.println("Starting from : " + Ax + "," + Ay);
			
			newPresumedPath = AstarSearch(Ax, Ay);
			
			
			
			
			/*System.out.print("\nPath Found := ");
			for(int b=newPresumedPath.size()-1; b>=0; b--) {
				System.out.print( " -> " + newPresumedPath.get(b));
			}
			System.out.println("");*/
			
			if(!newPresumedPath.containsAll(presumedPath)) {
				//System.out.println("\nPresumed Path changed:");
				Collections.reverse(newPresumedPath);
				presumedPath = newPresumedPath;
				printPath(presumedPath, Ax, Ay);
				presumedPath.remove(0);
			}
		}
		
		/*System.out.println("------------------ Walked Path ---------------------");
		
		for(int b=0; b<walkPath.size();  b++) {
			System.out.print(" -> " + walkPath.get(b));
		}*/
		
		
		long end = System.currentTimeMillis();
		
		long time_period = end-start;
		
		System.out.println("\n\nTime taken by the algorithm = " + time_period);
		System.out.println("\nTotal nodes visited = " + nodesVisitedCount);
	}
	/* The Heuristic Function */
	public static double h(int Tx, int Ty, int Lx, int Ly) {
		//return Math.sqrt((Tx-Lx)*(Tx-Lx) + (Ty-Ly)*(Ty-Ly));
		return (Math.abs(Tx-Lx)+Math.abs(Ty-Ly));
	}
	
	public static void printPath(ArrayList<String> path, int Ax, int Ay) {
		int grid[][] = new int [MAX_ROW][MAX_COL];
		System.out.println("\nThe grid known to the agent with the shortest path after the search");
		for(int b=0; b<path.size(); b++) {
			//System.out.print(path.get(b)+" ");
			String coordinates[] = path.get(b).toString().split(",");
			grid[Integer.parseInt(coordinates[0])][Integer.parseInt(coordinates[1])] = -10;
		}
		
		int p,q;
		System.out.print(" ");
		for(p=0; p<cols; p++)
			System.out.print(p);
		System.out.println();
		for(p=rows-1; p>=0; p--) {
			System.out.print(p);
			for(q=0; q<cols; q++) {
				if(q==Ax && p==Ay) {
					if(isBackward)
						System.out.print("T");
					else
						System.out.print("A");
				}
				else if(q==Ti && p==Tj) {
					if(isBackward)
						System.out.print("A");
					else
						System.out.print("T");
				}
				else if(grid[q][p] == -10) {
					System.out.print(".");
				}
				else if(blocked_cells.contains(q+","+p)) {
					System.out.print("X");
				}
				else {
					System.out.print(" ");
				}
			}
			System.out.println();
		}
	}
	
	public static ArrayList<String> AstarSearch(int Ax, int Ay) {
		ArrayList<String> path = null;
		while(open.size() != 0) {
			Collections.sort(open);
			Node n = open.get(0);
			
			if(isSmallerGvalue || isLargerGvalue) {
				for(int i=1; i<open.size(); i++) {
					Node t = open.get(i);
					if(t.fcost == n.fcost) {
						if(isSmallerGvalue) {
							if(t.gcost < n.gcost) {
								n=t; 
								open.remove(i);
								break;
							}
							else {
								open.remove(0);
								break;
							}
						}
						else if(isLargerGvalue) {
							if(t.gcost > n.gcost) {
								n = t;
								open.remove(i);
								break;
							}
							else {
								open.remove(0);
								break;
							}
						}
					}
				}
				open.remove(0);
			}
			else {
				open.remove(0);
			}
			nodesVisitedCount++;
			int xopen = n.x;
			int yopen = n.y;
			//System.out.print("=>" + xopen + "," + yopen);
			if((xopen==Ti) && (yopen==Tj)) {
				// Reached Target
				path = new ArrayList<String>();
				path.add(xopen+","+yopen);
				int currx = xopen;
				int curry = yopen;
				while(currx!=Ax || curry!=Ay) {
					xopen = currx;
					yopen = curry;
					currx = nodes[xopen][yopen].parent.x;
					curry = nodes[xopen][yopen].parent.y;
					//System.out.println("Adding: " + currx+","+curry);
					path.add(currx+","+curry);
				}
				return path;
			}
			
			closed.add(n);
			
			int Lx=-1, Ly=-1, Rx=-1, Ry=-1, Ux=-1, Uy=-1, Dx=-1, Dy=-1;
			
			
			Rx = nodes[xopen][yopen].x + 1;
			Ry = nodes[xopen][yopen].y;
			Lx = nodes[xopen][yopen].x - 1;
			Ly = nodes[xopen][yopen].y;
			Ux = nodes[xopen][yopen].x;
			Uy = nodes[xopen][yopen].y + 1;
			Dx = nodes[xopen][yopen].x;
			Dy = nodes[xopen][yopen].y - 1;
			
			
			if(Ux<cols && Ux>=0 && Uy<rows && Uy>=0) {
				if((nodes[Ux][Uy].ncost != 0)) {
					successors.add(new Node(nodes[Ux][Uy].x, nodes[Ux][Uy].y, nodes[Ux][Uy].gcost, nodes[Ux][Uy].heuristic, nodes[Ux][Uy].fcost, nodes[Ux][Uy].ncost));
				}
			}
			
			if(Dx<cols && Dx>=0 && Dy<rows && Dy>=0) {
				if((nodes[Dx][Dy].ncost != 0)) {
					successors.add(new Node(nodes[Dx][Dy].x, nodes[Dx][Dy].y, nodes[Dx][Dy].gcost, nodes[Dx][Dy].heuristic, nodes[Dx][Dy].fcost, nodes[Dx][Dy].ncost));
				}
			}
			
			if(Rx<cols && Rx>=0 && Ry<rows && Ry>=0) {
				if((nodes[Rx][Ry].ncost != 0)) {
					successors.add(new Node(nodes[Rx][Ry].x, nodes[Rx][Ry].y, nodes[Rx][Ry].gcost, nodes[Rx][Ry].heuristic, nodes[Rx][Ry].fcost, nodes[Rx][Ry].ncost));
				}
			}
			
			if(Lx<cols && Lx>=0 && Ly<rows && Ly>=0) {
				if((nodes[Lx][Ly].ncost != 0)) {
					successors.add(new Node(nodes[Lx][Ly].x, nodes[Lx][Ly].y, nodes[Lx][Ly].gcost, nodes[Lx][Ly].heuristic, nodes[Lx][Ly].fcost, nodes[Lx][Ly].ncost));
				}
			}
			
			while(successors.size() != 0) {
				Node temp = successors.get(0);
				successors.remove(0);
				int xsuccessor = temp.x;
				int ysuccessor = temp.y;
				if(!closed.contains(temp)) {
					if(!open.contains(temp)) {
						nodes[xsuccessor][ysuccessor].gcost=500;
						nodes[xsuccessor][ysuccessor].parent = null;
					}
					// UPDATE SUCCESSOR
					if((nodes[xopen][yopen].gcost + nodes[xsuccessor][ysuccessor].ncost) <= nodes[xsuccessor][ysuccessor].gcost) {
						nodes[xsuccessor][ysuccessor].gcost = nodes[xopen][yopen].gcost + nodes[xsuccessor][ysuccessor].ncost;
						nodes[xsuccessor][ysuccessor].parent = nodes[xopen][yopen];
						if(open.contains(nodes[xsuccessor][ysuccessor])) {
							open.remove(nodes[xsuccessor][ysuccessor]);
						}
						if(isBackward) {
							nodes[xsuccessor][ysuccessor].heuristic = h(xsuccessor, ysuccessor, Ti, Tj);
						}
						nodes[xsuccessor][ysuccessor].fcost = 
							nodes[xsuccessor][ysuccessor].gcost + nodes[xsuccessor][ysuccessor].heuristic;
						/*newNode.fcost = nodes[xsuccessor][ysuccessor].fcost;
						open.add(newNode);*/
						open.add(new Node(	nodes[xsuccessor][ysuccessor].x,
											nodes[xsuccessor][ysuccessor].y,
											0,
											0,
											nodes[xsuccessor][ysuccessor].fcost,
											0));
					}
				}
			}
			
		}
		return path;
	}
	
}

class Node implements Comparable {

	public int x;

	public int y;

	public double gcost;

	public double heuristic;

	public double fcost;
	
	public int ncost;
	
	public Node parent;
	
	public Node(int x, int y) {
		this.x = x;
		this.y = y;
	}
	
	public Node(int x, int y, int ncost) {
		this.x = x;
		this.y = y;
		this.ncost = ncost;
	}
	
	public Node(int x, int y, double gcost, double heuristic, double fcost, int ncost) {
		this.x = x;
		this.y = y;
		this.gcost = gcost;
		this.heuristic = heuristic;
		this.fcost = fcost;
		this.ncost = ncost;
	}
	
	public int compareTo(Object other) {
		Node o = (Node) other;
		double f = heuristic + gcost;
		double of = o.heuristic + o.gcost;
		
		if (f < of) {
			return -1;
		} else if (f > of) {
			return 1;
		} else {
			return 0;
		}
	
	}
	
	public boolean equals(Object o) {
		if(!(o instanceof Node)){
			return false;
		}
		if((this.x==((Node)o).x) && (this.y==((Node)o).y)) {
			return true;
		}
		else
			return false;
	}
}
