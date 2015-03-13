

import java.util.*;
import java.awt.Cell;



/**
 * Defines a Quagent capable of finding and navigating to tofu in undisclosed
 *	locations, inside a room of unknown layout
 */
class AStar extends Quagent{
	
	/**
	 * Size of each cell in the graph
	 */
	final static int CELL_SIZE = 64;
	
	/**
	 * Graph to hold a map of the room
	 */
	private static Graph room;
	
	/**
	 * Array list of explored cells
	 */
	private ArrayList<Cell> explored;
	
	/**
	 * Instance variable for the next cell to iterate through
	 */
	private Cell next;
	
	/**
	 * Path
	 */
	private Stack<Cell> path;
	
	/**
	 * Events
	 */
	private Events events;
	
	/**
	 * Reference frame of the quagent
	 */
	private double x,y,z,roll,pitch,yaw,velocity;
	
	/**
	 * Enumeration to describe the state of the agent
	 */
	private enum State{
		START, CHASING
	}
	
	/**
	 * State of the ngent
	 */
	private State state;
	
	/**
	 * Actual implementation of the A* algorithm
	 */
	private Stack a_star(Cell start, Cell goal){
		// The set of nodes already evaluated.
		ArrayList<Cell> closedset = new ArrayList<Cell>();
		ArrayList<Cell> openset = new ArrayList<Cell>();
		
		// The set of tentative nodes to be evaluated,
		//	initially containing the start node
		openset.add(start);
		
		// Nodes mapped to their parents
		HashMap<Cell, Cell> cameFrom = new HashMap<Cell, Cell>();
		
		// Table of g (distance from start) and f scores
		HashMap<Cell, Integer> g_score = new HashMap<Cell, Integer>(); 
		HashMap<Cell, Integer> f_score = new HashMap<Cell, Integer>(); 
		
		// The initial g score is zero
		g_score.put(start, new Integer(0));    // Cost from start along best known path.
		f_score.put(start, new Integer(g_score.get(start) + (int)start.distance(goal)));
	 
		// Iterate until we are certain that no path was found
		while (!openset.isEmpty()){
			
			// Get the cell witht eh best f score in the open list
			Cell current = openset.get(0);
			Integer current_f = f_score.get(current);
			for (Cell p : openset){
				if(f_score.get(p) < current_f){
					current_f = f_score.get(p);
					current = p;
				}
			}
			
			// If the current node is the goal, we are done
			if (current.equals(goal)){
				return reconstruct_path(cameFrom, goal);
			}
			
			// Otherwise, take the cell off the open list and
			//	add it to the closed list
			openset.remove(current);
			closedset.add(current);
			
			System.out.println("Hey " + current);
			
			// Go through each neighbor of the cell
			for (Cell neighbor : room.getNeighbors(current)){
				
				// Skip cells already in the closed list
				if (closedset.contains(neighbor))
					continue;
				
				// Calculate the g score
				Integer tentative_g_score = new Integer(g_score.get(current) + (int)current.distance(neighbor));
				
				// If going through the current node is better than going
				//	through the neighbor's current parent, change its parent
				if (!openset.contains(neighbor) || tentative_g_score < g_score.get(neighbor)){
					
					cameFrom.put(neighbor, current);
					g_score.put(neighbor, tentative_g_score);
					f_score.put(neighbor, new Integer(g_score.get(neighbor) + (int)neighbor.distance(goal)));
					
					// Add the neighbor to the open list if it isn't already
					if (!openset.contains(neighbor))
						openset.add(neighbor);
					
				}
			}
		}
		return null;
	}
	
	/**
	 * Reconstructs the path from the current cell through the passed in map
	 *	of parents
	 */
	private Stack reconstruct_path(HashMap<Cell, Cell> cameFrom, Cell current){
		Stack<Cell> total_path = new Stack<Cell>();
		total_path.push(current);
		while (cameFrom.containsKey(current)){
			current = cameFrom.get(current);
			total_path.push(current);
		}
		return total_path;
	}
	
	/**
	 * Main method just creates the room and the quagent
	 */
    public static void main(String[] args) throws Exception {
	
		// Build the room
		room = new Graph();
		
		// Make quagent
		new AStar();
    }
	
	/**
	 *
	 */
    AStar() throws Exception {
		super();
		try {
			// connect to a new quagent
			
			this.where();
			// loop forever -- that is until the bot dies of old age
			while(true) {
				// handle the events
				events = this.events();
				//printEvents(events);
				parseWalkEvents(events);
			}
		}	 
		catch (QDiedException e) { // the quagent died -- catch that exception
			System.out.println("bot died!");
		}
		catch (Exception e) { // something else went wrong???
			System.out.println("system failure: "+e);
			System.exit(0);
		}
    }
	
	/**
	 * Simple event printing method
	 */
    public void printEvents(Events events) {
		System.out.println("List of Events:");
		for (int ix = 0; ix < events.size(); ix++) {
			System.out.println(events.eventAt(ix));
		}
    }
	
	/**
	 * Method to handle most of the work of the program
	 */
    public void parseWalkEvents(Events events) {
		for (int ix = 0; ix < events.size(); ix++) {
			String e = events.eventAt(ix);
			//printEvents(events);
			try{
				if (e.indexOf("getwhere") >= 0) {
					String[] tokens = e.split("[()\\s]+");

					x = Double.parseDouble(tokens[3]);
					y = Double.parseDouble(tokens[4]);
					z = Double.parseDouble(tokens[5]);
					roll = Double.parseDouble(tokens[6]);
					pitch = Double.parseDouble(tokens[7]);
					yaw = Double.parseDouble(tokens[8]);
					velocity = Double.parseDouble(tokens[9]);
					
					//If we don't have a path, find a new one
					if(path == null){
						path = a_star(new Cell((((int)x/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), (((int)y/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2)), dest);
					}
					//If there is still no path, one doesn't exist. Self destruct
					if(path == null){
						System.out.println("No path");
						System.exit(0);
					}
					//We have a path, so pop off the next location and go there
					if(!path.isEmpty()){
						next = path.pop();
						double x2 = next.x - x;
						double y2 = next.y - y;
						double angle = Math.toDegrees(Math.atan2(y2, x2)) - this.pitch;
						this.turn((int)angle);
					}
				}	
				if (e.indexOf("STOPPED") >= 0) {
					this.where();
				}
				
				//Now we are facing the target so we can walk there
				if (e.indexOf("turnby") >= 0) {
					double x2 = next.x - x;
					double y2 = next.y - y;
					double dist = Math.sqrt(x2*x2 + y2*y2);
					this.walk((int)dist);
				}
			}
			catch (QDiedException er) { // the quagent died -- catch that exception
				System.out.println("bot died!");
			}
			catch (Exception er) { // something else went wrong???
				System.out.println("system failure: "+er);
				System.exit(0);
			}
		}
    }
}


