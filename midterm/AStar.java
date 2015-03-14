import java.util.*;


class AStar extends Quagent{
	final static int CELL_SIZE = 64;
	private static Graph room;
	private Cell next;
	private Stack<Cell> path;
    private Events events;
	private Cell location;
	private double x,y,z,roll,pitch,yaw,velocity;
 
	private enum State{
		START, SEARCHING
	}
	private State state = State.START;
	
	
	
	private Stack a_star(Cell start, Cell goal){
		ArrayList<Cell> closedset = new ArrayList<Cell>();    // The set of nodes already evaluated.
		ArrayList<Cell> openset = new ArrayList<Cell>(); 
		openset.add(start);    // The set of tentative nodes to be evaluated, initially containing the start node
		HashMap<Cell, Cell> cameFrom = new HashMap<Cell, Cell>();    // The map of navigated nodes.
		HashMap<Cell, Integer> g_score = new HashMap<Cell, Integer>(); 
		HashMap<Cell, Integer> f_score = new HashMap<Cell, Integer>(); 
		
		g_score.put(start, new Integer(0));    // Cost from start along best known path.
		f_score.put(start, new Integer(g_score.get(start) + (int)start.distance(goal)));
	 
		while (!openset.isEmpty()){
			Cell current = openset.get(0);
			Integer current_f = f_score.get(current);
			for (Cell p : openset){
				if(f_score.get(p) < current_f){
					current_f = f_score.get(p);
					current = p;
				}
			}
			if (current.equals(goal)){
				return reconstruct_path(cameFrom, goal);
			}
			openset.remove(current);
			closedset.add(current);
			//System.out.println("Hey " + current);
			for (Cell neighbor : room.getNeighbors(current)){
				if (closedset.contains(neighbor)){
					continue;
				}
				Integer tentative_g_score = new Integer(g_score.get(current) + (int)current.distance(neighbor));
				
				if (!openset.contains(neighbor) || tentative_g_score < g_score.get(neighbor)){
					cameFrom.put(neighbor, current);
					g_score.put(neighbor, tentative_g_score);
					f_score.put(neighbor, new Integer(g_score.get(neighbor) + (int)neighbor.distance(goal)));
					if (!openset.contains(neighbor)){
						openset.add(neighbor);
					}
				}
			}
		}
		return null;
	}
	
	private Stack reconstruct_path(HashMap<Cell, Cell> cameFrom, Cell current){
		Stack<Cell> total_path = new Stack<Cell>();
		total_path.push(current);
		while (cameFrom.containsKey(current)){
			current = cameFrom.get(current);
			total_path.push(current);
		}
		return total_path;
	}
	
	
	
    public static void main(String[] args) throws Exception {
	
		//Build the room
		room = new Graph(CELL_SIZE);
		
		//Make quagent
		new AStar();
    }

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

    public void printEvents(Events events) {
		System.out.println("List of Events:");
		for (int ix = 0; ix < events.size(); ix++) {
			System.out.println(events.eventAt(ix));
		}
    }

    public void parseWalkEvents(Events events) {
		for (int ix = 0; ix < events.size(); ix++) {
			String e = events.eventAt(ix);
			printEvents(events);
			try{
				switch (this.state) {
				case START:
					if (e.indexOf("getwhere") >= 0) {
						String[] tokens = e.split("[()\\s]+");

						x = Double.parseDouble(tokens[3]);
						y = Double.parseDouble(tokens[4]);
						z = Double.parseDouble(tokens[5]);
						roll = Double.parseDouble(tokens[6]);
						pitch = Double.parseDouble(tokens[7]);
						yaw = Double.parseDouble(tokens[8]);
						velocity = Double.parseDouble(tokens[9]);
						
						location = new Cell((((int)x/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), (((int)y/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2));
						room.addVertex(location);
						this.rays(15);
					}
					
					if (e.indexOf("rays") >= 0) {
						String[] tokens = e.split("[()\\s]+");
						for (int i = 0; i<((tokens.length - 4)/5); i++){
								int base = 5+5*i;
								Cell.Contents contents = Cell.Contents.EMPTY;
								if(tokens[base].equals("world_spawn")){
									contents = Cell.Contents.WALL;
								}
								else if(tokens[base].equals("TOFU")){
									contents = Cell.Contents.TOFU;
								}
								double ray_x = Double.parseDouble(tokens[base+1]) + x;
								double ray_y = Double.parseDouble(tokens[base+2]) + y;
								room.addVertex(new Cell((((int)ray_x/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), (((int)ray_y/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), contents));
								room.addLine(location,
													new Cell((((int)ray_x/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), (((int)ray_y/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2)));
			
						}
						//room.print();
						path = a_star(location, room.getFarthestUnexplored(location));
						state = State.SEARCHING;
						
						//If we don't have a path, find a new one
						if(path == null){
							System.out.println("Destination: " + room.getFarthestUnexplored(location));
							path = a_star(location, room.getFarthestUnexplored(location));
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
					break;
				case SEARCHING:
					if (e.indexOf("getwhere") >= 0) {
						String[] tokens = e.split("[()\\s]+");

						x = Double.parseDouble(tokens[3]);
						y = Double.parseDouble(tokens[4]);
						z = Double.parseDouble(tokens[5]);
						roll = Double.parseDouble(tokens[6]);
						pitch = Double.parseDouble(tokens[7]);
						yaw = Double.parseDouble(tokens[8]);
						velocity = Double.parseDouble(tokens[9]);
						
						location = new Cell((((int)x/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), (((int)y/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2));
						room.markExplored(location);
						this.rays(15);
					}
					
					if (e.indexOf("rays") >= 0) {
						String[] tokens = e.split("[()\\s]+");
						for (int i = 0; i<((tokens.length - 4)/5); i++){
								int base = 5+5*i;
								Cell.Contents contents = Cell.Contents.EMPTY;
								if(tokens[base].equals("world_spawn")){
									contents = Cell.Contents.WALL;
								}
								else if(tokens[base].equals("TOFU")){
									contents = Cell.Contents.TOFU;
								}
								double ray_x = Double.parseDouble(tokens[base+1]) + x;
								double ray_y = Double.parseDouble(tokens[base+2]) + y;
								room.addVertex(new Cell((((int)ray_x/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), (((int)ray_y/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), contents));
								room.addLine(location,
													new Cell((((int)ray_x/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), (((int)ray_y/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2)));
			
						}
						
						
						//If we don't have a path, find a new one
						if(path == null || path.isEmpty()){
							//System.out.println("Destination: " + room.getFarthestUnexplored(location));
							path = a_star(location, room.getFarthestUnexplored(location));
						}
						//If there is still no path, one doesn't exist. Self destruct
						if(path == null || path.isEmpty()){
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
					break;
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


