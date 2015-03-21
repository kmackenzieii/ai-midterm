import java.util.*;


/**
 * Defines a Quagent capable of finding and navigating to tofu in undisclosed
 *	locations, inside a room of unknown layout
 */
class AStar extends Quagent{
    
    /**
     * Size of each cell in the graph
     */
    final static int CELL_SIZE = 16;
    
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
     * True if the target is tofu
     */
    private boolean followingTofu = false;
    
    /**
     * Previous location of the quagent
     */
    private Cell last_location;
    
    /**
     * Current location of the quagent
     */
    private Cell location;
    
    /**
     * Current location of the quagent
     */
    private Cell target;
    
    /**
     * Events
     */
    private Events events;
    
    /**
     * For generating random numbers
     */
    Random rand = new Random();
    
    /**
     * Reference frame of the quagent
     */
    private double x,y,z,roll,pitch,yaw,velocity;
    
    /**
     * Enumeration to describe the state of the agent
     */
    private enum State{
        START, SEARCHING, UNSTICKING
    }
    
    /**
     * State of the ngent
     */
    private State state = State.START;
    
    /**
     * Actual implementation of the A* algorithm
     */
    private Stack<Cell> a_star(Cell start, Cell goal){
        // The set of nodes already evaluated.
        ArrayList<Cell> closedset = new ArrayList<Cell>();
        ArrayList<Cell> openset = new ArrayList<Cell>();
        
        if(start.equals(goal)){
            Stack<Cell> output = new Stack<Cell>();
            output.push(start);
            return output;
        }
        
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
            
            //System.out.println("Hey " + current);
            
            // Go through each neighbor of the cell
            for (Cell neighbor : room.getNeighbors(current)){
                
                // Skip cells already in the closed list
                if (closedset.contains(neighbor))
                    continue;
                
                // Calculate the g score
                int wall_modifier = room.neighborIsWall(neighbor) ? 500 : 0;
                wall_modifier = wall_modifier + (neighbor.isWall() ? 10000 : 0);
                //int exploration_modifier = room.isUnexplored(neighbor) ? 4 : 1;
                Integer tentative_g_score = new Integer((g_score.get(current) + (int)current.distance(neighbor) + wall_modifier));
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
    
    private Stack<Cell> indiscretize(Cell start, Cell goal) {
        Stack<Cell> path = a_star(start, goal);
        Stack<Cell> indiscretized = new Stack<Cell>();
        
        Cell current = path.pop();
        Cell next = current;
        Cell looking = path.pop();
        indiscretized.push(current);
        
        while (!looking.equals(goal)) {
            Stack<Cell> b = bresenham(current, looking);
            while (!containsWall(b) && !looking.equals(goal)) {
                next = looking;
                looking = path.pop();
                b = bresenham(current, looking);
            }
            if (!looking.equals(goal))
                indiscretized.push(next);
            current = next;
        }
        indiscretized.push(goal);
        while (!indiscretized.isEmpty())
            path.push(indiscretized.pop());
        return path;
    }
    
    private Stack<Cell> bresenham(Cell c1, Cell c2) {
        
        // Stack to hold the cells
        Stack<Cell> line = new Stack<Cell>();
        
        // Get endpoint coordinates
        int y1 = fitToGrid(c1.y);
        int y2 = fitToGrid(c2.y);
        int x1 = fitToGrid(c1.x);
        int x2 = fitToGrid(c2.x);
        
        // loop counter
        int i;
        
        // the step on y and x axis
        int ystep, xstep;
        
        // the error accumulated during the increment
        int error;
        
        // vision the previous value of the error variable
        int errorprev;
        
        // the line points
        int y = y1, x = x1;
        
        // compulsory variables: the double values of dy and dx
        int ddy, ddx;
        
        // Differences
        int dx = x2 - x1;
        int dy = y2 - y1;
        
        // first point
        Cell c = room.getCellAtIndex(x1, y1);
        if (c != null)
            line.push(c);
        
        // If slope is negative
        if (dy < 0) {
            ystep = -1;
            dy = -dy;
        } else
            ystep = 1;
        
        // If line goes from right to left
        if (dx < 0) {
            xstep = -1;
            dx = -dx;
        } else
            xstep = 1;
        
        // work with double values for full precision
        ddy = 2 * dy;
        ddx = 2 * dx;
        
        // first octant (0 <= slope <= 1)
        if (ddx >= ddy){
            
            // compulsory initialization (even for errorprev, needed when dx==dy)
            
            // start in the middle of the square
            errorprev = error = dx;
            
            // do not use the first point (already done)
            for (i=0 ; i < dx ; i++){
                x += xstep;
                error += ddy;
                
                // increment y if AFTER the middle ( > )
                if (error > ddx){
                    y += ystep;
                    error -= ddx;
                    
                    // Bottom
                    if (error + errorprev < ddx) {
                        c = room.getCellAtIndex(x, y-ystep);
                        if (c != null)
                            line.push(c);
                    }
                    // Left
                    else if (error + errorprev > ddx) {
                        c = room.getCellAtIndex(x-xstep, y);
                        if (c != null)
                            line.push(c);
                    }
                    // Both (crosses corner exactly)
                    else {
                        c = room.getCellAtIndex(x, y-ystep);
                        if (c != null)
                            line.push(c);
                        c = room.getCellAtIndex(x-xstep, y);
                        if (c != null)
                            line.push(c);
                    }
                }
                c = room.getCellAtIndex(x, y);
                if (c != null)
                    line.push(c);
                errorprev = error;
            }
        }
        
        // Second octant (slope  > 1)
        else{
            errorprev = error = dy;
            for (i=0 ; i < dy ; i++){
                y += ystep;
                error += ddx;
                if (error > ddy){
                    x += xstep;
                    error -= ddy;
                    if (error + errorprev < ddy) {
                        c = room.getCellAtIndex(x-xstep, y);
                        if (c != null)
                            line.push(c);
                    }
                    else if (error + errorprev > ddy) {
                        c = room.getCellAtIndex(x, y-ystep);
                        if (c != null)
                            line.push(c);
                    }
                    else{
                        c = room.getCellAtIndex(x, y-ystep);
                        if (c != null)
                            line.push(c);
                        c = room.getCellAtIndex(x-xstep, y);
                        if (c != null)
                            line.push(c);
                    }
                }
                c = room.getCellAtIndex(x, y);
                if (c != null)
                    line.push(c);
                errorprev = error;
            }
        }
        // assert ((y == y2) && (x == x2));
        // the last point (y2,x2) has to be the same as the last point of the algorithm
        return line;
    }
    
    private boolean containsWall(Stack<Cell> line) {
        for (Cell c : line)
            if (c.isWall())
                return true;
        return false;
    }
    
    /**
     * Reconstructs the path from the current cell through the passed in map
     *	of parents
     */
    private Stack<Cell> reconstruct_path(HashMap<Cell, Cell> cameFrom, Cell current){
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
        
        //Build the room
        room = new Graph(CELL_SIZE);
        
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
            path = null;
            this.where();
			this.walk(10);
            // loop forever -- that is until the bot dies of old age
            while(true) {
                
                
                
                
                // handle the events
                //events = this.events();
                //printEvents(events);
                parseWalkEvents();
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
     * Snap the number to a valid grid index based on CELL_SIZE
     * Take into account sign of the point.
     */
    private int fitToGrid(double num) {
        return (((int)num/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2);
    }
    
    /**
     * Method to handle most of the work of the program
     */
    public void parseWalkEvents() {
        //System.out.println(this.state);
        //for (int ix = 0; ix < events.size(); ix++) {
        //    String e = events.eventAt(ix);
        //printEvents(events);
        try{
            this.where();
			this.rays(16);
            Events ev = null;
			int c = 0;
            // Must get current position
            boolean gotwhere = false;
            while (!gotwhere) {
                ev = concatEvents(ev, this.events());
                gotwhere = findEvents(ev, "getwhere").size() > 0;
            }
            boolean gotrays = false;
            while (!gotrays) {
                ev = concatEvents(ev, this.events());
                gotrays = findEvents(ev, "rays").size() > 0;
            }
			printEvents(ev);
            double[] ref = parseWhere(findEvents(ev, "getwhere"));
			last_location = location;
            location = new Cell(fitToGrid(ref[0]),
                                fitToGrid(ref[1]));
			room.addVertex(location);
            
            if (findEvents(ev, "radius").size() > 0) {
                room.markExplored(location,200);
            }
            
            double traveled = parseStopped(findEvents(ev,
                                                      "STOPPED"));
            if (traveled > -1 && traveled < 1) {
				ev = removeEvents(ev, "STOPPED");
                this.turn(45);
				boolean turned = false;
				while (!turned) {
					ev = concatEvents(ev, this.events());
					turned = findEvents(ev, "turnby").size() > 0;
				}
                this.walk(10);
            }
            
            parseRays(findEvents(ev, "rays"));
            double[] tofu = null;//parseRadius(findEvents(ev, "radius"));
            
            if (tofu != null) {
                target = new Cell(fitToGrid(tofu[0]),
                                  fitToGrid(tofu[1]));
				System.out.println("NEW PATH");
                path = indiscretize(location, target);
            } else if (path == null || path.isEmpty()) {
                target = room.getFarthestUnexplored(location);
				System.out.println("NEW PATH");
                path = indiscretize(location, target);
            }
			
			if (last_location != null && location.distance(last_location) == 0)
				ev.add("TELL STOPPED 0.0");
            
            if (traveled > 1) {
				if (next == null || location.distance(next) < 20)
                next = path.pop();
                double relx = next.x - x;
                double rely = next.y - y;
                double dist = location.distance(next);
                double angle = Math.atan2(rely, relx);
                angle = Math.toDegrees(angle) - ref[2];
                this.turn((int)angle);
				boolean turned = false;
				while (!turned) {
					ev = concatEvents(ev, this.events());
					turned = findEvents(ev, "turnby").size() > 0;
				}
                this.walk((int)dist);
            }
            
            room.printMap();
            this.radius(200);
            this.rays(16);
            /*
             switch (this.state) {
             case UNSTICKING:
             /*
             if (e.indexOf("STOPPED") >= 0) {
             String[] tokens = e.split("[()\\s]+");
             double dist = Double.parseDouble(tokens[2]);
             System.out.println(dist);
             //Guagent is stuck so unstick
             if (dist < 1.0){
             this.state = State.UNSTICKING;
             int random = rand.nextInt(270)+90;
             this.turn(random);
             }
             path = a_star(location, target);
             this.state = State.SEARCHING;
             this.where();
             }
             
             //Now we are facing the target so we can walk there
             if (e.indexOf("turnby") >= 0) {
             this.walk(4*CELL_SIZE);
             }
             
             break;
             case START:
             
             
             this.where();
             Events ev = null;
             
             // Must get current position
             boolean gotwhere = false;
             while (!gotwhere) {
             ev = concatEvents(ev, this.events());
             gotwhere = findEvents(ev, "getwhere").size() > 0;
             }
             double[] ref = parseWhere(findEvents(ev, "getwhere"));
             location = new Cell(fitToGrid(ref[0]),
             fitToGrid(ref[1]));
             
             room.addVertex(location);
             
             if (findEvents(ev, "radius").size() > 0) {
             room.markExplored(location,200);
             }
             
             parseRays(findEvents(ev, "rays"));
             double[] tofu = parseRadius(findEvents(ev, "radius"));
             
             if (tofu != null) {
             target = new Cell(fitToGrid(tofu[0]),
             fitToGrid(tofu[1]));
             path = indiscretize(location, target);
             } else if (path == null || path.isEmpty()) {
             target = room.getFarthestUnexplored(location);
             path = indiscretize(location, target);
             }
             
             if (findEvents(ev, "STOPPED").size() > 0) {
             next = path.pop();
             double relx = next.x - x;
             double rely = next.y - y;
             double dist = location.distance(next);
             double angle = Math.atan2(rely, relx);
             angle = Math.toDegrees(angle) - ref[2];
             this.turn((int)angle);
             this.walk((int)dist);
             }
             
             room.printMap();
             this.radius(200);
             this.rays(16);
             
             /*
             if (e.indexOf("getwhere") >= 0) {
             String[] tokens = e.split("[()\\s]+");
             
             x = Double.parseDouble(tokens[3]);
             y = Double.parseDouble(tokens[4]);
             z = Double.parseDouble(tokens[5]);
             roll = Double.parseDouble(tokens[6]);
             pitch = Double.parseDouble(tokens[7]);
             yaw = Double.parseDouble(tokens[8]);
             velocity = Double.parseDouble(tokens[9]);
             
             last_location = location;
             location = new Cell(
             fitToGrid(x),
             fitToGrid(y));
             int i, j;
             for(j=fitToGrid(y-200);j<fitToGrid(y+200);j=j+CELL_SIZE){
             for(i=fitToGrid(x-200);i<fitToGrid(x+200);i=i+CELL_SIZE){
             room.markExplored(new Cell(i, j));
             }
             }
             room.addVertex(location);
             this.rays(16);
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
             Cell newCell = new Cell(
             fitToGrid(ray_x),
             fitToGrid(ray_y), contents);
             //System.out.println("Current: " + location + ", Ray: " + newCell);
             room.addVertex(newCell);
             room.addLine(location,newCell);
             
             }
             //room.print();
             //Attempt to go as far away as possible
             target = room.getFarthestUnexplored(location);
             path = a_star(location, target);
             state = State.SEARCHING;
             
             //If we don't have a path, find a new one
             if(path == null){
             //System.out.println("Destination: " + room.getFarthestUnexplored(location));
             path = a_star(location, target);
             //path = indiscretize(location, room.getFarthestUnexplored(location));
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
             this.where();
             Events ev = null;
             
             // Must get current position
             boolean gotwhere = false;
             while (!gotwhere) {
             ev = concatEvents(ev, this.events());
             gotwhere = findEvents(ev, "getwhere").size() > 0;
             }
             double[] ref = parseWhere(findEvents(ev, "getwhere"));
             location = new Cell(fitToGrid(ref[0]),
             fitToGrid(ref[1]));
             
             if (findEvents(ev, "radius").size() > 0) {
             room.markExplored(location,200);
             }
             
             double traveled = parseStopped(findEvents(ev,
             "STOPPED"));
             if (traveled > -1 && traveled < 1) {
             path = null;
             }
             
             parseRays(findEvents(ev, "rays"));
             double[] tofu = parseRadius(findEvents(ev, "radius"));
             
             if (tofu != null) {
             target = new Cell(fitToGrid(tofu[0]),
             fitToGrid(tofu[1]));
             path = indiscretize(location, target);
             } else if (path == null || path.isEmpty()) {
             target = room.getFarthestUnexplored(location);
             path = indiscretize(location, target);
             }
             
             if (findEvents(ev, "STOPPED").size() > 0) {
             if (dist < 1) {
             path = null;
             
             } else {
             next = path.pop();
             double relx = next.x - x;
             double rely = next.y - y;
             double dist = location.distance(next);
             double angle = Math.atan2(rely, relx);
             angle = Math.toDegrees(angle) - ref[2];
             this.turn((int)angle);
             this.walk((int)dist);
             }
             }
             
             room.printMap();
             this.radius(200);
             this.rays(16);
             
             /*
             if (e.indexOf("getwhere") >= 0) {
             String[] tokens = e.split("[()\\s]+");
             
             x = Double.parseDouble(tokens[3]);
             y = Double.parseDouble(tokens[4]);
             z = Double.parseDouble(tokens[5]);
             roll = Double.parseDouble(tokens[6]);
             pitch = Double.parseDouble(tokens[7]);
             yaw = Double.parseDouble(tokens[8]);
             velocity = Double.parseDouble(tokens[9]);
             
             last_location = location;
             
             location = new Cell(
             fitToGrid(x),
             fitToGrid(y));
             int i, j;
             for(j=fitToGrid(y-200);j<fitToGrid(y+200);j=j+CELL_SIZE){
             for(i=fitToGrid(x-200);i<fitToGrid(x+200);i=i+CELL_SIZE){
             room.markExplored(new Cell(i, j));
             }
             }
             
             room.printUnexplored();
             this.radius(10000);
             }
             
             
             if (e.indexOf("rays") >= 0) {
             String[] tokens = e.split("[()\\s]+");
             for (int i = 0; i<((tokens.length - 4)/5); i++){
             int base = 5+5*i;
             Cell.Contents contents = Cell.Contents.EMPTY;
             if(tokens[base].equals("worldspawn")){
             contents = Cell.Contents.WALL;
             }
             else if(tokens[base].equals("TOFU")){
             contents = Cell.Contents.TOFU;
             }
             //Adjust the ray location from relative to absolute coordinates
             double ray_x = Double.parseDouble(tokens[base+1]) + x;
             double ray_y = Double.parseDouble(tokens[base+2]) + y;
             //Add the point where the ray hit
             room.addVertex(new Cell(
             fitToGrid(ray_x),
             fitToGrid(ray_y), contents));
             //...and everything in between
             room.addLine(location, new Cell(
             fitToGrid(ray_x),
             fitToGrid(ray_y)));
             
             }
             room.printMap();
             
             //If we don't have a path, find a new one
             if(path == null || path.isEmpty()){
             target = room.getFarthestUnexplored(location);
             followingTofu = false;
             //System.out.println("Destination: " + room.getFarthestUnexplored(location));
             path = a_star(location, target);
             //path = indiscretize(location, room.getFarthestUnexplored(location));
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
             String[] tokens = e.split("[()\\s]+");
             double dist = Double.parseDouble(tokens[2]);
             //System.out.println(dist);
             //Guagent is stuck so unstick
             if (dist < 1.0){
             room.removeEdge(location, next);
             this.state = State.UNSTICKING;
             int random = rand.nextInt(270)+90;
             path=null;
             this.turn(random);
             }
             this.where();
             }
             
             //Now we are facing the target so we can walk there
             if (e.indexOf("turnby") >= 0) {
             double x2 = next.x - x;
             double y2 = next.y - y;
             double dist = Math.sqrt(x2*x2 + y2*y2);
             this.walk((int)dist);
             }
             
             
             if(e.indexOf("ask radius") >= 0){
             double tofu_x=0;
             double tofu_y=0;
             double dist = Double.MAX_VALUE;
             double angle = 0;
             boolean tofuFound = false;
             String[] tokens = e.split("[()\\s]+");
             int numItems = Integer.parseInt(tokens[4]);
             //this.radius(10000);
             for (int i = 0; i<numItems; i++){
             if (tokens[5+i*4].equals("tofu")){
             tofuFound = true;
             double temp_tofu_x = Double.parseDouble(tokens[5+i*4+1]);
             double temp_tofu_y = Double.parseDouble(tokens[5+i*4+2]);
             double temp_dist = Math.sqrt(temp_tofu_x*temp_tofu_x + temp_tofu_y*temp_tofu_y);
             if(temp_dist<dist){
             dist = temp_dist;
             tofu_x = temp_tofu_x + x;
             tofu_y = temp_tofu_y + y;
             angle = Math.toDegrees(Math.atan2(tofu_x, tofu_y)) - this.pitch;
             }
             //System.out.println("ANGLE: " + angle + " DIST: " + dist);
             }
             }
             if(!tofuFound){
             this.rays(16);
             }
             else{
             System.out.println("Tofu: "+fitToGrid(tofu_x)+", "+fitToGrid(tofu_y)+" "+followingTofu);
             if (dist > 60)
             {
             if(!followingTofu){
             target = new Cell(fitToGrid(tofu_x), fitToGrid(tofu_y));
             path = a_star(location, target);
             followingTofu = true;
             }
             }
             else{
             this.pickup("tofu");
             followingTofu = false;
             }
             this.rays(16);
             }
             room.markExplored(location,200);
             }
             break;
             
             
             }
             */
        }
        catch (QDiedException er) { // the quagent died -- catch that exception
            System.out.println("bot died!");
        }
        catch (Exception er) { // something else went wrong???
            System.out.println("system failure: "+er);
            System.exit(0);
        }
        
    }
    
    /**
     * Returns a subset of the passed in events, each of which contain the
     *	passed in keyword
     *
     * @param events
     *			Events to search through
     *
     * @param str
     *			String to search events for
     *
     * @return
     *			Subset of events containing the keyword
     */
    public Events findEvents(Events events, String str) {
        Events found = new Events();
        
        for (int i = 0; i < events.size(); i++) {
            String eventString = events.eventAt(i);
            if (eventString.indexOf(str) > 0)
                found.add(eventString);
        }
        return found;
    }
	
	public Events removeEvents(Events events, String str) {
		Events newEvents = new Events();
		
		for (int i = 0; i < events.size(); i++) {
            String eventString = events.eventAt(i);
            if (!(eventString.indexOf(str) > 0))
                newEvents.add(eventString);
        }
        return newEvents;
	}
    
    public Events concatEvents(Events e1, Events e2) {
        Events e3 = new Events();
        if (e1 != null)
            for (int i = 0; i < e1.size(); i++) {
                String eventString = e1.eventAt(i);
                e3.add(eventString);
            }
        if (e2 != null)
            for (int i = 0; i < e2.size(); i++) {
                String eventString = e2.eventAt(i);
                e3.add(eventString);
            }
        return e3;
    }
    
    /**
     * Returns an array of doubles in the following format:
     *	[x, y, yaw]
     *
     * @param events
     *			Events to parse
     *
     * @return
     *			The following doubles: [x, y, yaw]
     */
    public double[] parseWhere(Events events) {
        double[] referenceFrame = new double[3];
        if (events.size() > 0) {
            String eventString = events.eventAt(0);
            String[] tokens = eventString.split("()\\s+");
            referenceFrame[0] = Double.parseDouble(tokens[3]);
            referenceFrame[1] = Double.parseDouble(tokens[4]);
            referenceFrame[2] = Double.parseDouble(tokens[7]);
        }
        return referenceFrame;
    }
    
    public void parseRays(Events events) {
        if (events.size() > 0) {
            String eventString = events.eventAt(0);
            String[] tokens = eventString.split("()\\s+");
            
            for (int i = 5; i < tokens.length; i+=5) {
                double relx = Double.parseDouble(tokens[i+1]);
                double rely = Double.parseDouble(tokens[i+2]);
                Cell.Contents contents = Cell.Contents.EMPTY;
                if (tokens[i].equals("worldspawn"))
                    contents = Cell.Contents.WALL;
                
				double x = relx+location.x;
				double y = rely+location.y;
				
                //Add the point where the ray hit
                room.addVertex(new Cell(fitToGrid(x),
                                        fitToGrid(y), contents));
                
                //...and everything in between
                room.addLine(location, new Cell(fitToGrid(x),
                                                fitToGrid(y)));
            }
        }
    }
    
    public double[] parseRadius(Events events) {
        double tofux = 0;
        double tofuy = 0;
        
        double[] tofu = null;
        
        if (events.size() > 0) {
            String eventString = events.eventAt(0);
            String[] tokens = eventString.split("()\\s+");
            
            for (int i = 5; i < tokens.length; i+=4) {
                if (tokens[i].equals("tofu")) {
                    double relx = Double.parseDouble(tokens[i+1]);
                    double rely = Double.parseDouble(tokens[i+2]);
                    tofu = new double[2];
                    tofu[0] = relx + location.x;
                    tofu[1] = rely + location.y;
                }
            }
        }
        return tofu;
    }
    
    public double parseStopped(Events events) {
        double dist = -1;
        if (events.size() > 0) {
            String eventString = events.eventAt(0);
            String[] tokens = eventString.split("()\\s+");
            
            dist = Double.parseDouble(tokens[2]);
        }
        return dist;
    }
}


