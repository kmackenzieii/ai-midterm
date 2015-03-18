import java.util.*;


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
     * Current location of the quagent
     */
    private Cell location;
    
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
        START, SEARCHING
    }
    
    /**
     * State of the ngent
     */
    private State state = State.START;
    
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
            
            //System.out.println("Hey " + current);
            
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
    
    private Stack undiscretize(Cell start, Cell goal) {
        Stack<Cell> path = a_star(start, goal);
        Stack<Cell> newPath = new Stack<Cell>();
        
        Cell current = path.pop();
        Cell next = current;
        Cell looking = path.pop();
        newPath.push(current);
        
        while (!current.equals(goal)) {
            Stack<Cell> b = bresenham(current, looking);
            while (!containsWall(b) && !looking.equals(goal)) {
                next = looking;
                looking = path.pop();
            }
            if (!looking.equals(goal))
                newPath.push(next);
            current = next;
        }
        newPath.push(goal);
        while (!newPath.isEmpty())
            path.push(newPath.pop());
        return path;
    }
    
    private Stack bresenham(Cell c1, Cell c2) {
        
        // Stack to hold the cells
        Stack<Cell> line = new Stack<Cell>();
        
        // Get endpoint coordinates
        int y1 = c1.y / CELL_SIZE;
        int y2 = c2.y / CELL_SIZE;
        int x1 = c1.x / CELL_SIZE;
        int x2 = c2.x / CELL_SIZE;
        
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
        Cell c = room.getCellAt(x1, y1);
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
                        c = room.getCellAt(x, y-ystep);
                        if (c != null)
                            line.push(c);
                    }
                    // Left
                    else if (error + errorprev > ddx) {
                        c = room.getCellAt(x-xstep, y);
                        if (c != null)
                            line.push(c);
                    }
                    // Both (crosses corner exactly)
                    else {
                        c = room.getCellAt(x, y-ystep);
                        if (c != null)
                            line.push(c);
                        c = room.getCellAt(x-xstep, y);
                        if (c != null)
                            line.push(c);
                    }
                }
                c = room.getCellAt(x, y);
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
                        c = room.getCellAt(x-xstep, y);
                        if (c != null)
                            line.push(c);
                    }
                    else if (error + errorprev > ddy) {
                        c = room.getCellAt(x, y-ystep);
                        if (c != null)
                            line.push(c);
                    }
                    else{
                        c = room.getCellAt(x, y-ystep);
                        if (c != null)
                            line.push(c);
                        c = room.getCellAt(x-xstep, y);
                        if (c != null)
                            line.push(c);
                    }
                }
                c = room.getCellAt(x, y);
                if (c != null)
                    line.push(c);
                errorprev = error;
            }
        }
        // assert ((y == y2) && (x == x2));
        // the last point (y2,x2) has to be the same as the last point of the algorithm
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
                            //Attempt to go as far away as possible
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
                                //Adjust the ray location from relative to absolute coordinates
                                double ray_x = Double.parseDouble(tokens[base+1]) + x;
                                double ray_y = Double.parseDouble(tokens[base+2]) + y;
                                //Add the point where the ray hit
                                room.addVertex(new Cell((((int)ray_x/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), (((int)ray_y/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), contents));
                                //...and everything in between
                                room.addLine(location, new Cell((((int)ray_x/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2), (((int)ray_y/CELL_SIZE)*CELL_SIZE+CELL_SIZE/2)));
                                
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


