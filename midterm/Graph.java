import java.util.*;
import java.lang.*;

/**
 * Defines a graph of vertices and edges. Each vertex is a cell, and each edge
 *	is a connection between two cels
 */
class Graph{
    /**
     * Width of each cell
     */
    private static int cell_size;
    
    /**
     * Array list of vertices
     */
    private ArrayList<Cell> vertices;
    
    /**
     * Hash map of cells, each bound to an array list of neighboring cells
     */
    private HashMap<Cell, ArrayList<Cell>> edges;
    
    /**
     * Array list of unexplored vertices
     */
    private ArrayList<Cell> unexplored;
    
    /**
     * Number of edges
     */
    private int E;
    
    /**
     * Number of vertices
     */
    private int V;
    
    /**
     * Constructor initializes variables
     */
    public Graph(int cell_size){
        this.cell_size = cell_size;
        this.vertices = new ArrayList<Cell>();
        this.unexplored = new ArrayList<Cell>();
        this.edges = new HashMap<Cell, ArrayList<Cell>>();
        this.E = 0;
        this.V = 0;
    }
    
    /**
     * Accessor method for number of vertices
     */
    public int V(){
        return V;
    }
    
    /**
     * Accessor method for number of edges
     */
    public int E(){
        return E;
    }
    
    public int numUnexplored() {
        return unexplored.size();
    }
    
    /**
     * Determines whether the graph contains the passed in cell
     */
    public boolean contains(Cell v){
        return vertices.contains(v);
    }
    
    public boolean neighborIsWall(Cell v){
        for(Cell neighbor : getNeighbors(v)){
            if(neighbor.isWall()){
                return true;
            }
        }
        return false;
    }
    
    /**
     * Add the passed in cell to the graph.
     */
    public void addVertex(Cell v){
        if(!this.contains(v)){
            vertices.add(v);
            if(!v.isWall()){
                unexplored.add(v);
            }
            edges.put(v, new ArrayList<Cell>());
            V++;
        }
        addEdge(v, new Cell(v.x-cell_size, v.y));
        //addEdge(v, new Cell(v.x-cell_size, v.y-cell_size));
        //addEdge(v, new Cell(v.x-cell_size, v.y+cell_size));
        
        addEdge(v, new Cell(v.x, v.y-cell_size));
        
        addEdge(v, new Cell(v.x, v.y+cell_size));
        
        addEdge(v, new Cell(v.x+cell_size, v.y));
        //addEdge(v, new Cell(v.x+cell_size, v.y-cell_size));
        //addEdge(v, new Cell(v.x+cell_size, v.y+cell_size));
        
        if(v.isWall()){
            vertices.get(vertices.indexOf(v)).setContents(Cell.Contents.WALL);
            disconnectWalls();
        }
    }
    
    private void disconnectWalls(){
        for(Cell v : vertices){
            if(v.isWall()){
                removeEdge(v, new Cell(v.x-cell_size, v.y));
                
                removeEdge(v, new Cell(v.x, v.y-cell_size));
                
                removeEdge(v, new Cell(v.x, v.y+cell_size));
                
                removeEdge(v, new Cell(v.x+cell_size, v.y));
            }
        }
    }
    
    /**
     * Snap the number to a valid point index based on CELL_SIZE
     * Take into account sign of the point.
     */
    public static int gridToPoint(int num){
        return (num*cell_size+cell_size/2);
    }
    
    /**
     * Snap the number to a valid grid index based on CELL_SIZE
     * Take into account sign of the point.
     */
    public static int pointToGrid(int num){
        return ((num-cell_size/2)/cell_size);
    }
    /**
     * Adds a line according to Bresenham's algorithm
     */
    public void addLine(Cell c1, Cell c2){
        int y0 = pointToGrid(c1.y);
        int y1 = pointToGrid(c2.y);
        int x0 = pointToGrid(c1.x);
        int x1 = pointToGrid(c2.x);
        // If slope is outside the range [-1,1], swap x and y
        boolean xy_swap = false;
        if (Math.abs(y1 - y0) > Math.abs(x1 - x0)) {
            xy_swap = true;
            int temp = x0;
            x0 = y0;
            y0 = temp;
            temp = x1;
            x1 = y1;
            y1 = temp;
        }
        
        // If line goes from right to left, swap the endpoints
        if (x0 - x1 > 0) {
            int temp = x0;
            x0 = x1;
            x1 = temp;
            temp = y0;
            y0 = y1;
            y1 = temp;
        }
        
        
        int deltax = Math.abs(x1 - x0);
        int deltay = Math.abs(y1 - y0);
        int d = 2*deltay - deltax;
        
        if(xy_swap){
            addVertex(new Cell(gridToPoint(y0), gridToPoint(x0)));
            //System.out.println(y0+", "+x0);
        }
        else {
            addVertex(new Cell(gridToPoint(x0), gridToPoint(y0)));
            //System.out.println(x0+", "+y0);
        }
        
        int y = y0;
        for (int x = x0+1; x<x1+1; x++) {
            if(d>0){
                y = y+Integer.signum(y1-y0);
                
                if(d==1){
                    if(xy_swap){
                        addVertex(new Cell(gridToPoint(y-1), gridToPoint(x)));
                        //System.out.println(y+", "+x);
                    }
                    else {
                        addVertex(new Cell(gridToPoint(x), gridToPoint(y-1)));
                        //System.out.println(x+", "+y);
                    }
                }
                else{
                    if(xy_swap){
                        addVertex(new Cell(gridToPoint(y), gridToPoint(x-1)));
                        //System.out.println(y+", "+x);
                    }
                    else {
                        addVertex(new Cell(gridToPoint(x-1), gridToPoint(y)));
                        //System.out.println(x+", "+y);
                    }
                }
                
                if(xy_swap){
                    addVertex(new Cell(gridToPoint(y), gridToPoint(x)));
                    //System.out.println(y+", "+x);
                }
                else {
                    addVertex(new Cell(gridToPoint(x), gridToPoint(y)));
                    //System.out.println(x+", "+y);
                }
                d = d + (2*deltay-2*deltax);
            }
            else{
                if(xy_swap){
                    addVertex(new Cell(gridToPoint(y), gridToPoint(x)));
                    //System.out.println(y+", "+x);
                }
                else {
                    addVertex(new Cell(gridToPoint(x), gridToPoint(y)));
                    //System.out.println(x+", "+y);
                }
                d = d + (2*deltay);
            }
        }
        
    }
    
    /**
     * Marks a cell as being explored by removing it from the
     * unexplored list
     */
    public void markExplored(Cell v){
        unexplored.remove(v);
    }
    
    public void markExplored(Cell v, int radius) {
        ArrayList<Cell> toMark = new ArrayList<Cell>();
        for (Cell u : unexplored) {
            if (v.distance(u) < radius)
                toMark.add(u);
        }
        for (Cell m : toMark) {
            markExplored(m);
        }
    }
    
    
    /**
     * Return closest, most unexplored cell
     */
    public Cell getIsolatedUnexplored(Cell v, int radius){
        Cell ret = null;
        int nearby_unexplored = 0;
        double distance = Double.MAX_VALUE;
        for (Cell u : unexplored) {
            int nearby_compare = 0;
            for (Cell u2 : unexplored){
                if (u.distance(u2) < radius)
                    nearby_compare++;
            }
            if(nearby_compare > nearby_unexplored){// && v.distance(u) < distance){
                nearby_unexplored = nearby_compare;
                distance = v.distance(u);
                ret = u;
            }
        }
        return ret;
    }
    
    /**
     * Return closest cell with the most unseen in its neighborhood
     */
    public Cell getIsolatedUnseen(Cell v, int radius){
        Cell ret = null;
        int nearby_unseen = 0;
        for (Cell c : vertices) {
            int nearby_compare = 0;
            if (!c.isWall()) {
                for (Cell n : vertices){
                    if (c.distance(n) < radius && !n.isWall() &&
                        getNeighbors(n).size() < 4)
                        nearby_compare++;
                }
                if(nearby_compare > nearby_unseen){
                    nearby_unseen = nearby_compare;
                    ret = c;
                }
            }
        }
        return ret;
    }
    
    /**
     * Returns the unexplored cell farthest from cell v
     */
    public Cell getFarthestUnexplored(Cell v){
        double distance = 0;
        Cell ret = null;
        for(Cell w : unexplored){
            if(v.distance(w) > distance){
                distance = v.distance(w);
                ret = w;
            }
        }
        return ret;
    }
    
    /**
     * Returns the unexplored cell nearest to cell v
     */
    public Cell getNearestUnexplored(Cell v){
        double distance = Double.MAX_VALUE;
        Cell ret = null;
        for(Cell w : unexplored){
            // Anything within 200 clicks will have been explored
            //  by the time we reach it
            if(v.distance(w) <= distance && v.distance(w) >= 200){
                distance = v.distance(w);
                ret = w;
            }
        }
        // If nothing was found outside 200 clicks, look outside
        if (ret == null)
            for(Cell w : unexplored){
                if(v.distance(w) <= distance){
                    distance = v.distance(w);
                    ret = w;
                }
            }
        return ret;
    }
    
    /**
     * Returns true if the cell has not been explored
     */
    public boolean isUnexplored(Cell v){
        boolean isUnexplored = false;
        for(Cell w : unexplored){
            if(v.equals(w)){
                isUnexplored=true;
            }
        }
        return isUnexplored;
    }
    
    
    /**
     * Create an edge between the passed in cells (vertices)
     */
    public void addEdge(Cell w, Cell v){
        if(this.contains(w) && this.contains(v)){
            E++;
            if(!edges.get(w).contains(v)){
                edges.get(w).add(v);
            }
            if(!edges.get(v).contains(w)){
                edges.get(v).add(w);
            }
        }
    }
    
    /**
     * Remove the edge between the passed in cells (vertices)
     */
    public void removeEdge(Cell w, Cell v){
        if(this.contains(w) && this.contains(v)) {
            E--;
            edges.get(w).remove(v);
            edges.get(v).remove(w);
        }
    }
    
    /**
     * Returns the neighbors of the passed in cell, as defined by the its edges
     */
    public ArrayList<Cell> getNeighbors(Cell v){
        return edges.get(v);
    }
    
    public Cell getCellAt(int x, int y) {
        for (Cell c : vertices)
            if (c.x == x && c.y == y)
                return c;
        return null;
    }
    
    public Cell getCellAtIndex(int x, int y) {
        for (Cell c : vertices)
            if (c.x == (gridToPoint(x)) && c.y == (gridToPoint(y)))
                return c;
        return null;
    }
    
    public void printUnexplored(){
        System.out.println("# Unexplored: "+unexplored.size());
    }
    
    /**
     * Prints every vertex in the map as well as the neighbors for each.
     */
    public void print(){
        for(Cell vertex : vertices){
            System.out.println(vertex);
            for(Cell neighbor : edges.get(vertex)){
                System.out.println("\t"+neighbor);
            }
        }
    }
    
    /**
     * Print the map of Ms and Os
     */
    public void printMap()
    {
        // Figure out the current bounds of the map
        Cell leftmost = vertices.get(0);
        Cell rightmost = vertices.get(0);
        Cell topmost = vertices.get(0);
        Cell bottommost = vertices.get(0);
        
        for (Cell v : vertices) {
            if (v.x < leftmost.x)
                leftmost = v;
            if (v.x > rightmost.x)
                rightmost = v;
            if (v.y > topmost.y)
                topmost = v;
            if (v.y < bottommost.y)
                bottommost = v;
        }
        
        int leftBound = leftmost.x;
        int rightBound = rightmost.x;
        int upperBound = topmost.y;
        int lowerBound = bottommost.y;
        
        // Do the printing (space for unseen, . for unexplored, + for explored,
        //  # for walls
        System.out.println("MAP: ");
        Cell current;
        for (int row = lowerBound; row <= upperBound; row += cell_size) {
            for (int col = leftBound; col <= rightBound; col += cell_size) {
                current = getCellAt(col, row);
                if (current != null) {
                    if (current.isWall())
                        System.out.print("#");
                    else if (unexplored.contains(current))
                        System.out.print(".");
                    else
                        System.out.print("+");
                } else {
                    System.out.print(" ");
                }
            }
            System.out.print("\n");
        }
    }
    
    /**
     * Print the map of Ms and Os
     */
    public void printMap(Stack<Cell> path)
    {
        // Figure out the current bounds of the map
        Cell leftmost = vertices.get(0);
        Cell rightmost = vertices.get(0);
        Cell topmost = vertices.get(0);
        Cell bottommost = vertices.get(0);
        
        for (Cell v : vertices) {
            if (v.x < leftmost.x)
                leftmost = v;
            if (v.x > rightmost.x)
                rightmost = v;
            if (v.y > topmost.y)
                topmost = v;
            if (v.y < bottommost.y)
                bottommost = v;
        }
        
        int leftBound = leftmost.x;
        int rightBound = rightmost.x;
        int upperBound = topmost.y;
        int lowerBound = bottommost.y;
        
        // Do the printing (space for unseen, . for unexplored, + for explored,
        //  # for walls
        System.out.println("MAP: ");
        Cell current;
        for (int row = upperBound; row >= lowerBound; row -= cell_size) {
            for (int col = leftBound; col <= rightBound; col += cell_size) {
                current = getCellAt(col, row);
                if (current != null) {
                    if(path.contains(current))
                        System.out.print("O");
                    else if (current.isWall())
                        System.out.print("#");
                    else if (unexplored.contains(current))
                        System.out.print(".");
                    else
                        System.out.print("+");
                } else {
                    System.out.print(" ");
                }
            }
            System.out.print("\n");
        }
    }
}
