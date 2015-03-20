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
    private final int cell_size;
    
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
    
    /**
     * Determines whether the graph contains the passed in cell
     */
    public boolean contains(Cell v){
        return vertices.contains(v);
    }
    
    /**
     * Add the passed in cell to the graph.
     */
    public void addVertex(Cell v){
        if(!this.contains(v)){
            vertices.add(v);
            if(v.content() != Cell.Contents.WALL){
                unexplored.add(v);
            }
            edges.put(v, new ArrayList<Cell>());
            V++;
            
            addEdge(v, new Cell(v.x-cell_size, v.y-cell_size));
            addEdge(v, new Cell(v.x-cell_size, v.y));
            addEdge(v, new Cell(v.x-cell_size, v.y+cell_size));
            
            addEdge(v, new Cell(v.x, v.y-cell_size));
            addEdge(v, new Cell(v.x, v.y+cell_size));
            
            addEdge(v, new Cell(v.x+cell_size, v.y-cell_size));
            addEdge(v, new Cell(v.x+cell_size, v.y));
            addEdge(v, new Cell(v.x+cell_size, v.y+cell_size));
        }
    }
    
    /**
     * Adds a line according to Bresenham's algorithm
     */
    public void addLine(Cell w, Cell v){
        int y0 = w.y / cell_size;
        int y1 = v.y / cell_size;
        int x0 = w.x / cell_size;
        int x1 = v.x / cell_size;
        
        
        // If slope is outside the range [-1,1], swap x and y
        boolean xy_swap = false;
        if (Math.abs(y1 - y0) > Math.abs(x1 - x0)) {
            xy_swap = true;
            int temp = x1;
            x0 = y0;
            y0 = temp;
            temp = x1;
            x1 = y1;
            y1 = temp;
        }
        
        // If line goes from right to left, swap the endpoints
        if (x1 - x0 < 0) {
            int temp = x0;
            x0 = x1;
            x1 = temp;
            temp = y0;
            y0 = y1;
            y1 = temp;
        }
        
        
        
        
        int deltax = x1 - x0;
        int deltay = y1 - y0;
        int d = 2*deltay - deltax;
        int y = y0;
        for (int x = x0+1; x<x1; x++) {
            if(d>0){
                y = y+1;
                if(xy_swap){
                    addVertex(new Cell(y*cell_size+cell_size/2, x*cell_size+cell_size/2));
                }
                else {
                    addVertex(new Cell(x*cell_size+cell_size/2, y*cell_size+cell_size/2));
                }
                d = d + (2*deltay-2*deltax);
            }
            else{
                if(xy_swap){
                    addVertex(new Cell(y*cell_size+cell_size/2, x*cell_size+cell_size/2));
                }
                else {
                    addVertex(new Cell(x*cell_size+cell_size/2, y*cell_size+cell_size/2));
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
        if(unexplored.contains(v)){
            unexplored.remove(v);
        }
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
        if(this.contains(w) && this.contains(v)
           && edges.get(w).contains(v)) {
            E--;
            edges.get(w).remove(v);
            edges.get(v).remove(w);
        }
    }
    
    /**
     * Returns the neighbors of the passed in cell, as defined by the it's edges
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
            if (c.x == (x * cell_size) && c.y == (y * cell_size))
                return c;
        return null;
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
    
    //print the map of Ms and Os
    public void printMap()
    {
        int row = 0;
        int col = 0;
        System.out.println("\n\n\n\n\n");
        boolean yes = false;
        //for(Cell vertex : vertices)
        //{
        //if vertex = the first vertex in the list, i.e. my Loc, then store it
        //    if(vertex==vertices.get(0))
        //{
        //    myX=vertex.x;
        //    myY=vertex.y;
        //}
        //System.out.println(vertex.x);
        //}
        
        /*search through u list (row) 32 at a time
         //then k list (col) 32 at a time
         //if both row and col match the x and y in a cell, then print M instead of O */
        while(row<=512) // u or y`
        {
            while(col<=512) // k or x
            {
                /*Loops through the values 0-512 by incrementing 32 units at a time.
                 In this way, it compares all values of x per 1 value of y, and checks each
                 vertex in 'vertices' for each comparison*/
                for(Cell vertex : vertices)
                {
                    if(vertex.y == row && vertex.x==col)
                        yes=true;
                }
                
                if(yes)
                {
                    System.out.print("M "); // mapped
                    yes=false; //reset yes so it can be mused again
                }
                else
                    System.out.print("O "); // Unmapped
                col+=32;
            }
            row+=32;
            System.out.println("\n"); // mapped
            col=0;
        }
    }
}
