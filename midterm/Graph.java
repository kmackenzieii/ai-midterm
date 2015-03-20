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
        }
		addEdge(v, new Cell(v.x-cell_size, v.y-cell_size));
		addEdge(v, new Cell(v.x-cell_size, v.y));
		addEdge(v, new Cell(v.x-cell_size, v.y+cell_size));
		
		addEdge(v, new Cell(v.x, v.y-cell_size));
		addEdge(v, new Cell(v.x, v.y+cell_size));
		
		addEdge(v, new Cell(v.x+cell_size, v.y-cell_size));
		addEdge(v, new Cell(v.x+cell_size, v.y));
		addEdge(v, new Cell(v.x+cell_size, v.y+cell_size));
    }
    
	/**
	 * Snap the number to a valid point index based on CELL_SIZE
	 * Take into account sign of the point.
	 */
	private int gridToPoint(int num){
		return (num*cell_size+cell_size/2);
	}
	
	/**
	 * Snap the number to a valid grid index based on CELL_SIZE
	 * Take into account sign of the point.
	 */
	private int pointToGrid(int num){
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
		//System.out.println("Start: "+x0+", "+y0);
		//System.out.println("End: "+x1+", "+y1);
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
            if (c.x == (gridToPoint(x)) && c.y == (gridToPoint(y)))
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
}
