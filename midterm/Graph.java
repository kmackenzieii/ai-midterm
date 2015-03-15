import java.util.*;

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
		int deltax = x1 - x0;
		int deltay = y1 - y0;
		double error = 0;
		double deltaerr = abs (deltay / deltax);
		int y = y0;
		for int x = x0; x<x1; x++ {
			addVertex(new Cell(x*cell_size+cell_size/2, y*cell_size+cell_size/2));
			error = error + deltaerr;
			while (error >= 0.5) {
				addVertex(new Cell(x*cell_size+cell_size/2, y*cell_size+cell_size/2));
				y = y + sign(y1 - y0);
				error = error - 1.0;
			}
		}
	}
	
	/**
	 * Create an edge between the passed in cells (vertices)
	 */
	public void addEdge(Cell w, Cell v){
		if(this.contains(w) && this.contains(v)){
			E++;
			edges.get(w).add(v);
			edges.get(v).add(w);
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
}