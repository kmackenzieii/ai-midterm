import java.util.*;
class Graph{
	private final int cell_size;
	private ArrayList<Cell> vertices;
	private HashMap<Cell, ArrayList<Cell>> edges;
	private int E;
	private int V;
	
	public Graph(int cell_size){
		this.cell_size = cell_size;
		this.vertices = new ArrayList<Cell>();
		this.edges = new HashMap<Cell, ArrayList<Cell>>();
		this.E = 0;
		this.V = 0;
	}
	
	public int V(){
		return V;
	}
	
	public int E(){
		return E;
	}
	
	public boolean contains(Cell v){
		return vertices.contains(v);
	}
	
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
	
	public void addEdge(Cell w, Cell v){
		if(this.contains(w) && this.contains(v)){
			E++;
			edges.get(w).add(v);
			edges.get(v).add(w);
		}
	}
	
	public void removeEdge(Cell w, Cell v){
		if(this.contains(w) && this.contains(v)
				&& edges.get(w).contains(v){
			E--;
			edges.get(w).remove(v);
			edges.get(v).remove(w);
		}
	}
	
	public ArrayList<Cell> getNeighbors(Cell v){
		return edges.get(v);
	}
}