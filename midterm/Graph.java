import java.util.*;
import java.lang.*;
class Graph{
	private final int cell_size;
	private ArrayList<Cell> vertices;
	private HashMap<Cell, ArrayList<Cell>> edges;
	private ArrayList<Cell> unexplored;
	private int E;
	private int V;
	
	public Graph(int cell_size){
		this.cell_size = cell_size;
		this.vertices = new ArrayList<Cell>();
		this.unexplored = new ArrayList<Cell>();
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
	
	public void markExplored(Cell v){
		if(unexplored.contains(v)){
			unexplored.remove(v);
		}
	}
	
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
	
	public void removeEdge(Cell w, Cell v){
		if(this.contains(w) && this.contains(v)
				&& edges.get(w).contains(v)){
			E--;
			edges.get(w).remove(v);
			edges.get(v).remove(w);
		}
	}
	
	public ArrayList<Cell> getNeighbors(Cell v){
		return edges.get(v);
	}
	
	public void print(){
		for(Cell vertex : vertices){
			System.out.println(vertex);
			for(Cell neighbor : edges.get(vertex)){
				System.out.println("\t"+neighbor);
			}
		}
	}
}