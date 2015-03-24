import java.awt.Point;

/**
 * Defines a cell
 */
class Cell extends Point {
    
    /**
     * Contents of the cell
     */
    private Contents contents;
    
    /**
     * Constructor initializes variables
     */
    Cell(int x, int y){
        super(x,y);
        this.contents = Contents.EMPTY;
    }
    
    /**
     * Constructor initializes variables
     */
    Cell(int x, int y, Contents contents){
        super(x,y);
        this.contents = contents;
    }
    
    /**
     *  Returns true if the cell contains a wall
     */
    public boolean isWall() {
        return (contents == Contents.WALL);
    }
    
    /**
     * Returns true if the passed in cell has the same coordinates as this one
     */
    public boolean equals(Cell c) {
        return x == c.x && y == c.y;
    }
    
    /**
     * Return the contents of the cell
     */
    public Contents content(){
        return contents;
    }
    
    /**
     * Returns the contents of this cell
     */
	public void setContents(Contents c){
        this.contents = c;
    }
    
    /**
     * Returns the manhatten disticane between the passed in cell and this one
     */
    public int heuristic (Cell c) {
        return Math.abs(c.x - x) + Math.abs(c.y - y);
    }
	
    /**
     * Enumeration to describe the contents of a cell
     */
    public enum Contents {
        
        /**
         * The cell is empty
         */
        EMPTY,
        
        /**
         * The cell contains a wall
         */
        WALL,
        
        /**
         * The cell contains tofu
         */
        TOFU
    }
    
}
