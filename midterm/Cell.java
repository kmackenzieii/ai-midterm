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
    
    public boolean isWall() {
        return (contents == Contents.WALL);
    }
    
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
