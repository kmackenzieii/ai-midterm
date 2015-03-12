import java.awt.Point;

class Cell extends(Point){
	public enum Contents{
		EMPTY, WALL, TOFU
	}
	private Contents contents;
	
	Cell(int x, int y){
		super(x,y);
		this.contents = Contents.EMPTY;
	}
	
	Cell(int x, int y, Contents contents){
		super(x,y);
		this.contents = contents;
	}
	
	
}