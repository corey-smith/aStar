package com.astar;

public class PathNode {

	protected int x;
	protected int y;
	protected int width;
	protected int height;
	protected boolean collidable;
	protected PathNode prevNode;
	protected Float fCost = null;
	protected Float gCost = null;
	protected Float hCost = null;
	protected String id;
	
	public PathNode(int x, int y) {
		this.x = x;
		this.y = y;
		this.setID(x,y);
	}
	
	public PathNode(PathNode pathNode) {
		this(pathNode.getX(), pathNode.getY());
		this.collidable = pathNode.isCollidable();
	}
	
	public void setX(int x) {
		this.x = x;
	}
	
	public void setY(int y) {
		this.y = y;
	}
	
	public int getX() {
		return this.x;
	}
	
	public int getY() {
		return this.y;
	}
	
	public void setWidth(int width) {
		this.width = width;
	}
	
	public void setHeight(int height) {
		this.height = height;
	}
	
	public int getWidth() {
		return this.width;
	}
	
	public int getHeight() {
		return this.height;
	}
	
	public void setCollidable(boolean collidable) {
		this.collidable = collidable;
	}
	
	public boolean isCollidable() {
		return this.collidable;
	}
	
	public void setPrevNode(PathNode prevNode) {
		this.prevNode = prevNode;
	}
	
	public PathNode getPrevNode() {
		return this.prevNode;
	}
	
	public void setFCost(float fCost) {
		this.fCost = fCost;
	}
	
	public void setGCost(float gCost) {
		this.gCost = gCost;
	}
	
	public void setHCost(float hCost) {
		this.hCost = hCost;
	}
	
	public Float getFCost() {
		return this.fCost;
	}
	
	public Float getGCost() {
		return this.gCost;
	}
	
	public Float getHCost() {
		return this.hCost;
	}
	
	public String getID() {
		return this.id;
	}
	
	public void setID(int x, int y) {
		this.id = "X:" + x + "Y:" + y;
	}
	
	public String toString() {
		return this.id;
	}
	
}
