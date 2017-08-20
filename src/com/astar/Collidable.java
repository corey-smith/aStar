package com.astar;

public interface Collidable {
	
	/**
	 * get right bound
	 * @return offsetX
	 */
	public float getRightBound();
	
	/**
	 * get left bound
	 * @return offsetX
	 */
	public float getLeftBound();
	
	/**
	 * get top bound
	 * @return offsetY + height
	 */
	public float getTopBound();
	
	/**
	 * get bottom bound
	 * @return offsetY
	 */
	public float getBottomBound();
	
	/**
	 * get width
	 * @return width
	 */
	public float getWidth();
	
	/**
	 * get height
	 * @return height
	 */
	public float getHeight();
	
	/**
	 * get x position within map
	 * @return offsetX
	 */
	public float getXOffset();
	
	/**
	 * get y position within map
	 * @return
	 */
	public float getYOffset();
	
}
