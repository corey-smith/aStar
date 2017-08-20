package com.astar;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;

/**
 * 
 * A* path finder
 */
public abstract class AStar {

	//2D array or x/y position of tiles - should be height of map
	static PathNode nodes[][];
	static ArrayList<Collidable> collidablePathNodes;
	static int tileWidth;
	static int tileHeight;
	
	/**
	 * Should only need to declare this once in the main class to initialize these params
	 * @param tiles
	 * @param tileWidth
	 * @param tileHeight
	 */
	public static void initialize(ArrayList<Collidable> tmpCollidablePathNodes, int[][] tmpNodes, int tmpTileWidth, int tmpTileHeight) {
		collidablePathNodes = tmpCollidablePathNodes;
		tileWidth = tmpTileWidth;
		tileHeight = tmpTileHeight;
		nodes = new PathNode[tmpNodes.length][tmpNodes[0].length];
		initializeNodes();
		setCollidableNodes(tmpCollidablePathNodes);
	}
	
	/**
	 * build out node array and clear any previous values
	 */
	private static void initializeNodes() {
		//build out nodes array
		for(int x = 0; x < nodes.length; x++) {
			for(int y = 0; y < nodes[0].length; y++) {
				nodes[x][y] = new PathNode(x, y);
			}
		}
	}
	
	/**
	 * loop through all of the collidable objects and find their corresponding node and set it to collidable
	 */
	private static void setCollidableNodes(ArrayList<Collidable> collidablePathNodes) {
		for(Collidable collNode : collidablePathNodes) {
			float objLeft = collNode.getLeftBound();
			float objBottom = collNode.getBottomBound();
			float objRight = objLeft + collNode.getWidth();
			float objTop = objBottom + collNode.getHeight();
			int firstX = (int) Math.floor(objLeft / tileWidth);
			int lastX = (int) Math.ceil(objRight / tileWidth);
			int firstY = (int) Math.floor(objBottom / tileHeight);
			int lastY = (int) Math.ceil(objTop / tileHeight);
			for(int i = firstX; i < lastX; i++) {
				for(int j = firstY; j < lastY; j++) {
					nodes[i][j].setCollidable(true);
				}
			}
		}
	}
	
	public static ArrayList<PathNode> findPath(PathNode startingNode, PathNode endingNode) {
		return findPath(startingNode.getX(),  startingNode.getY(), endingNode.getX(), endingNode.getY());
	}

	public static ArrayList<PathNode> findPath(float startingX, float startingY, float endingX, float endingY) {
		return findPath(getXNodeIndexFromOffset(startingX), getYNodeIndexFromOffset(startingY), getXNodeIndexFromOffset(endingX), getYNodeIndexFromOffset(endingY));
	}
	
	/**
	 * Actual logic to find the path
	 * @param startingX - map in relation to map
	 * @param startingY - map in relation to map
	 * @param endingX   - map in relation to map
	 * @param endingY   - map in relation to map
	 * @return			- ArrayList<PathNode> of the path nodes in order
	 */
	public static ArrayList<PathNode> findPath(int startingX, int startingY, int endingX, int endingY) {
		LinkedList<PathNode> openList = new LinkedList<PathNode>();
		LinkedList<PathNode> closedList = new LinkedList<PathNode>();
		PathNode[][] pathNodes = getPathNodes();
		PathNode startingNode =  pathNodes[startingX][startingY];
		PathNode endingNode = nodes[endingX][endingY];
		openList.add(startingNode);
		startingNode.setGCost(0f);
		boolean done = false;
		PathNode curNode = startingNode;
		while(!done) {
			curNode = lowestFCostOpenNode(openList);
			closedList.add(curNode);
			openList.remove(curNode);
			
			//check to see if the current node is the ending node
			if ((curNode.getX() == endingNode.getX()) && (curNode.getY() == endingNode.getY())) {
                return getPath(startingNode, curNode);
            }
			
			//find all adjacent nodes and evaluate
			LinkedList<PathNode> adjacentNodes = getAdjacentNodes(curNode, pathNodes);
			for(PathNode curAdj : adjacentNodes) {
				if (!openList.contains(curAdj) && !closedList.contains(curAdj)) {
					curAdj.setPrevNode(curNode);
					//set h costs of this node (estimated costs to goal)
					curAdj.setHCost(calculateHCost(curAdj, endingNode));
					//set g costs of this node (costs from start to this node), just add one to the cost of the current node
					curAdj.setGCost(curNode.getGCost() + 1);
					curAdj.setFCost(curAdj.getHCost() + curAdj.getGCost());
                    openList.add(curAdj);
                } else {
                	//costs from current node are cheaper than previous costs
                    if (curAdj.getGCost() > curNode.getGCost() + 1) {
                    	closedList.remove(curAdj);
                    	//set current node as previous for this node
                    	curAdj.setPrevNode(curNode);
                    	//set g costs of this node (costs from start to this node)
    					curAdj.setHCost(calculateHCost(curAdj, endingNode));
                    	curAdj.setGCost(curNode.getGCost() + 1); 
    					curAdj.setFCost(curAdj.getHCost() + curAdj.getGCost());
                    	if(!openList.contains(curAdj)) openList.add(curAdj);
                    }
                }
			}
			//no path exists, return empty list
			if(openList.isEmpty()) {
				return new ArrayList<PathNode>();
			}
		}
		//this isn't actually reachable
		return null;
	}
	
	/**
	 * Get a local copy of all nodes to use in determining path
	 * @return - basically a copy of nodes[][]
	 */
	private static PathNode[][] getPathNodes() {
		PathNode[][] returnArray = new PathNode[nodes.length][nodes[0].length];
		for(int x = 0; x < nodes.length; x++) {
			for(int y = 0; y < nodes[0].length; y++) {
				returnArray[x][y] = new PathNode(nodes[x][y]);
			}
		}
		return returnArray;
	}
	
	private static PathNode lowestFCostOpenNode(LinkedList<PathNode> openList) {
		PathNode returnNode = null;
		for(PathNode curNode : openList) {
			if(returnNode == null || (curNode.getFCost() != null && returnNode.getFCost() != null && curNode.getFCost() < returnNode.getFCost())) {
				returnNode = curNode;
			}
		}
		return returnNode;
	}
	
	/**
	 * Get all of the possible adjacent (not collidable) nodes given a node
	 * @return - list of adjacent nodes
	 */
	public static LinkedList<PathNode> getAdjacentNodes(PathNode currentNode) {
		return getAdjacentNodes(currentNode, getPathNodes());
	}
	
	/**
	 * Get all of the possible adjacent (not collidable) nodes given a node and an original or copy of the nodes in the map
	 * @return - list of adjacent nodes
	 */
	public static LinkedList<PathNode> getAdjacentNodes(PathNode currentNode, PathNode[][] pathNodes) {
		LinkedList<PathNode> returnList = new LinkedList<PathNode>();
		//define nodes, validate they're there
		int curX = currentNode.x;
		int curY = currentNode.y;
		PathNode northNode = (PathNode) (nodeExists(curX, curY + 1) ? pathNodes[currentNode.x][currentNode.y+1] : null);
		PathNode southNode = (PathNode) (nodeExists(curX, curY - 1) ? pathNodes[currentNode.x][currentNode.y-1] : null);
		PathNode eastNode = (PathNode) (nodeExists(curX + 1, curY) ? pathNodes[currentNode.x+1][currentNode.y] : null);
		PathNode westNode = (PathNode) (nodeExists(curX - 1, curY) ? pathNodes[currentNode.x-1][currentNode.y] : null);
		//add non-collidable to list
		//north
		if(northNode != null && !northNode.isCollidable()) {
			returnList.add(northNode);
		}
		//south
		if(southNode != null && !southNode.isCollidable()) {
			returnList.add(southNode);
		}
		//east
		if(eastNode != null && !eastNode.isCollidable()) {
			returnList.add(eastNode);
		}
		//west
		if(westNode != null && !westNode.isCollidable()) {
			returnList.add(westNode);
		}
		return returnList;
	}
	
	private static boolean nodeExists(int x, int y) {
		if(x >= 0 && y >= 0 && x < nodes.length && y < nodes[0].length) {
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Calcualate H cost from a given node to the ending node
	 * H cost is basically the cost of from here to the goal
	 * @param curNode
	 * @param endingNode
	 * @return
	 */
	private static float calculateHCost(PathNode curNode, PathNode endingNode) {
	    //less costly and simpler method
	    int dx = Math.abs(endingNode.getX() - curNode.getX());
	    int dy = Math.abs(endingNode.getY() - curNode.getY());
	    return (float) dx+dy;
	     
	}
	
	/**
	 * Build out arraylist with the resulting path
	 * @return - ordered list of nodes in the path
	 */
	public static ArrayList<PathNode> getPath(PathNode startingNode, PathNode endingNode) {
		ArrayList<PathNode> returnPath = new ArrayList<PathNode>();
		PathNode curNode = endingNode;
		//work back through previous nodes and add them to the path
		while(curNode.getPrevNode() != null) {
			returnPath.add(curNode);
			curNode = curNode.getPrevNode();
		}
		//reverse order of path
		Collections.reverse(returnPath);
		return returnPath;
	}
	
	/**
	 * Get a list of all of the nodes a specific unit can move to in one turn
	 * @param curUnit - the unit who is going to move somewhere
	 * @return - arraylist of nodes that the unit can move to
	 */
	public static ArrayList<PathNode> getMovableNodes(int xNode, int yNode, int distance) {
		ArrayList<PathNode> movableNodes = new ArrayList<PathNode>();
		ArrayList<String> movableIDs = new ArrayList<String>();
		//loop through x direction, y direction
		for(int x = xNode - distance; x <= xNode + distance; x++) {
			for(int y = yNode - distance; y <= yNode + distance; y++) {
				if(x >= 0 && x < nodes.length && y >= 0 && y < nodes[0].length) {
					int absDistance = Math.abs(xNode - x) + Math.abs(yNode - y);
					if(absDistance <= distance && !movableIDs.contains(getNodeIDFromLoc(x,y))) {
						ArrayList<PathNode> curPath = findPath(xNode, yNode, x, y);
						if(curPath.size() > 0 && curPath.size() <= distance) {
							//add all of the nodes in the path
							for(PathNode curPathNode : curPath) {
								if(!movableIDs.contains(curPathNode.getID())) {
									movableNodes.add(curPathNode);
									movableIDs.add(curPathNode.getID());
								}
							}
						}
					}
				}
			}
		}
		return movableNodes;
	}
	
	/**
	 * This is a way to see what a node's ID would be given an X and Y location
	 * @return String of ID
	 */
	public static String getNodeIDFromLoc(int x, int y) {
		PathNode tempNode = new PathNode(x,y);
		return tempNode.getID();
	}
	
	public static int getXNodeIndexFromOffset(float xOffset) {
		return (int) Math.floor(xOffset/tileWidth);
	}
	
	public static int getYNodeIndexFromOffset(float yOffset) {
		return (int) Math.floor(yOffset/tileHeight);
	}
	
	public static int getXOffsetFromNodePos(int xNode) {
	    return (int) xNode * tileWidth + (tileWidth/2);
	}
    
    public static int getYOffsetFromNodePos(int yNode) {
        return (int) yNode * tileHeight + (tileHeight/2);
    }
}
