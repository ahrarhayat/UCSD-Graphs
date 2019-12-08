/**
 * @author UCSD MOOC development team and YOU
 *
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 *
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3(DONE)
    //This is a hashmap with the location of the start point, and a MapNode with all the edges related to it
    private Map<GeographicPoint, MapNode> vertices;
    private Set<MapEdge> edges;




	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3 (DONE)
		this.vertices=new HashMap<GeographicPoint,MapNode>();
		this.edges=new HashSet<MapEdge>();



	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3(DONE)
		return vertices.values().size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3(DONE)
        HashSet<GeographicPoint> result= new HashSet<GeographicPoint>();
        //add all the vertices into the result HashSet and return
        result.addAll(vertices.keySet());
		return result;
	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3 (DONE)
		return edges.size();
	}



	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3 (DONE)
      if (!vertices.containsKey(location) && location!=null)
      {
          MapNode vertex=new MapNode(location);
          vertices.put(location,vertex);
          return true;
      }

		return false;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2.
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
	    if (vertices.containsKey(from) && vertices.containsKey(to) && from!=null && to!=null)
        {
            //System.out.println("From"+ from.toString()+ " to"+to.toString());
            //Get the start vertex
            MapNode start=vertices.get(from);
            MapNode end=vertices.get(to);
            //Create an edge between the two points from and to
            MapEdge link=new MapEdge(start,end,roadName,length,roadType);
            //Add the edge to the vertex
            start.addEdge(link);
            edges.add(link);
        }



		//TODO: Implement this method in WEEK 3 (DONE)

	}


	/** Find the path from start to goal using breadth first search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}

	/** Find the path from start to goal using breadth first search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start,
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3(DONE)
        if (start==null || goal==null) {
        	System.out.println("Route was never searched since the input was invalid");
        	return null;
		}
		MapNode begin=vertices.get(start);
		MapNode end=vertices.get(goal);
		HashMap<MapNode,MapNode> way = new HashMap<MapNode,MapNode>();

		boolean check=checkBFS(start,goal,nodeSearched,way);
		if (!check)
		{
			System.out.println("Route was not found after searching");
			return null;
		}
		List<GeographicPoint> result=findWay(begin,end,way);
		return result;
	}
	List<GeographicPoint> findWay(MapNode start, MapNode end, HashMap<MapNode,MapNode> way)
    {
        MapNode current=way.get(end);
        LinkedList<GeographicPoint> route=new LinkedList<GeographicPoint>();
        route.addFirst(end.getLoc());
        while (!current.equals(start))
        {
            route.addFirst(current.getLoc());
            current=way.get(current);
        }
		route.addFirst(start.getLoc());
        return route;
    }
    boolean checkBFS(GeographicPoint start,
					 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched,HashMap<MapNode,MapNode> way)
	{
		Queue<MapNode> lineUp = new LinkedList<MapNode>();
		MapNode begin=vertices.get(start);
		MapNode end=vertices.get(goal);
		HashSet<MapNode> visited = new HashSet<MapNode>();
		lineUp.add(begin);
		MapNode current;

		while (!lineUp.isEmpty()) {
			current=lineUp.poll();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(current.getLoc());

			//if current is the goal return true
			if (current.getLoc().x==end.getLoc().x && current.getLoc().y==end.getLoc().y)
			{
				return true;
			}
			//search for neighbor nodes and put them on the visited list if they are not there already and put the MapNodes on either side of the route
			//in the way HashMap
			for (MapEdge edge:current.getEdges())
			{

				MapNode x = edge.getEnd();
				if (!visited.contains(x))
				{
					visited.add(x);
					lineUp.add(x);
					//System.out.println("Nodes put into visited: "+vertices.get(edge.getLoc(current.getLoc())).getLoc());
					//System.out.println("Start: "+vertices.get(edge.getLoc(current.getLoc())).getLoc()+" End "+current.getLoc());
					way.put(x,current);
				}
			}

		}

		return false;
	}




	/** Find the path from start to goal using Dijkstra's algorithm
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start,
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4(DONE)
		if (start==null || goal==null) {
			System.out.println("Route was never searched since the input was invalid");
			return null;
		}

		MapNode begin=vertices.get(start);
		MapNode end=vertices.get(goal);

		HashMap<MapNode,MapNode> way = new HashMap<MapNode,MapNode>();
		for (MapNode n : vertices.values()) {
			n.setDistance(Double.POSITIVE_INFINITY);
		}

		boolean check=checkDijkstra(begin,end,nodeSearched,way);
		if (!check)
		{
			System.out.println("Route was not found after searching");
			return null;
		}
		List<GeographicPoint> result=findWay(begin,end,way);

		return result;



	}
	private  static boolean checkDijkstra(MapNode start,MapNode goal,Consumer<GeographicPoint> nodeSearched,HashMap<MapNode,MapNode> way)
	{
        //initialize priority queue
        PriorityQueue<MapNode> pq = new PriorityQueue<MapNode>();
        ////initialize visited set
        HashSet<MapNode> visited = new HashSet<MapNode>();
        start.setDistance(0);
        pq.add(start);
        int numOfLocations=0;
        //while pq is not empty
        while (!pq.isEmpty()) {
            MapNode current = pq.remove();
            System.out.println("DIJKSTRA visiting[NODE at location "+current.getLoc()+ "intersects streets: ");
            numOfLocations++;
            nodeSearched.accept(current.getLoc());
            if (current.equals(goal)) {
                System.out.println("number of Locations: "+numOfLocations);
               return true;
            }

            if(!visited.contains(current)) {
                visited.add(current);

                for (MapEdge edge : current.getEdges()) {
                    MapNode neighbor = edge.getEnd();
                    if (!visited.contains(neighbor)) {
                        double weight = edge.getDistance() + current.getDistance();
                        if (weight < neighbor.getDistance()) {
                            way.put(neighbor, current);
                            neighbor.setDistance(weight);
                            pq.add(neighbor);
                        }
                    }
                }
            }
        }
        //return false if goal is not found
        System.out.println("number of Locations: "+numOfLocations);
        return false;
	}


	/** Find the path from start to goal using A-Star search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}

	/** Find the path from start to goal using A-Star search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start,
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
        if (start==null || goal==null) {
            System.out.println("Route was never searched since the input was invalid");
            return null;
        }

        MapNode begin=vertices.get(start);
        MapNode end=vertices.get(goal);

        HashMap<MapNode,MapNode> way = new HashMap<MapNode,MapNode>();
        for (MapNode n : vertices.values()) {
            n.setDistance(Double.POSITIVE_INFINITY);
            n.setEstimatedDistance(Double.POSITIVE_INFINITY);
        }

        boolean check=aStarCheck(begin,end,nodeSearched,way,goal);
        if (!check)
        {
            System.out.println("Route was not found after searching");
            return null;
        }
        List<GeographicPoint> result=findWay(begin,end,way);

        return result;

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

	}
	private boolean aStarCheck(MapNode start,MapNode goal,Consumer<GeographicPoint> nodeSearched,HashMap<MapNode,MapNode> way,GeographicPoint dest)
    { //initialize priority queue
        PriorityQueue<MapNode> pq = new PriorityQueue<MapNode>();
        ////initialize visited set
        HashSet<MapNode> visited = new HashSet<MapNode>();
        pq.add(start);
        start.setDistance(0);
        start.setEstimatedDistance(0);

        pq.add(start);

        int numofLocs = 0;


        while (!pq.isEmpty()) {
            MapNode current = pq.remove();
            numofLocs++;
            nodeSearched.accept(current.getLoc());
            System.out.println("A* visiting" + current.getLoc()+"Actual = "+current.getEstimatedDistance()+", Pred: "+current.getDistance()
            +"intersects streets: ");
            if (current.equals(goal)) {
                numofLocs--;
                System.out.println("Nodes visited in search: "+numofLocs);
                return true;

            }
            if(!visited.contains(current)) {
                visited.add(current);

                for (MapEdge edge : current.getEdges()) {
                    MapNode neighbor = edge.getEnd();
                    if (!visited.contains(neighbor)) {

                        double currDist = edge.getDistance()+current.getEstimatedDistance();

                        double predDist = currDist+ (neighbor.getLoc()).distance(goal.getLoc());
                        if(predDist < neighbor.getDistance()){
                            way.put(neighbor, current);
                            neighbor.setEstimatedDistance(currDist);
                            neighbor.setDistance(predDist);
                            pq.add(neighbor);
                        }
                    }
                }
            }
        }
        numofLocs--;
        System.out.println("Nodes visited in search: "+numofLocs);
        return false;

    }





	public static void main(String[] args)
	{
        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
        System.out.println("DONE.");

        GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
        GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

        List<GeographicPoint> route = theMap.dijkstra(start,end);
        List<GeographicPoint> route2 = theMap.aStarSearch(start,end);


		// You can use this method for testing.


		/* Here are some test cases you should try before you attempt
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);


		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);


		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/


		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/

	}

}
