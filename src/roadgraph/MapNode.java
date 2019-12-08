package roadgraph;

import geography.GeographicPoint;

import java.util.LinkedList;
import java.util.List;

/**
 * Created by ahr_a on 11/24/2019.
 */
public class MapNode extends MapGraph implements Comparable<MapNode>{
    //location of.. say a point A
 private  GeographicPoint location;
    //edges related to A
  private  List<MapEdge> edges;
  private double distance;
  private double estimatedDistance;


    public MapNode(GeographicPoint loc)
    {
        this.location=loc;
        edges=new LinkedList<MapEdge>();
        distance=0;
        estimatedDistance=0;
    }
    public int compareTo(MapNode m) {
        return ((Double)this.getDistance()).compareTo(m.getDistance());
    }

    //This method is used to add edges to a MapNode instance
    public void addEdge(MapEdge edge)
    {
      edges.add(edge);
    }
    //Getter for the GeographicPoint location
    public GeographicPoint getLoc()
    {
        return location;
    }
    //Getter fo a Linked list of edges
    public List<MapEdge> getEdges()
    {
        return edges;
    }
    public void setDistance(double dist)
    {
        this.distance=dist;
    }
    public void setEstimatedDistance(double distance)
    {
        this.estimatedDistance=distance;
    }
    public double getDistance()
    {
        return distance;
    }
    public double getEstimatedDistance()
    {
        return estimatedDistance;
    }


}
