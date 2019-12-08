package roadgraph;

import geography.GeographicPoint;

/**
 * Created by ahr_a on 11/24/2019.
 */
public class MapEdge extends MapGraph {

    //start of say a point in question A
   private GeographicPoint start;
    //A particular instance will have differnet edges from A, so end will be B or Cs location
   private GeographicPoint end;
    //street name of say A to B
   private String StreetName;
    //distance from A to B
   private double distance;
    //type of the road
   private String RoadType;
   private MapNode begin;
   private MapNode finish;


    public MapEdge(MapNode first, MapNode last, String Name, double dist, String type)
    {
        this.StreetName=Name;
        this.distance=dist;
        this.RoadType=type;
        this.begin=first;
        this.finish=last;
    }
    //get the start node of a map edge
    public MapNode getStart()
    {
        return begin;
    }
    //getter for the end node of a Map edge
    public MapNode getEnd()
    {
        return finish;
    }
    public double getDistance()
    {
        return distance;
    }




}
