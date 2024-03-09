package algorithm.routing;

import Entity.Location;
import instance_generation.Instance;

import java.util.List;

public interface Routing {
    public void init(Instance instance);
    public double getRouteLength(List<Location> locationsList);
}
