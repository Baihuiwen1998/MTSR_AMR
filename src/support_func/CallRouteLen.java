package support_func;

import Entity.Location;
import algorithm.routing.Combined;
import algorithm.routing.Routing;
import gurobi.GRBException;
import instance_generation.Instance;

import java.io.IOException;
import java.util.*;
public class CallRouteLen {
    Instance instance;
    Routing routing;
    Set<Location>[] locationSetBySKU;
    public CallRouteLen( Instance instance){

        this.instance = instance;
        this.routing = new Combined();
        this.routing.init(instance);
    }
    public double calRouteLen(List<List<Integer>> skuComeRoutes, HashMap<Integer, Integer>skuUseNumMap) throws IOException, GRBException {
        // 可行解情况下,计算路径长
        this.locationSetBySKU = new Set[instance.skuNum];
        for(int s:skuUseNumMap.keySet()){
            this.locationSetBySKU[s] = new HashSet<>(instance.locationSetBySKU[s]);
        }
        double sumRouteLen = 0.0;
        List<Integer> subSKUComeSeq;
        for(int r = 0;r<skuComeRoutes.size();r++){
            subSKUComeSeq = skuComeRoutes.get(r);
            double routeTime =  calTotesAndRoute(subSKUComeSeq, r);
            sumRouteLen += routeTime;
        }
        return sumRouteLen;
    }

    public List<Double> calRouteLenList(List<List<Integer>> skuComeRoutes, HashMap<Integer, Integer>skuUseNumMap) throws IOException, GRBException {
        // 可行解情况下,计算路径长
        this.locationSetBySKU = new Set[instance.skuNum];
        for(int s:skuUseNumMap.keySet()){
            this.locationSetBySKU[s] = new HashSet<>(instance.locationSetBySKU[s]);
        }
        List<Double> routeLenList = new ArrayList<>();
        List<Integer> subSKUComeSeq;
        for(int r = 0;r<skuComeRoutes.size();r++){
            subSKUComeSeq = skuComeRoutes.get(r);
            double routeTime =  calTotesAndRoute(subSKUComeSeq, r);
            routeLenList.add(routeTime);
        }
        return routeLenList;
    }

    public double calTotesAndRoute(List<Integer> subSKUComeSeq, int r){
        // LSA5-2.Order by the number of storage locations of SKUs. 根据升序排列
        int[][] SKULocationNum = new int[subSKUComeSeq.size()][2];
        int sku;
        for(int i = 0;i<subSKUComeSeq.size();i++){
            sku = subSKUComeSeq.get(i);
            SKULocationNum[i][0] = sku;
            SKULocationNum[i][1] = this.locationSetBySKU[sku].size();
        }
        Arrays.sort(SKULocationNum, new Comparator<int[]>() {
            @Override
            public int compare(int[] o1, int[] o2) {
                return o1[1]-o2[1];
            }
        });

        // According to the ordering result above,check all storage locations for the SKUs and select the storage location
        // that minimizes the increment picking distance of the current V after adding it to the picking list as the picking
        // location of this SKU.
        List<Location> locationsList = new ArrayList<>();
        List<Location> tmpLocationsList;

        double tmp_rl;
        double min_rl =Double.MAX_VALUE;
        Location best_location;
        Set<Location> locationsForSelection;
        int idx = 0;
        while(idx<subSKUComeSeq.size()){
            min_rl = Double.MAX_VALUE;
            best_location = null;
            locationsForSelection= locationSetBySKU[SKULocationNum[idx][0]];
            for(Location location:locationsForSelection){
                tmpLocationsList = new ArrayList<>(locationsList);
                tmpLocationsList.add(location);
                tmp_rl = routing.getRouteLength(tmpLocationsList);
                if(tmp_rl<min_rl){
                    min_rl = tmp_rl;
                    best_location = location;
                }
            }
            this.locationSetBySKU[SKULocationNum[idx][0]].remove(best_location);
            locationsList.add(best_location);
            idx++;
        }
        return min_rl;
    }

}
