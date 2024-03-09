package algorithm.tote_selection;

import Entity.Location;
import algorithm.Solution;
import algorithm.routing.Combined;
import algorithm.routing.Routing;
import gurobi.GRBException;
import instance_generation.Instance;

import java.io.IOException;
import java.util.*;

/*
Step1.Order the SKUs in s by a rule(such as LSA5-1 and LSA5-2)to give S reordered.
Step2.According to the ordering result above,check all storage locations for the SKUs and select the storage location
that minimizes the increment picking distance of the current V after adding it to the picking list as the picking
location of this SKU.
LSA5-2.Order by the number of storage locations of SKUs. Reorder the SKUs in S in ascending orderof V||s
 */
public class TSAlg implements ToteSelection{
    Instance instance;
    Routing routing;
    Set<Location>[] locationSetBySKU;
    List<Location>[] routeLocationList;

    public void init(Instance instance){
        this.instance = instance;
        this.routing = new Combined();
        this.routing.init(instance);
    }

    public void cal(Solution solution) throws IOException, GRBException {
        List<Integer> skuComeSeq = solution.skuComeSeq;
        this.locationSetBySKU = new Set[instance.skuNum];
        for(int s:skuComeSeq){
            this.locationSetBySKU[s] = new HashSet<>(instance.locationSetBySKU[s]);
        }
        int R = skuComeSeq.size() / instance.toteCapByRobot;
        if(skuComeSeq.size() % instance.toteCapByRobot > 0){
            R += 1;
        }
        routeLocationList = new List[R];
        double sumRouteTime = 0.0;
        List<Integer> subSKUComeSeq;
        solution.routeLen = new double[R];
        for(int r = 0;r<R;r++){
            subSKUComeSeq = skuComeSeq.subList(r*instance.toteCapByRobot,
                    Math.min((r+1)*instance.toteCapByRobot, skuComeSeq.size()));
            double routeTime =  calTotesAndRoute(subSKUComeSeq, r);
            sumRouteTime += routeTime;
            solution.routeLen[r] = routeTime;
        }
        sumRouteTime = sumRouteTime/instance.moveSpeed;
        sumRouteTime += instance.pickTime*skuComeSeq.size();
        solution.objVal = sumRouteTime;
        solution.routeList = routeLocationList.clone();
    }

    @Override
    public double calTotesAndRoute(List<Integer> subSKUComeSeq, int r){
//        this.locationSetBySKU = new Set[instance.skuNum];
//        for(int sku:subSKUComeSeq){
//            this.locationSetBySKU[sku] = new HashSet<>(instance.locationSetBySKU[sku]);
//        }
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

        routeLocationList[r] = locationsList;
        return min_rl;
    }

    public double calTotesAndRouteIndividually(List<Integer> subSKUComeSeq){
        this.locationSetBySKU = new Set[instance.skuNum];
        for(int sku:subSKUComeSeq){
            this.locationSetBySKU[sku] = new HashSet<>(instance.locationSetBySKU[sku]);
        }
        // LSA5-2.Order by the number of storage locations of SKUs. 根据升序排列
        int[][] SKULocationNum = new int[subSKUComeSeq.size()][2];
        for(int i = 0;i<subSKUComeSeq.size();i++){
            SKULocationNum[i][0] = subSKUComeSeq.get(i);
            SKULocationNum[i][1] = this.locationSetBySKU[subSKUComeSeq.get(i)].size();
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
            locationSetBySKU[SKULocationNum[idx][0]].remove(best_location);
            locationsList.add(best_location);
            idx++;
        }
        return min_rl;
    }
}
