package algorithm.cal_ftness;

import algorithm.AlgorithmParams;
import algorithm.Solution;
import algorithm.routing.Combined;
import algorithm.routing.Routing;
import algorithm.sku_sequencing.SKUScheduling;
import algorithm.sku_sequencing.SSGreedy;
import algorithm.sku_sequencing.SSMIP;
import algorithm.tote_selection.TSAlg;
import algorithm.tote_selection.TSMIP;
import algorithm.tote_selection.ToteSelection;
import gurobi.GRBException;
import instance_generation.Instance;
import instance_generation.InstanceParams;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.*;

public class CFImpr implements CalFitness{
    int[] orderSeq;
    static Instance instance;
    SKUScheduling skuScheduling;
    Routing routing;
    ToteSelection toteSelection;
    double bestObj;
    int iter;
    boolean isMoreImpr;

    HashMap<Integer,Integer> skuUsedNumMap;
    Set<Integer> skuUsedOnlyOnce;
    boolean[] routeIsOnlyCalSubRouteEff;


    @Override
    public void init(Instance instance){
        this.instance = instance;
        if(AlgorithmParams.skuSchedulingAlgMode == 0){
            skuScheduling = new SSMIP();
        }else if (AlgorithmParams.skuSchedulingAlgMode == 1) {
            skuScheduling = new SSGreedy();
        }
        this.skuScheduling.init(instance);
        if(AlgorithmParams.toteSelectionAlgMode == 0){
            toteSelection = new TSMIP();
        } else if (AlgorithmParams.toteSelectionAlgMode == 1) {
            toteSelection = new TSAlg();
        }
        this.toteSelection.init(instance);
        this.routing = new Combined();
        this.routing.init(instance);
    }

    @Override
    public Solution cal(Solution solution) throws IOException, GRBException {
        this.orderSeq = solution.orderSeq;
        List<Integer> skuComeSeq = solution.skuComeSeq;
        // 声明变量
        iter=0;
        bestObj = solution.objVal;
        double removedRouteTime;
        isMoreImpr = true;
        while(iter< AlgorithmParams.maxSSAImprIteraion && isMoreImpr) {
            iter++;
            isMoreImpr = false;
            /* 采用worst-distance-destroy & greedy-insertion 算法，对获得的初始SKUComeSeq进行改进 */
            double[][] savedDistance = new double[skuComeSeq.size()][2];
            int R = (int) Math.ceil((double) skuComeSeq.size() / (double) instance.toteCapByRobot);

            // Step 1: 获得每个route中，移除sku后Route的路径长节约数
            int skuIdx = 0;
            List<Integer> subSKUList;
            List<Integer> removedSKUList = new ArrayList<>();
            for (int r = 0; r < R; r++) {
                subSKUList = skuComeSeq.subList(r*instance.toteCapByRobot, Math.min((r+1)*instance.toteCapByRobot, skuComeSeq.size()));
                for(int i=0;i<subSKUList.size();i++){
                    if(subSKUList.size()==1){
                        removedRouteTime = 0.0;
                    }else {
                        removedSKUList.clear();
                        removedSKUList.addAll(subSKUList.subList(0,i));
                        removedSKUList.addAll(subSKUList.subList(i+1,subSKUList.size()));
                        removedRouteTime = this.toteSelection.calTotesAndRouteIndividually(removedSKUList);
                    }
                    savedDistance[skuIdx] = new double[]{skuIdx, solution.routeLen[r] - removedRouteTime};
                    skuIdx++;
                }
            }
            Arrays.sort(savedDistance, new Comparator<double[]>() {
                @Override
                public int compare(double[] o1, double[] o2) {
                    return (int) ((o2[1] - o1[1]) * 10);
                }
            });
            // Step 2: 选择拆除后saving最多的且属于不同route的sku交换位置，检验是否可行
            solution = destroyRepairSKUComeSeq(savedDistance, skuComeSeq, solution, R);
            skuComeSeq = solution.skuComeSeq;
        }
        solution.orderSeq = this.orderSeq;
        return solution;
    }
    public Solution destroyRepairSKUComeSeq(double[][] savedDistance, List<Integer> skuComeSeq, Solution solution, int R) throws IOException, GRBException {
        int findSKUPairNum = 0;
        int idx1 = 0;
        int idx2 = 1;
        double tmpObj = Double.MAX_VALUE;
        // 声明变量
        List<Integer> swappedSKUComeSeq, subSKUList;
        int skuIdx1, skuIdx2, r1, r2;
        double changedRouteLen_1, changedRouteLen_2;
        Solution tmpSolution;
        // preCalculation
        skuUsedNumMap = listToCntMap(skuComeSeq);
        skuUsedOnlyOnce = new HashSet<>();
        for(int sku:skuUsedNumMap.keySet()){
            if(skuUsedNumMap.get(sku) == 1){
                skuUsedOnlyOnce.add(sku);
            }
        }
        routeIsOnlyCalSubRouteEff = new boolean[R];
        Arrays.fill(routeIsOnlyCalSubRouteEff, true);
        for(int r = 0;r<R;r++) {
            for (int k = instance.toteCapByRobot * r; k < Math.min(instance.toteCapByRobot * (r + 1), skuComeSeq.size()); k++) {
                if (!skuUsedOnlyOnce.contains(skuComeSeq.get(k))) {
                    routeIsOnlyCalSubRouteEff[r] = false;
                    break;
                }
            }
        }
        while (findSKUPairNum < AlgorithmParams.maxSSAImprPairs && idx2 < skuComeSeq.size()) {
            skuIdx1 = Math.min((int) savedDistance[idx1][0], (int) savedDistance[idx2][0]);
            skuIdx2 = Math.max((int) savedDistance[idx1][0], (int) savedDistance[idx2][0]);
            if (skuIdx2 < skuComeSeq.size()) { // (savedDistance[idx1][1] != 0 | savedDistance[idx2][1] != 0) &
                r1 = skuIdx1 / instance.toteCapByRobot;
                r2 = skuIdx2 / instance.toteCapByRobot;
                if (r1 != r2) {
                    // 计算交换后的路径长
                    swappedSKUComeSeq = new ArrayList<>(skuComeSeq.subList(0, skuIdx1));
                    swappedSKUComeSeq.add(skuComeSeq.get(skuIdx2));
                    swappedSKUComeSeq.addAll(skuComeSeq.subList(skuIdx1 + 1, skuIdx2));
                    swappedSKUComeSeq.add(skuComeSeq.get(skuIdx1));
                    swappedSKUComeSeq.addAll(skuComeSeq.subList(skuIdx2 + 1, skuComeSeq.size()));
                    // 判断是否可以只计算交换的subRoute
                    if(routeIsOnlyCalSubRouteEff[r1] && routeIsOnlyCalSubRouteEff[r2]){
                        // 如果可以直接计算subRoute
                        subSKUList = swappedSKUComeSeq.subList(r1*instance.toteCapByRobot, Math.min((r1+1)*instance.toteCapByRobot, skuComeSeq.size()));
                        changedRouteLen_1 =  this.toteSelection.calTotesAndRouteIndividually(subSKUList);

                        subSKUList = swappedSKUComeSeq.subList(r2*instance.toteCapByRobot, Math.min((r2+1)*instance.toteCapByRobot, skuComeSeq.size()));
                        changedRouteLen_2 = this.toteSelection.calTotesAndRouteIndividually(subSKUList);
                        tmpObj = solution.objVal + (changedRouteLen_1-solution.routeLen[r1] + changedRouteLen_2- solution.routeLen[r2]) / InstanceParams.moveSpeed;
                    }else {
                        // 如果不能直接计算subRoute
                        tmpSolution = new Solution();
                        tmpSolution.skuComeSeq = swappedSKUComeSeq;//.subList(0, skuComeSeqLen);
                        this.toteSelection.cal(tmpSolution);
                        tmpObj = tmpSolution.objVal;
                    }
                    if (tmpObj <bestObj) {
                        // 检验是否可行
                        int skuComeSeqLen = checkIfFeasibleByGreedy(swappedSKUComeSeq);
                        if (skuComeSeqLen > 0) {
                            tmpSolution = new Solution();
                            tmpSolution.skuComeSeq = swappedSKUComeSeq.subList(0, skuComeSeqLen);
                            this.toteSelection.cal(tmpSolution);
                            // 更新skuUsedNumMap
                            for(int idx=skuComeSeqLen;idx<swappedSKUComeSeq.size();idx++){
                                int sku = swappedSKUComeSeq.get(idx);
                                skuUsedNumMap.put(sku, skuUsedNumMap.get(sku)-1);
                            }
                            // 更新route的属性
                            if(!(routeIsOnlyCalSubRouteEff[r1] && routeIsOnlyCalSubRouteEff[r2])) {
                                changeIfRouteSKUUseNumChanged(r1, tmpSolution.skuComeSeq);
                                changeIfRouteSKUUseNumChanged(r2, tmpSolution.skuComeSeq);
                            }
                            // 更新最优解值
                            bestObj = tmpSolution.objVal;
                            solution = tmpSolution;
                            skuComeSeq = solution.skuComeSeq;
                            findSKUPairNum++;
                            // 对savedDistance重排
                            savedDistance[idx1][1] = Integer.MIN_VALUE;
                            savedDistance[idx2][1] = Integer.MIN_VALUE;
//                            System.out.println("idx_1=" + idx1 + ",idx_2=" + idx2);
                            // 降序排序
                            Arrays.sort(savedDistance, new Comparator<double[]>() {
                                @Override
                                public int compare(double[] o1, double[] o2) {
                                    return (int) ((o2[1] - o1[1]) * 10);
                                }
                            });
                            idx2 = idx1 + 1;
                            isMoreImpr = true; // 可以继续迭代优化
//                            System.out.println("findSKUPairNum=" + findSKUPairNum + ", improvedSolution=" + solution.objVal);
                            //                            solution.orderSeq = getOrderSeq(swappedSKUComeSeq);
                        }
                    }
                }
            }
            idx2++;
            if (idx2 == skuComeSeq.size()) {
                idx1++;
                idx2 = idx1 + 1;
            }
        }
        return solution;
    }

    public void changeIfRouteSKUUseNumChanged(int r, List<Integer> skuComeSeq){
        // 检查
        boolean check = true;
        for (int k = instance.toteCapByRobot * r; k < Math.min(instance.toteCapByRobot * (r + 1), skuComeSeq.size()); k++) {
            if (!skuUsedOnlyOnce.contains(skuComeSeq.get(k))) {
                check = false;
                break;
            }
        }
        this.routeIsOnlyCalSubRouteEff[r] = check;
    }

    public static HashMap<Integer, Integer> listToCntMap(List<Integer> list){
        HashMap<Integer, Integer> map = new HashMap<>();
        for(int i:list){
            map.put(i,map.getOrDefault(i,0)+1);
        }
        return map;
    }

    public int checkIfFeasibleByGreedy(List<Integer> skuComeSeq){
        HashMap<Integer, Set<Integer>> pickingOrders = new HashMap<>();
        HashSet<Integer> skuRemain =new HashSet<>();
        Set<Integer> remainingOrders = new HashSet<>();
        for(int i = 0;i<this.instance.orderNum;i++){
            remainingOrders.add(i);
        }
        int nextSKUIndex = this.instance.toteCapByStation;
        Set<Integer> activeSKU = new HashSet<>(skuComeSeq.subList(0, this.instance.toteCapByStation));
        HashSet<Integer> curOrderDemand;
        int most_unneeded_sku = -1;
        int nextOrder;
        ArrayList<Integer> iterList = new ArrayList<>();
        Set<Integer> removeSKUSet;
        while(remainingOrders.size()>0 || skuRemain.size()>0){
            //订单拣选
            while(pickingOrders.size()<this.instance.orderBinCap && remainingOrders.size()>0){
                nextOrder = calBestSimilarOrder(skuComeSeq.subList(nextSKUIndex,Math.min(nextSKUIndex+2,skuComeSeq.size())),remainingOrders,activeSKU);
                remainingOrders.remove(nextOrder);
                curOrderDemand = new HashSet<>(instance.skuSetByOrder[nextOrder]);
                curOrderDemand.removeAll(activeSKU);
                if(curOrderDemand.size()>0) {
                    skuRemain.addAll(curOrderDemand);
                    pickingOrders.put(nextOrder, curOrderDemand);
                }
                removeSKUSet = new HashSet<>();
                // 检查是否有sku完成了所有要拣选的订单任务，可以直接从activeSKU中删除
                for (int sku : activeSKU) {
                    if (checkIfSkuUseless(sku, remainingOrders)) {
                        removeSKUSet.add(sku);
                    }
                }
                // 从activeSKU中删除
                int removeSKUNum = removeSKUSet.size();
                if (removeSKUNum > 0) {
                    activeSKU.removeAll(removeSKUSet);
                    while(activeSKU.size()<instance.toteCapByStation && nextSKUIndex != skuComeSeq.size()){
                        activeSKU.add(skuComeSeq.get(nextSKUIndex));
                        nextSKUIndex++;
                    }
                    // 对所有pickingOrder进行拣选
                    iterList.clear();
                    iterList.addAll(pickingOrders.keySet());
                    //针对当前pickingOrders里面的订单拣选
                    for (int key : iterList) {
                        pickingOrders.get(key).removeAll(activeSKU);//对订单进行拣选
                        skuRemain.removeAll(activeSKU);
                        if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                            pickingOrders.remove(key);//删除该订单
                        }
                    }
                }
            }
            if(pickingOrders.size()==0&&remainingOrders.size()==0){
                return nextSKUIndex;
            }
            most_unneeded_sku = getMostUnneededSKU(remainingOrders, activeSKU, skuComeSeq.subList(nextSKUIndex, skuComeSeq.size()));
            if(most_unneeded_sku==-1){
                return -1;
            }
            activeSKU.remove(most_unneeded_sku);
           if(nextSKUIndex==skuComeSeq.size()){
                return -1;
            }
            activeSKU.add(skuComeSeq.get(nextSKUIndex));
            iterList.clear();
            iterList.addAll(pickingOrders.keySet());
            //针对当前pickingOrders里面的订单拣选
            for (int key : iterList) {
                pickingOrders.get(key).remove(skuComeSeq.get(nextSKUIndex));//对订单进行拣选
                skuRemain.remove(skuComeSeq.get(nextSKUIndex));
                if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                    pickingOrders.remove(key);//删除该订单
                }
            }
            nextSKUIndex++;
            if(pickingOrders.size()==0&&remainingOrders.size()==0){
                return nextSKUIndex;
            }
        }
        return  nextSKUIndex;
    }

    public boolean checkIfSkuUseless(int sku, Set<Integer> remainingOrders){
        Set<Integer> unFinishedOrder = new HashSet<>(instance.orderSetBySKU[sku]);
        unFinishedOrder.retainAll(remainingOrders); // 无需考虑pickingOrder，cuz该订单必然已经拣选了activeSKU集合中的sku
        if(unFinishedOrder.size()==0){
            return true;
        }else{
            return false;
        }
    }

    public int[][] getPickRecord(List<Integer> skuComeSeq){
        HashMap<Integer, Set<Integer>> pickingOrders = new HashMap<>();
        HashSet<Integer> skuRemain =new HashSet<>();
        Set<Integer> remainingOrders = new HashSet<>();
        int[][] pickK = new int[this.instance.orderNum][this.instance.skuNum];

        for(int i = 0;i<this.instance.orderNum;i++){
            remainingOrders.add(i);
            Arrays.fill(pickK[i], 1000);
        }

        int nextSKUIndex = this.instance.toteCapByStation;
        Set<Integer> activeSKU = new HashSet<>(skuComeSeq.subList(0, this.instance.toteCapByStation));
        HashSet<Integer> curOrderDemand;
        int most_unneeded_sku = -1;
        int nextOrder;
        ArrayList<Integer> iterList = new ArrayList<>();
        Set<Integer> removeSKUSet;
        while(remainingOrders.size()>0 || skuRemain.size()>0){
            //订单拣选
            while(pickingOrders.size()<this.instance.orderBinCap && remainingOrders.size()>0){
                nextOrder = calBestSimilarOrder(skuComeSeq.subList(nextSKUIndex,Math.min(nextSKUIndex+2,skuComeSeq.size())),remainingOrders,activeSKU);
                remainingOrders.remove(nextOrder);
                curOrderDemand = new HashSet<>(instance.skuSetByOrder[nextOrder]);
                curOrderDemand.removeAll(activeSKU);
                for(int s:activeSKU){
                    pickK[nextOrder][s] = Math.min(pickK[nextOrder][s], nextSKUIndex-1);
                }
                if(curOrderDemand.size()>0) {
                    skuRemain.addAll(curOrderDemand);
                    pickingOrders.put(nextOrder, curOrderDemand);
                }
                removeSKUSet = new HashSet<>();
                // 检查是否有sku完成了所有要拣选的订单任务，可以直接从activeSKU中删除
                for (int sku : activeSKU) {
                    if (checkIfSkuUseless(sku, remainingOrders)) {
                        removeSKUSet.add(sku);
                    }
                }
                // 从activeSKU中删除
                int removeSKUNum = removeSKUSet.size();
                if (removeSKUNum > 0) {
                    activeSKU.removeAll(removeSKUSet);
                    while(activeSKU.size()<instance.toteCapByStation && nextSKUIndex != skuComeSeq.size()){
                        activeSKU.add(skuComeSeq.get(nextSKUIndex));
                        nextSKUIndex++;
                    }
                    // 对所有pickingOrder进行拣选
                    iterList.clear();
                    iterList.addAll(pickingOrders.keySet());
                    //针对当前pickingOrders里面的订单拣选
                    for (int key : iterList) {
                        pickingOrders.get(key).removeAll(activeSKU);//对订单进行拣选
                        for(int s:activeSKU){
                            pickK[key][s] = Math.min(pickK[key][s], nextSKUIndex-1);
                        }
                        skuRemain.removeAll(activeSKU);
                        if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                            pickingOrders.remove(key);//删除该订单
                        }
                    }
                }
            }
            if(pickingOrders.size()==0&&remainingOrders.size()==0){
                return pickK;
            }
            most_unneeded_sku = getMostUnneededSKU(remainingOrders, activeSKU, skuComeSeq.subList(nextSKUIndex, skuComeSeq.size()));
            if(most_unneeded_sku==-1){
                return null;
            }
            activeSKU.remove(most_unneeded_sku);
            if(nextSKUIndex==skuComeSeq.size()){
                return null;
            }
            activeSKU.add(skuComeSeq.get(nextSKUIndex));
            iterList.clear();
            iterList.addAll(pickingOrders.keySet());
            //针对当前pickingOrders里面的订单拣选
            for (int key : iterList) {
                int nextSKU = skuComeSeq.get(nextSKUIndex);
                pickingOrders.get(key).remove(nextSKU);//对订单进行拣选
                pickK[key][nextSKU] = Math.min(pickK[key][nextSKU], nextSKUIndex);
                System.out.println("o = "+key+", sku="+ nextSKU+ ", k = "+(nextSKUIndex)+ "record k ="+pickK[key][nextSKU]);
                skuRemain.remove(skuComeSeq.get(nextSKUIndex));
                if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                    pickingOrders.remove(key);//删除该订单
                }
            }
            nextSKUIndex++;
            if(pickingOrders.size()==0&&remainingOrders.size()==0){
                return pickK;
            }
        }
        return  pickK;
    }

    public int[][] getOrderSeq(List<Integer> skuComeSeq){
        HashMap<Integer, Set<Integer>> pickingOrders = new HashMap<>();
        HashSet<Integer> skuRemain =new HashSet<>();
        Set<Integer> remainingOrders = new HashSet<>();
        int[][] orderSeq = new int[this.instance.orderNum][2];

        for(int i = 0;i<this.instance.orderNum;i++){
            remainingOrders.add(i);
        }
        int nextSKUIndex = this.instance.toteCapByStation;
        Set<Integer> activeSKU = new HashSet<>(skuComeSeq.subList(0, this.instance.toteCapByStation));
        HashSet<Integer> curOrderDemand;
        int most_unneeded_sku = -1;
        int nextOrder;
        ArrayList<Integer> iterList = new ArrayList<>();
        Set<Integer> removeSKUSet;
        while(remainingOrders.size()>0 || skuRemain.size()>0){
            //订单拣选
            while(pickingOrders.size()<this.instance.orderBinCap && remainingOrders.size()>0){
//                long start = System.currentTimeMillis();
                nextOrder = calBestSimilarOrder(skuComeSeq.subList(nextSKUIndex,Math.min(nextSKUIndex+2,skuComeSeq.size())),remainingOrders,activeSKU);
//                long end = System.currentTimeMillis();
//                this.timeCostTest += end-start;
                orderSeq[nextOrder][0] = nextSKUIndex-1;
                System.out.print(nextOrder+", ");
                System.out.println(activeSKU.toString());
                remainingOrders.remove(nextOrder);
                curOrderDemand = new HashSet<>(instance.skuSetByOrder[nextOrder]);
                curOrderDemand.removeAll(activeSKU);
                if(curOrderDemand.size()>0) {
                    skuRemain.addAll(curOrderDemand);
                    pickingOrders.put(nextOrder, curOrderDemand);
                }else{
                    orderSeq[nextOrder][1] = nextSKUIndex-1;
                }
                removeSKUSet = new HashSet<>();
                // 检查是否有sku完成了所有要拣选的订单任务，可以直接从activeSKU中删除
                for (int sku : activeSKU) {
                    if (checkIfSkuUseless(sku, remainingOrders)) {
                        removeSKUSet.add(sku);
                    }
                }
                // 从activeSKU中删除
                int removeSKUNum = removeSKUSet.size();
                if (removeSKUNum > 0) {
                    activeSKU.removeAll(removeSKUSet);
                    while(activeSKU.size()<instance.toteCapByStation && nextSKUIndex != skuComeSeq.size()){
                        activeSKU.add(skuComeSeq.get(nextSKUIndex));
                        nextSKUIndex++;
                    }
                    // 对所有pickingOrder进行拣选
                    iterList.clear();
                    iterList.addAll(pickingOrders.keySet());
                    //针对当前pickingOrders里面的订单拣选
                    for (int key : iterList) {
                        pickingOrders.get(key).removeAll(activeSKU);//对订单进行拣选
                        skuRemain.removeAll(activeSKU);
                        if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                            orderSeq[key][1] = nextSKUIndex-1;
                            pickingOrders.remove(key);//删除该订单
                        }
                    }
                }
            }
            if(pickingOrders.size()==0&&remainingOrders.size()==0){
                return orderSeq;
            }
            most_unneeded_sku = getMostUnneededSKU(remainingOrders, activeSKU, skuComeSeq.subList(nextSKUIndex, skuComeSeq.size()));
            if(most_unneeded_sku==-1){
                return null;
            }
            activeSKU.remove(most_unneeded_sku);
            if(nextSKUIndex==skuComeSeq.size()){
                return null;
            }
            activeSKU.add(skuComeSeq.get(nextSKUIndex));
            iterList.clear();
            iterList.addAll(pickingOrders.keySet());
            //针对当前pickingOrders里面的订单拣选
            for (int key : iterList) {
                pickingOrders.get(key).remove(skuComeSeq.get(nextSKUIndex));//对订单进行拣选
                skuRemain.remove(skuComeSeq.get(nextSKUIndex));
                if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                    orderSeq[key][1] = nextSKUIndex;
                    pickingOrders.remove(key);//删除该订单
                }
            }
            nextSKUIndex++;
            if(pickingOrders.size()==0&&remainingOrders.size()==0){
                return orderSeq;
            }
        }
        return  orderSeq;
    }

    public List<Integer> getSKULeaveSeq(List<Integer> skuComeSeq){
        HashMap<Integer, Set<Integer>> pickingOrders = new HashMap<>();
        HashSet<Integer> skuRemain =new HashSet<>();
        Set<Integer> remainingOrders = new HashSet<>();
        List<Integer> skuLeaveSeq = new ArrayList<>();

        for(int i = 0;i<this.instance.orderNum;i++){
            remainingOrders.add(i);
        }
        int nextSKUIndex = this.instance.toteCapByStation;
        Set<Integer> activeSKU = new HashSet<>(skuComeSeq.subList(0, this.instance.toteCapByStation));
        int[] comeKOfSKU = new int[instance.skuNum];
        HashSet<Integer> curOrderDemand;
        int most_unneeded_sku = -1;
        int nextOrder;
        ArrayList<Integer> iterList = new ArrayList<>();
        Set<Integer> removeSKUSet;
        while(remainingOrders.size()>0 || skuRemain.size()>0){
            //订单拣选
            while(pickingOrders.size()<this.instance.orderBinCap && remainingOrders.size()>0){
//                long start = System.currentTimeMillis();
                nextOrder = calBestSimilarOrder(skuComeSeq.subList(nextSKUIndex,Math.min(nextSKUIndex+2,skuComeSeq.size())),remainingOrders,activeSKU);
//                long end = System.currentTimeMillis();
//                this.timeCostTest += end-start;
                remainingOrders.remove(nextOrder);
                curOrderDemand = new HashSet<>(instance.skuSetByOrder[nextOrder]);
                curOrderDemand.removeAll(activeSKU);
                if(curOrderDemand.size()>0) {
                    skuRemain.addAll(curOrderDemand);
                    pickingOrders.put(nextOrder, curOrderDemand);
                }
                removeSKUSet = new HashSet<>();
                // 检查是否有sku完成了所有要拣选的订单任务，可以直接从activeSKU中删除
                for (int sku : activeSKU) {
                    if (checkIfSkuUseless(sku, remainingOrders)) {
                        removeSKUSet.add(sku);
                    }
                }
                // 从activeSKU中删除
                int removeSKUNum = removeSKUSet.size();
                if (removeSKUNum > 0) {
                    activeSKU.removeAll(removeSKUSet);
                    // 按照进入的顺序添加到离开的顺序
                    if(removeSKUNum==1){
                        skuLeaveSeq.addAll(removeSKUSet);
                    }else{
                        int[][] skuComeK = new int[removeSKUNum][2];
                        int idx = 0;
                        for(int s:removeSKUSet){
                            skuComeK[idx][0] = s;
                            skuComeK[idx][1] =comeKOfSKU[s];
                            idx++;
                        }
                        Arrays.sort(skuComeK, new Comparator<int[]>() {
                            @Override
                            public int compare(int[] o1, int[] o2) {
                                return o1[1]-o2[1];
                            }
                        });
                        for(int[] sck:skuComeK){
                            skuLeaveSeq.add(sck[0]);
                        }
                    }


                    while(activeSKU.size()<instance.toteCapByStation && nextSKUIndex != skuComeSeq.size()){
                        int nextSKU = skuComeSeq.get(nextSKUIndex);
                        if(activeSKU.contains(nextSKU)){
                            skuLeaveSeq.add(nextSKU);
                        }
                        activeSKU.add(nextSKU);
                        comeKOfSKU[nextSKU] = nextSKUIndex;
                        nextSKUIndex++;
                    }
                    // 对所有pickingOrder进行拣选
                    iterList.clear();
                    iterList.addAll(pickingOrders.keySet());
                    //针对当前pickingOrders里面的订单拣选
                    for (int key : iterList) {
                        pickingOrders.get(key).removeAll(activeSKU);//对订单进行拣选
                        skuRemain.removeAll(activeSKU);
                        if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                            pickingOrders.remove(key);//删除该订单
                        }
                    }
                }
            }
            if(pickingOrders.size()==0&&remainingOrders.size()==0){
                return skuLeaveSeq;
            }
            most_unneeded_sku = getMostUnneededSKU(remainingOrders, activeSKU, skuComeSeq.subList(nextSKUIndex, skuComeSeq.size()));
            if(most_unneeded_sku==-1){
                return null;
            }
            activeSKU.remove(most_unneeded_sku);
            skuLeaveSeq.add(most_unneeded_sku);
            System.out.print(","+most_unneeded_sku+',');
            if(nextSKUIndex==skuComeSeq.size()){
                return null;
            }
            int nextSKU = skuComeSeq.get(nextSKUIndex);
            if(activeSKU.contains(nextSKU)){
                skuLeaveSeq.add(nextSKU);
            }
            activeSKU.add(nextSKU);
            comeKOfSKU[skuComeSeq.get(nextSKUIndex)] = nextSKUIndex;
            iterList.clear();
            iterList.addAll(pickingOrders.keySet());
            //针对当前pickingOrders里面的订单拣选
            for (int key : iterList) {
                pickingOrders.get(key).remove(skuComeSeq.get(nextSKUIndex));//对订单进行拣选
                skuRemain.remove(skuComeSeq.get(nextSKUIndex));
                if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                    pickingOrders.remove(key);//删除该订单
                }
            }
            nextSKUIndex++;
            if(pickingOrders.size()==0&&remainingOrders.size()==0){
                return skuLeaveSeq;
            }
        }
        return  skuLeaveSeq;
    }
    public int calBestSimilarOrder(List<Integer> newSKUComeSeq, Set<Integer> remainingOrders, Set<Integer> activeSKU){
        double bestSim= -1.0;
        int bestO=-1;
        double curSim;
        Set<Integer> intersection = new HashSet<>();
        int intersectionSize;
        int differenceSize;
        Set<Integer> ordersInConsiderationActive = new HashSet<>();
        Set<Integer> ordersInConsiderationFuture = new HashSet<>();
        Set<Integer> order;
        for(int sku:activeSKU){
            ordersInConsiderationActive.addAll(instance.orderSetBySKU[sku]);
        }
        ordersInConsiderationActive.retainAll(remainingOrders);

        for(int sku:newSKUComeSeq){
            ordersInConsiderationFuture.addAll(instance.orderSetBySKU[sku]);
        }
        ordersInConsiderationFuture.retainAll(remainingOrders);

        if(ordersInConsiderationActive.size()==0 && ordersInConsiderationFuture.size() ==0){
            return remainingOrders.iterator().next();
        }

        for(int o:ordersInConsiderationActive){
            order = this.instance.skuSetByOrder[o];
            intersection.clear();
            intersection.addAll(order);
            intersection.retainAll(activeSKU);
            intersectionSize = intersection.size();
            differenceSize = order.size() - intersectionSize;
            curSim = intersectionSize+ intersectionSize/Math.sqrt(intersectionSize*intersectionSize+differenceSize*differenceSize);
            if(curSim>0 && order.size()==1){
                return o;
            }
            if(ordersInConsiderationFuture.contains(o)){
                intersection.clear();
                intersection.addAll(order);
                intersection.retainAll(newSKUComeSeq);
                intersectionSize = intersection.size();
                differenceSize = order.size() - intersection.size();
                curSim += 0.3*intersectionSize/Math.sqrt(intersectionSize*intersectionSize+differenceSize*differenceSize);
            }
            if(curSim>bestSim){
                bestSim = curSim;
                bestO = o;
            }
        }
        for(int o:ordersInConsiderationFuture){
            if(!ordersInConsiderationActive.contains(o)) {
                order = this.instance.skuSetByOrder[o];
                intersection.clear();
                intersection.addAll(order);
                intersection.retainAll(newSKUComeSeq);
                intersectionSize = intersection.size();
                differenceSize = order.size() - intersection.size();
                curSim = 0.3 * intersectionSize / Math.sqrt(intersectionSize * intersectionSize + differenceSize * differenceSize);
                if (curSim > bestSim) {
                    bestSim = curSim;
                    bestO = o;
                }
            }
        }
        return bestO;
    }


    public int getMostUnneededSKU(Set<Integer> remainingOrders,  Set<Integer> activeSKU, List<Integer> newSKUComeSeq) {
        int most_unneeded_sku = -1;
        int most_unneeded_score = 100000;
        Map<Integer,Integer> skuDemandScore = new HashMap<>();
        Set<Integer> unFinishedOrder;

        for(int sku:activeSKU){
            unFinishedOrder = new HashSet<>(instance.orderSetBySKU[sku]);
            unFinishedOrder.retainAll(remainingOrders);
            if(newSKUComeSeq.contains(sku)){
                skuDemandScore.put(sku, unFinishedOrder.size() - 10000);
            }else{
                skuDemandScore.put(sku, unFinishedOrder.size());
            }
        }
        for(int sku:activeSKU){
            if((skuDemandScore.get(sku)<most_unneeded_score)){
                most_unneeded_sku = sku;
                most_unneeded_score = skuDemandScore.get(sku);
            }
        }
        if(most_unneeded_score > 0){
            return -1;
        }else {
            return most_unneeded_sku;
        }
    }



}
