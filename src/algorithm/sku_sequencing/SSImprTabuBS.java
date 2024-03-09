package algorithm.sku_sequencing;


import algorithm.AlgorithmParams;
import algorithm.Operators;
import algorithm.Solution;
import algorithm.order_sequencing.BeamSearch;
import algorithm.routing.Combined;
import algorithm.routing.Routing;
import algorithm.tote_selection.TSAlg;
import algorithm.tote_selection.TSMIP;
import algorithm.tote_selection.ToteSelection;
import gurobi.GRBException;
import instance_generation.Instance;

import java.io.IOException;
import java.util.*;
/*
得到最终的SKUComeSeq之后，对其进行禁忌搜索/
 */
public class SSImprTabuBS {
    int[] orderSeq;
    Instance instance;
    SKUScheduling skuScheduling;
    Routing routing;
    ToteSelection toteSelection;
    Operators operator;

    int combinationNumSKU;
    int[][] skuCombination;
    BeamSearch beamSearch;
    List<String> tabuList = new ArrayList<>();

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
        this.operator = new Operators();
        this.beamSearch = new BeamSearch(instance);
    }

    public void calSKUCombination(int skuComeSeqSize){
        this.skuCombination = new int[skuComeSeqSize*(skuComeSeqSize-1)/2][2];
        int[] combi;
        this.combinationNumSKU = 0;
        for(int i= 0;i<skuComeSeqSize;i++){
            for(int j = i+1;j<skuComeSeqSize;j++){
                combi = new int[2];
                combi[0] = i;
                combi[1] = j;
                this.skuCombination[this.combinationNumSKU] = combi;
                this.combinationNumSKU++;
            }
        }
    }

    public Solution cal(Solution solution) throws IOException, GRBException, CloneNotSupportedException {
        List<Integer> skuComeSeq = solution.skuComeSeq;
        // 更新combination
        calSKUCombination(skuComeSeq.size());
        Solution neighSolution;
        Solution currentSolution = solution;
        int neigh;
        int iter = 0;
        while(iter< 100){
            iter++;
            neigh = iter%4 +1; // 定义邻域的序号
            neighSolution = findBestNeighbor(currentSolution.skuComeSeq, neigh, Math.min(this.combinationNumSKU, instance.orderNum));
            if (neighSolution == null) {
                continue;
            }
            if(neighSolution.skuComeSeq.size()<currentSolution.skuComeSeq.size()){
                calSKUCombination(neighSolution.skuComeSeq.size());
            }
            currentSolution = neighSolution;

            tabuList.add(currentSolution.skuComeSeq.toString());
            if (neighSolution.objVal < solution.objVal) {
                solution = neighSolution;
            }

            if (tabuList.size() > 100) {
                // Remove the oldest entry from the tabu list if it exceeds the size
                tabuList.remove(0);
            }
        }
        // 输出最优值
        return solution;
    }

    public Solution findBestNeighbor(List<Integer> skuComeSeq, int neigh, int neighborSize) throws IOException, GRBException, CloneNotSupportedException {
        // 声明变量
        List<Integer> neighSKUComeSeq;
        Solution neighSolution;
        Solution bestNeighSolution = null;
        int[] combi;
        Set<Integer> combiSet;
        if(neigh != 4) {
            combiSet = operator.randomSetGeneration(neighborSize, this.combinationNumSKU);
        }else{
            combiSet = new HashSet<>();
            for(int i = 1;i<skuComeSeq.size();i++){
                combiSet.add(i);
            }
        }
        List<Integer> neighSKUComeSeqFinal;
        if(neigh==1) {
            for (int idx : combiSet) {
                combi = this.skuCombination[idx];
                neighSKUComeSeq = this.operator.twoOptList(combi[0], combi[1], skuComeSeq);
                if(this.tabuList.contains(neighSKUComeSeq.toString())){
                    continue;
                }
                neighSKUComeSeqFinal = this.beamSearch.beamSearch(neighSKUComeSeq);
                if(neighSKUComeSeqFinal!=null){
                    neighSolution = new Solution();
                    neighSolution.skuComeSeq = neighSKUComeSeqFinal;
                    this.toteSelection.cal(neighSolution);
                    if (bestNeighSolution == null) {
                        bestNeighSolution = neighSolution;
                    } else if (bestNeighSolution.objVal > neighSolution.objVal) {
                        bestNeighSolution = neighSolution;
                    }
                }
            }
        } else if (neigh ==2) {
            for (int idx : combiSet) {
                combi = this.skuCombination[idx];
                neighSKUComeSeq = this.operator.reversePartList(combi[0], combi[1], skuComeSeq);
                if(this.tabuList.contains(neighSKUComeSeq.toString())){
                    continue;
                }
                neighSKUComeSeqFinal = this.beamSearch.beamSearch(neighSKUComeSeq);
                if(neighSKUComeSeqFinal!=null){
                    neighSolution = new Solution();
                    neighSolution.skuComeSeq = neighSKUComeSeqFinal;
                    this.toteSelection.cal(neighSolution);
//                    System.out.println(neighSolution.objVal);
                    if (bestNeighSolution == null) {
                        bestNeighSolution = neighSolution;
                    } else if (bestNeighSolution.objVal > neighSolution.objVal) {
                        bestNeighSolution = neighSolution;
                    }
                }
            }
        } else if (neigh == 3) {
            for (int idx : combiSet) {
                combi = this.skuCombination[idx];
                neighSKUComeSeq = this.operator.twoForwardList(combi[0], combi[1], skuComeSeq);
                if(this.tabuList.contains(neighSKUComeSeq.toString())){
                    continue;
                }
                neighSKUComeSeqFinal = this.beamSearch.beamSearch(neighSKUComeSeq);
                if(neighSKUComeSeqFinal!=null){
                    neighSolution = new Solution();
                    neighSolution.skuComeSeq = neighSKUComeSeqFinal;
                    this.toteSelection.cal(neighSolution);
//                    System.out.println(neighSolution.objVal);
                    if (bestNeighSolution == null) {
                        bestNeighSolution = neighSolution;
                    } else if (bestNeighSolution.objVal > neighSolution.objVal) {
                        bestNeighSolution = neighSolution;
                    }
                }
            }
        }else{
            for (int idx : combiSet) {
                neighSKUComeSeq = this.operator.partForwardList(idx, skuComeSeq);
                if(this.tabuList.contains(neighSKUComeSeq.toString())){
                    continue;
                }
                neighSKUComeSeqFinal = this.beamSearch.beamSearch(neighSKUComeSeq);
                if(neighSKUComeSeqFinal!=null){
                    neighSolution = new Solution();
                    neighSolution.skuComeSeq = neighSKUComeSeqFinal;
                    this.toteSelection.cal(neighSolution);
//                    System.out.println(neighSolution.objVal);
                    if (bestNeighSolution == null) {
                        bestNeighSolution = neighSolution;
                    } else if (bestNeighSolution.objVal > neighSolution.objVal) {
                        bestNeighSolution = neighSolution;
                    }
                }
            }
        }
        return bestNeighSolution;
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

    public int[] getOrderSeq(List<Integer> skuComeSeq){
        int[] orderSeq = new int[instance.orderNum];
        int orderIdx = 0;
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
                orderSeq[orderIdx] = nextOrder;
                orderIdx++;
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
                return orderSeq;
            }
            most_unneeded_sku = getMostUnneededSKU(remainingOrders, activeSKU, skuComeSeq.subList(nextSKUIndex, skuComeSeq.size()));
            activeSKU.remove(most_unneeded_sku);
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
                return orderSeq;
            }
        }
        return  orderSeq;
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
        Set<Integer> ordersInConsideration = new HashSet<>(ordersInConsiderationActive);

        for(int sku:newSKUComeSeq){
            ordersInConsiderationFuture.addAll(instance.orderSetBySKU[sku]);
        }
        ordersInConsiderationFuture.retainAll(remainingOrders);

        // 两个都要考虑，求交集
        ordersInConsideration.retainAll(ordersInConsiderationFuture);
        // 只考虑active
        ordersInConsiderationActive.removeAll(ordersInConsideration);
        // 只考虑未来
        ordersInConsiderationFuture.removeAll(ordersInConsideration);

        if(ordersInConsideration.size() == 0 && ordersInConsiderationActive.size()==0 && ordersInConsiderationFuture.size() ==0){
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
            if(curSim>bestSim){
                bestSim = curSim;
                bestO = o;
            }
        }

        for(int o:ordersInConsideration){
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
            intersection.clear();
            intersection.addAll(order);
            intersection.retainAll(newSKUComeSeq);
            intersectionSize = intersection.size();
            differenceSize = order.size() - intersection.size();
            curSim += 0.3*intersectionSize/Math.sqrt(intersectionSize*intersectionSize+differenceSize*differenceSize);
            if(curSim>bestSim){
                bestSim = curSim;
                bestO = o;
            }
        }
        for(int o:ordersInConsiderationFuture){
            order = this.instance.skuSetByOrder[o];
            intersection.clear();
            intersection.addAll(order);
            intersection.retainAll(newSKUComeSeq);
            intersectionSize = intersection.size();
            differenceSize = order.size() - intersection.size();
            curSim = 0.3*intersectionSize/Math.sqrt(intersectionSize*intersectionSize+differenceSize*differenceSize);
            if(curSim>bestSim){
                bestSim = curSim;
                bestO = o;
            }
        }
        return bestO;
    }


    public int getMostUnneededSKU(Set<Integer> remainingOrders,  Set<Integer> activeSKU, List<Integer> newSKUComeSeq) {
        int most_unneeded_sku=-1;
        int most_unneeded_score = 100000;
        Map<Integer,Integer> skuDemandScore = new HashMap<>();
        Set<Integer> unFinishedOrder;

        for(int sku:activeSKU){
            unFinishedOrder = new HashSet<>(instance.orderSetBySKU[sku]);
            unFinishedOrder.retainAll(remainingOrders);
            if(newSKUComeSeq.contains(sku)){
                skuDemandScore.put(sku, unFinishedOrder.size() - 1000);
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
        return most_unneeded_sku;
    }

}
