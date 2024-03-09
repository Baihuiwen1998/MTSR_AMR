package algorithm.cal_solution;

import algorithm.AlgorithmParams;
import algorithm.routing.Combined;
import algorithm.routing.Routing;
import algorithm.sku_sequencing.SKUScheduling;
import algorithm.sku_sequencing.SSGreedy;
import algorithm.sku_sequencing.SSMIP;
import algorithm.tote_selection.TSAlg;
import algorithm.tote_selection.TSMIP;
import algorithm.tote_selection.ToteSelection;
import instance_generation.Instance;

import java.util.*;

public class CSImpr {

    static Instance instance;
    SKUScheduling skuScheduling;
    Routing routing;
    ToteSelection toteSelection;


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

    public StartSolution calStartSolution(List<Integer> skuComeSeq){
        StartSolution ss = new StartSolution();
        int[][] pickRecord = new int[this.instance.orderNum][this.instance.skuNum];
        int[][] orderOpenIntervals = new int[this.instance.orderNum][2];
        int[] orderSeq = new int[this.instance.orderNum];
        int orderIdx = 0;
        List<Integer> skuLeaveSeq = new ArrayList<>();
        int[] comeKOfSKU = new int[instance.skuNum];


        HashMap<Integer, Set<Integer>> pickingOrders = new HashMap<>();
        HashSet<Integer> skuRemain =new HashSet<>();
        Set<Integer> remainingOrders = new HashSet<>();
        for(int i = 0;i<this.instance.orderNum;i++){
            remainingOrders.add(i);
            Arrays.fill(pickRecord[i], 1000);
        }
        int nextSKUIndex = this.instance.toteCapByStation;
        Set<Integer> activeSKU = new HashSet<>(skuComeSeq.subList(0, this.instance.toteCapByStation));
        HashSet<Integer> curOrderDemand;
        int most_unneeded_sku;
        int nextOrder;
        ArrayList<Integer> iterList = new ArrayList<>();
        Set<Integer> removeSKUSet;
        while(remainingOrders.size()>0 || skuRemain.size()>0){
            //订单拣选
            while(pickingOrders.size()<this.instance.orderBinCap && remainingOrders.size()>0){
                nextOrder = calBestSimilarOrder(skuComeSeq.subList(nextSKUIndex,Math.min(nextSKUIndex+2,skuComeSeq.size())),remainingOrders,activeSKU);
                orderOpenIntervals[nextOrder][0] = nextSKUIndex-1;
                remainingOrders.remove(nextOrder);
                curOrderDemand = new HashSet<>(instance.skuSetByOrder[nextOrder]);
                orderSeq[orderIdx] = nextOrder;
                orderIdx++;
                curOrderDemand.removeAll(activeSKU);
                for(int s:activeSKU){
                    pickRecord[nextOrder][s] = Math.min(pickRecord[nextOrder][s], nextSKUIndex-1);
                }
                if(curOrderDemand.size()>0) {
                    skuRemain.addAll(curOrderDemand);
                    pickingOrders.put(nextOrder, curOrderDemand);
                }else{
                    orderOpenIntervals[nextOrder][1] = nextSKUIndex-1;
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
                        nextSKUIndex++;
                    }
                    // 对所有pickingOrder进行拣选
                    iterList.clear();
                    iterList.addAll(pickingOrders.keySet());
                    //针对当前pickingOrders里面的订单拣选
                    for (int key : iterList) {
                        pickingOrders.get(key).removeAll(activeSKU);//对订单进行拣选
                        for(int s:activeSKU){
                            pickRecord[key][s] = Math.min(pickRecord[key][s], nextSKUIndex-1);
                        }
                        skuRemain.removeAll(activeSKU);
                        if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                            pickingOrders.remove(key);//删除该订单
                            orderOpenIntervals[key][1] = nextSKUIndex-1;
                        }
                    }
                }
            }
            if(pickingOrders.size()==0&&remainingOrders.size()==0){
                ss.orderOpenIntervals = orderOpenIntervals;
                ss.pickRecord = pickRecord;
                ss.skuLeaveSeq = skuLeaveSeq;
                ss.orderSeq = orderSeq;
                return ss;
            }
            most_unneeded_sku = getMostUnneededSKU(remainingOrders, activeSKU, skuComeSeq.subList(nextSKUIndex, skuComeSeq.size()));
            if(most_unneeded_sku==-1){
                return null;
            }
            activeSKU.remove(most_unneeded_sku);
            skuLeaveSeq.add(most_unneeded_sku);
            if(nextSKUIndex==skuComeSeq.size()){
                ss.orderOpenIntervals = orderOpenIntervals;
                ss.pickRecord = pickRecord;
                ss.skuLeaveSeq = skuLeaveSeq;
                ss.orderSeq = orderSeq;
                return ss;
            }
            int nextSKU = skuComeSeq.get(nextSKUIndex);
            if(activeSKU.contains(nextSKU)){
                skuLeaveSeq.add(nextSKU);
            }
            activeSKU.add(nextSKU);

            iterList.clear();
            iterList.addAll(pickingOrders.keySet());
            //针对当前pickingOrders里面的订单拣选
            for (int key : iterList) {
                nextSKU = skuComeSeq.get(nextSKUIndex);
                pickingOrders.get(key).remove(nextSKU);//对订单进行拣选
                pickRecord[key][nextSKU] = Math.min(pickRecord[key][nextSKU], nextSKUIndex);
                skuRemain.remove(skuComeSeq.get(nextSKUIndex));
                if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                    orderOpenIntervals[key][1] = nextSKUIndex;
                    pickingOrders.remove(key);//删除该订单
                }
            }
            nextSKUIndex++;
            if(pickingOrders.size()==0&&remainingOrders.size()==0){
                ss.orderOpenIntervals = orderOpenIntervals;
                ss.pickRecord = pickRecord;
                ss.skuLeaveSeq = skuLeaveSeq;
                ss.orderSeq = orderSeq;
                return ss;
            }
        }
        ss.orderOpenIntervals = orderOpenIntervals;
        ss.pickRecord = pickRecord;
        ss.skuLeaveSeq = skuLeaveSeq;
        ss.orderSeq = orderSeq;
        return ss;
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
