package algorithm.order_sequencing;

import instance_generation.Instance;

import java.util.*;

public class BeamSearch {
    public Instance instance;

    public int beamSize = 100;

    public BeamSearch(Instance instance) {
        this.instance = instance;
    }

    public int beamSearchTest(List<Integer> skuComeSeq) throws CloneNotSupportedException {
        List<State> stateList = new ArrayList<>();
        List<State> tempStateList;
        List<Integer> iterList =new ArrayList<>();
        State initState = new State(instance.toteCapByStation);
        for(int i = 0;i<this.instance.orderNum;i++){
            initState.remainingOrders.add(i);
        }
        initState.nextSKUIndex = this.instance.toteCapByStation;
        initState.activeSKU = new HashSet<>(skuComeSeq.subList(0, this.instance.toteCapByStation));
        int nextOrder;

        State state = initState;
        int most_unneeded_sku;
        List<Integer> orderSeq = List.of(new Integer[]{3,6,21,22,14,20,11,13,8,5,1,15,7,23,9,2,16,19,18,10,17,0,24,12,4});
        for (int o : orderSeq) {
            boolean flag = state.remainingOrders.size() > 0 || state.skuRemain.size() > 0;
            pickOrders(state, o, skuComeSeq);
            if (state.pickingOrders.size() < this.instance.orderBinCap && state.remainingOrders.size() > 0) {
                // 需要添加下一个Order，直接加入stateList
                state.updateFitness(instance.skuSetByOrder);
            } else {
                // 不满足上述条件，需要进入sku的移除过程
                while (flag) {
                    if (state.pickingOrders.size() == 0 && state.remainingOrders.size() == 0) {
                        return state.nextSKUIndex;
                    }
                    most_unneeded_sku = getMostUnneededSKU(state.remainingOrders, state.activeSKU, skuComeSeq.subList(state.nextSKUIndex, skuComeSeq.size()));
                    if (most_unneeded_sku == -1) {
                        // 不可行，该state作废

                        return -1;
                    }
                    state.activeSKU.remove(most_unneeded_sku);
                    if (state.nextSKUIndex == skuComeSeq.size()) {
                        return -1;
                    }
                    state.activeSKU.add(skuComeSeq.get(state.nextSKUIndex));
                    iterList.clear();
                    iterList.addAll(state.pickingOrders.keySet());
                    //针对当前pickingOrders里面的订单拣选
                    for (int key : iterList) {
                        state.pickingOrders.get(key).remove(skuComeSeq.get(state.nextSKUIndex));//对订单进行拣选
                        state.skuRemain.remove(skuComeSeq.get(state.nextSKUIndex));
                        if (state.pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                            state.pickingOrders.remove(key);//删除该订单
                        }
                    }
                    state.nextSKUIndex++;
                    if (state.pickingOrders.size() == 0 && state.remainingOrders.size() == 0) {
                        // 直接输出有可行解
                        return state.nextSKUIndex;
                    }
                    // 仍有订单没有拣选or仍有在拣选订单未能拣选完毕
                    flag = !(state.pickingOrders.size() < this.instance.orderBinCap && state.remainingOrders.size() > 0);
                    // 检查是否remainingOrder内仍有订单要加入 or pickingOrder内仍有加入订单的空间，如果没有就要继续通过移除sku的形式进行拣选
                }
            }
        }
        return -1;
    }

    public List<Integer> beamSearch(List<Integer> skuComeSeq) throws CloneNotSupportedException {
        List<State> stateList = new ArrayList<>();
        List<State> tempStateList;
        List<Integer> iterList =new ArrayList<>();
        State initState = new State(instance.toteCapByStation);
        for(int i = 0;i<this.instance.orderNum;i++){
            initState.remainingOrders.add(i);
        }
        initState.nextSKUIndex = this.instance.toteCapByStation;
        initState.activeSKU = new HashSet<>(skuComeSeq.subList(0, this.instance.toteCapByStation));
        initState.skuComeSeq.addAll(skuComeSeq.subList(0, this.instance.toteCapByStation));
        stateList.add(initState);
        Set<Integer> nextOrderSet;
        tempStateList = new ArrayList<>();
        State tempState;
        int most_unneeded_sku;
        while(stateList.size()>0) {
            tempStateList.clear();
            for (State state : stateList) {
                // 对于每一个状态而言，第一次都是进行nextOrder的选择
                nextOrderSet = calNextOrderSet(state.remainingOrders, state.activeSKU);
                loop:
                for (int o : nextOrderSet) {
                    tempState = (State) state.clone();
                    boolean flag = tempState.remainingOrders.size() > 0 || tempState.skuRemain.size() > 0;
                    pickOrders(tempState, o, skuComeSeq);
                    if (tempState.pickingOrders.size() < this.instance.orderBinCap && tempState.remainingOrders.size() > 0) {
                        // 需要添加下一个Order，直接加入tempStateList
                        tempState.updateFitness(instance.skuSetByOrder);
                        tempStateList.add(tempState);
                    } else {
                        // 不满足上述条件，需要进入sku的移除过程
                        while (flag) {
                            if (tempState.pickingOrders.size() == 0 && tempState.remainingOrders.size() == 0) {
                                return tempState.skuComeSeq;
                            }
                            most_unneeded_sku = getMostUnneededSKU(tempState.remainingOrders, tempState.activeSKU, skuComeSeq.subList(tempState.nextSKUIndex, skuComeSeq.size()));
                            if (most_unneeded_sku == -1) {
                                // 不可行，该state作废
                                continue loop;
                            }
                            tempState.activeSKU.remove(most_unneeded_sku);
                            if (tempState.nextSKUIndex == skuComeSeq.size()) {
                                continue loop;
                            }

                            if(!tempState.activeSKU.contains(skuComeSeq.get(tempState.nextSKUIndex))){
                                tempState.skuComeSeq.add(skuComeSeq.get(tempState.nextSKUIndex));
                            }
                            tempState.activeSKU.add(skuComeSeq.get(tempState.nextSKUIndex));
                            iterList.clear();
                            iterList.addAll(tempState.pickingOrders.keySet());
                            //针对当前pickingOrders里面的订单拣选
                            for (int key : iterList) {
                                tempState.pickingOrders.get(key).remove(skuComeSeq.get(tempState.nextSKUIndex));//对订单进行拣选
                                tempState.skuRemain.remove(skuComeSeq.get(tempState.nextSKUIndex));
                                if (tempState.pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                                    tempState.pickingOrders.remove(key);//删除该订单
                                }
                            }
                            tempState.nextSKUIndex++;
                            if (tempState.pickingOrders.size() == 0 && tempState.remainingOrders.size() == 0) {
                                // 直接输出有可行解
                                return tempState.skuComeSeq;
                            }
                            // 仍有订单没有拣选or仍有在拣选订单未能拣选完毕
                            flag = !(tempState.pickingOrders.size() < this.instance.orderBinCap && tempState.remainingOrders.size() > 0);
                            // 检查是否remainingOrder内仍有订单要加入 or pickingOrder内仍有加入订单的空间，如果没有就要继续通过移除sku的形式进行拣选
                        }
                        // 跳出flag，说明需要添加新的订单
                        tempState.updateFitness(instance.skuSetByOrder);
                        tempStateList.add(tempState);
                    }
                }
            }
            if (tempStateList.size() == 0) {
                return null;
            } else {
                // 对tempStateList排序，选出前BeamSize个结果
                Collections.sort(tempStateList, new Comparator<State>() {
                    @Override
                    public int compare(State o1, State o2) {
                        return (int)((o1.fitness - o2.fitness)*10000);
                    }
                });
                stateList.clear();
                stateList.addAll(tempStateList.subList(0, Math.min(beamSize, tempStateList.size())));
            }
        }
        return null;
    }
    public Set<Integer> calNextOrderSet(Set<Integer> remainingOrders, Set<Integer> activeSKU){
        /*
        获取待选订单/
         */
        Set<Integer> orderSet = new HashSet<>();
        double curSim;
        Set<Integer> intersection = new HashSet<>();
        int intersectionSize;
        int differenceSize;
        Set<Integer> ordersInConsiderationActive = new HashSet<>();
        Set<Integer> order;
        for(int sku:activeSKU){
            ordersInConsiderationActive.addAll(instance.orderSetBySKU[sku]);
        }
        ordersInConsiderationActive.retainAll(remainingOrders);
        int[][] simRecord = new int[ordersInConsiderationActive.size()][2];
        int idx = 0;
        for(int o:ordersInConsiderationActive){
            order = this.instance.skuSetByOrder[o];
            intersection.clear();
            intersection.addAll(order);
            intersection.retainAll(activeSKU);
            intersectionSize = intersection.size();
            differenceSize = order.size() - intersectionSize;
            curSim = intersectionSize+ intersectionSize/Math.sqrt(intersectionSize*intersectionSize+differenceSize*differenceSize);
            simRecord[idx][0] = o;
            simRecord[idx][1] = (int)(curSim*1000);
            idx++;
        }
        Arrays.sort(simRecord, new Comparator<int[]>() {
            @Override
            public int compare(int[] o1, int[] o2) {
                return -o1[1]+o2[1];
            }
        });
        // 获取前5个
        for(int i = 0;i<Math.min(3, ordersInConsiderationActive.size());i++){
            orderSet.add(simRecord[i][0]);
        }
        return orderSet;
    }

    public void pickOrders(State state, int nextOrder, List<Integer> skuComeSeq){
        ArrayList<Integer> iterList = new ArrayList<>();
        state.remainingOrders.remove(nextOrder);
        state.orderComeSeq.add(nextOrder);
        state.curOrderDemand = new HashSet<>(instance.skuSetByOrder[nextOrder]);
        state.curOrderDemand.removeAll(state.activeSKU);
        if(state.curOrderDemand.size()>0) {
            state.skuRemain.addAll(state.curOrderDemand);
            state.pickingOrders.put(nextOrder, state.curOrderDemand);
        }
        Set<Integer> removeSKUSet = new HashSet<>();
        // 检查是否有sku完成了所有要拣选的订单任务，可以直接从activeSKU中删除
        for (int sku : state.activeSKU) {
            if (checkIfSkuUseless(sku, state.remainingOrders)) {
                removeSKUSet.add(sku);
            }
        }
        // 从activeSKU中删除
        int removeSKUNum = removeSKUSet.size();
        if (removeSKUNum > 0) {
            state.activeSKU.removeAll(removeSKUSet);
            while(state.activeSKU.size()<instance.toteCapByStation && state.nextSKUIndex != skuComeSeq.size()){
                if(!state.activeSKU.contains(skuComeSeq.get(state.nextSKUIndex))){
                    state.skuComeSeq.add(skuComeSeq.get(state.nextSKUIndex));
                }
                state.activeSKU.add(skuComeSeq.get(state.nextSKUIndex));
                state.nextSKUIndex++;
            }
            // 对所有pickingOrder进行拣选
            iterList.clear();
            iterList.addAll(state.pickingOrders.keySet());
            //针对当前pickingOrders里面的订单拣选
            for (int key : iterList) {
                state.pickingOrders.get(key).removeAll(state.activeSKU);//对订单进行拣选
                state.skuRemain.removeAll(state.activeSKU);
                if (state.pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                    state.pickingOrders.remove(key);//删除该订单
                }
            }
        }
    }

    public int checkIfFeasibleByBS(List<Integer> skuComeSeq){
        /*
        采用束搜索寻找可行的订单顺序/
         */
        HashMap<Integer, Set<Integer>> pickingOrders = new HashMap<>();
        HashSet<Integer> skuRemain =new HashSet<>(); // 表示目前拣选台上面需要的sku
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

    public boolean checkIfSkuUseless(int sku, Set<Integer> remainingOrders){
        Set<Integer> unFinishedOrder = new HashSet<>(instance.orderSetBySKU[sku]);
        unFinishedOrder.retainAll(remainingOrders); // 无需考虑pickingOrder，cuz该订单必然已经拣选了activeSKU集合中的sku
        if(unFinishedOrder.size()==0){
            return true;
        }else{
            return false;
        }
    }
}
