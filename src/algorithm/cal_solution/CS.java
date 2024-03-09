package algorithm.cal_solution;

import algorithm.AlgorithmParams;
import instance_generation.Instance;

import java.util.*;

public class CS {
    int[] orderSeq;
    HashMap<Integer, Set<Integer>> pickingOrders;
    List<Integer> activeSKUs;
    Instance instance;
    int nextOrderSeq;
    int nextOrder;
    int[] skuUsedNum;

    public void init(Instance instance){
        this.instance = instance;
    }

    public StartSolution calStartSolution(int[] orderSeq) {
        this.orderSeq = orderSeq;
        List<Integer> skuComeSeq = new ArrayList<>();
        // Start Solution
        StartSolution ss= new StartSolution();
        List<Integer> skuLeaveSeq = new ArrayList<>();
        int[][] pickRecord = new int[this.instance.orderNum][this.instance.skuNum];
        int[][] orderOpenIntervals = new int[this.instance.orderNum][2];
        int[] comeKOfSKU = new int[instance.skuNum];

        Arrays.fill(comeKOfSKU, 1000);
        // 辅助性参数
        int nextSKUidx = 0;
        skuUsedNum = new int[instance.skuNum];
        Arrays.fill(skuUsedNum, 0);
        pickingOrders = new HashMap<>();
        activeSKUs = new ArrayList<>();
        //初始化最初拣选的订单
        HashSet<Integer> curOrderDemand;
        for (int i = 0; i < instance.orderBinCap; i++) {
            nextOrder = orderSeq[i];
            curOrderDemand = new HashSet<>(instance.skuSetByOrder[nextOrder]);
            pickingOrders.put(nextOrder, curOrderDemand);
            orderOpenIntervals[nextOrder][0] = 0;
        }
        nextOrderSeq=instance.orderBinCap;
        /* 寻找下一个到达的SKU和离开的SKU */
        // 选出当前拣选订单和未来要拣选的k个订单中选择需求最大的SKU作为第一个服务的SKU
        int most_needed_sku;
        int most_unneeded_sku;
        boolean ordernotadded;
        ArrayList<Integer> keyList = new ArrayList<>();
        while (!pickingOrders.isEmpty()) {
            if (activeSKUs.size() == instance.toteCapByStation) {
                //选出未来几个订单中最不需要的SKU信息
                most_unneeded_sku = getMostUnneededSku();
                if(most_unneeded_sku==-1){
                    int removedSKU = activeSKUs.remove(0);
                    skuLeaveSeq.add(removedSKU);
                }else{
                    //将该SKU移出拣选站
                    activeSKUs.remove(Integer.valueOf(most_unneeded_sku));
                    skuLeaveSeq.add(most_unneeded_sku);
                }
            }
            //选出拣选站最需要的SKU信息
            most_needed_sku = getMostNeededSKU();
            if(most_needed_sku == -1){
                return null;
            }
            skuUsedNum[most_needed_sku]++;
            skuComeSeq.add(most_needed_sku);
            activeSKUs.add(most_needed_sku);
            nextSKUidx++;
            keyList.clear();
            keyList.addAll(pickingOrders.keySet());
            for (int key : keyList) {
                pickingOrders.get(key).remove(most_needed_sku);//对订单进行拣选
                pickRecord[key][most_needed_sku] = nextSKUidx-1;
                if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                    pickingOrders.remove(key);//删除该订单
                    orderOpenIntervals[key][1] = nextSKUidx-1;
                    ordernotadded = true;
                    while (ordernotadded && (nextOrderSeq!=orderSeq.length) ){//判断是否需要添加新的订单，或者是否仍有新订单可以添加
                        nextOrder = orderSeq[nextOrderSeq];
                        orderOpenIntervals[nextOrder][0] = nextSKUidx-1;
                        curOrderDemand = new HashSet<>(instance.skuSetByOrder[nextOrder]);
                        activeSKUs.forEach(curOrderDemand::remove);
                        for(int s:activeSKUs){
                            pickRecord[nextOrder][s] = nextSKUidx-1;
                        }
                        if (!curOrderDemand.isEmpty()) {
                            // 如果该新订单可以直接被当前服务的SKU完成，则不需要进行进入pickingOrders的操作
                            pickingOrders.put(nextOrder, curOrderDemand);
                            ordernotadded = false;
                        }else{
                            orderOpenIntervals[nextOrder][1] = nextSKUidx-1;
                        }
                        nextOrderSeq++;
                    }
                }
            }
        }
        ss.orderOpenIntervals = orderOpenIntervals;
        ss.skuLeaveSeq = skuLeaveSeq;
        ss.pickRecord = pickRecord;
        ss.skuComeSeq = skuComeSeq;
        ss.orderSeq = orderSeq;
        return ss;
    }

    //根据当前拣选站正在拣选的订单信息，求当前拣选站台上最不需要的SKU种类，移出拣选站
    public int getMostUnneededSku() {
        int most_unneeded_sku=-1;
        int countMin = Integer.MAX_VALUE;
        HashMap<Integer,Integer> skuScore = new HashMap<>();
        Set<Integer> skuByOrder = new HashSet<>();
        int order;
        for(int i = nextOrderSeq; i<Math.min(nextOrderSeq + AlgorithmParams.forwardOrderNum,instance.orderNum); i++){
            order = orderSeq[i];
            skuByOrder.clear();
            skuByOrder.addAll(instance.skuSetByOrder[order]);
            skuByOrder.retainAll(activeSKUs);
            for(int sku:skuByOrder){
                skuScore.put(sku,skuScore.getOrDefault(sku,0)+1);
            }
        }
        for(int sku:activeSKUs){
            if(skuScore.containsKey(sku)){
                if(skuScore.get(sku)<countMin&& skuUsedNum[sku]<instance.toteNumBySKU[sku]){
                    countMin = skuScore.get(sku);
                    most_unneeded_sku = sku;
                }
            }else{
                return sku;
            }
        }
        return most_unneeded_sku;
    }
    // 已知正在拣选的订单所需SKU信息，求需求度最高的SKU
    public int getMostNeededSKU() {
        int most_needed_sku = -1;
        int countMax = -1;
        HashMap<Integer,Integer> skuScore = new HashMap<>();
        Set<Integer> skuByOrder;
        for(int key:pickingOrders.keySet()) {
            for (int sku : pickingOrders.get(key)) {
                skuScore.put(sku, skuScore.getOrDefault(sku, 0) + 10);
            }
        }
        for(int i = nextOrderSeq; i<Math.min(nextOrderSeq + AlgorithmParams.forwardOrderNum,instance.orderNum); i++) {
            skuByOrder = instance.skuSetByOrder[i];
            for (int sku : skuByOrder) {
                skuScore.put(sku, skuScore.getOrDefault(sku, 0) + 1);
            }
        }
        for(int key:skuScore.keySet()) {
            if(skuScore.get(key)>countMax && skuUsedNum[key]<instance.toteNumBySKU[key]){
                most_needed_sku =  key;
                countMax = skuScore.get(key);
            }
        }
        return most_needed_sku;
    }
}
