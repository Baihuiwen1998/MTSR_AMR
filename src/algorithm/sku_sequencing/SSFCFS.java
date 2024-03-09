package algorithm.sku_sequencing;

import algorithm.AlgorithmParams;
import instance_generation.Instance;

import java.util.*;
// 根据订单的需求情况先到先得
public class SSFCFS implements SKUScheduling{
    int[] orderSeq;
    HashMap<Integer, Set<Integer>> pickingOrders;
    List<Integer> activeSKUs;
    Instance instance;
    int nextOrderSeq;
    int nextOrder;
    int[] skuUsedNum;
    HashMap<Integer, Integer> pickingOrderSeq;
    @Override
    public void init(Instance instance){
        this.instance = instance;
    }
    @Override
    public List<Integer> getSKUSchedule(int[] orderSeq) {
        this.orderSeq = orderSeq;
        this.pickingOrderSeq = new HashMap<>();
        List<Integer> skuComeSeq = new ArrayList<>();
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
            pickingOrderSeq.put(nextOrder, i);
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
                    activeSKUs.remove(0);
                }else{
                    //将该SKU移出拣选站
                    activeSKUs.remove(Integer.valueOf(most_unneeded_sku));
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
            keyList.clear();
            keyList.addAll(pickingOrders.keySet());
            for (int key : keyList) {
                pickingOrders.get(key).remove(most_needed_sku);//对订单进行拣选
                if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
                    pickingOrders.remove(key);//删除该订单
                    pickingOrderSeq.remove(key);
                    ordernotadded = true;
                    while (ordernotadded && (nextOrderSeq!=orderSeq.length) ){//判断是否需要添加新的订单，或者是否仍有新订单可以添加
                        nextOrder = orderSeq[nextOrderSeq];
                        curOrderDemand = new HashSet<>(instance.skuSetByOrder[nextOrder]);
                        activeSKUs.forEach(curOrderDemand::remove);
                        if (!curOrderDemand.isEmpty()) {
                            // 如果该新订单可以直接被当前服务的SKU完成，则不需要进行进入pickingOrders的操作
                            pickingOrders.put(nextOrder, curOrderDemand);
                            pickingOrderSeq.put(nextOrder, nextOrderSeq);
                            ordernotadded = false;
                        }
                        nextOrderSeq++;
                    }
                }
            }
        }
        return skuComeSeq;
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
                skuScore.put(sku,skuScore.getOrDefault(sku,0)+2);
            }
        }
        // 判断是否还有订单没有被打开
        for(int sku:activeSKUs){
            if(skuScore.containsKey(sku)){
                if(skuScore.get(sku)<countMin){
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
        int firstComeOrder = -1;
        int firstComeOrderIdx = instance.orderNum;
        for(int o: pickingOrderSeq.keySet()){
            if(pickingOrderSeq.get(o)<firstComeOrderIdx){
                firstComeOrder = o;
                firstComeOrderIdx = pickingOrderSeq.get(o);
            }
        }
        HashMap<Integer,Integer> skuScore = new HashMap<>();

        for (int sku : pickingOrders.get(firstComeOrder)) {
            skuScore.put(sku, skuScore.getOrDefault(sku, 0) + 10);
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
