package algorithm.order_sequencing;

import java.util.*;

public class State implements  Cloneable{
    public HashMap<Integer, Set<Integer>> pickingOrders;
    public HashSet<Integer> skuRemain;
    public Set<Integer> remainingOrders;
    public Set<Integer> activeSKU;
    public List<Integer> orderComeSeq;
    public int nextSKUIndex;
    public double fitness;
    HashSet<Integer> curOrderDemand;
    public List<Integer> skuComeSeq;

    public State(int toteCapByStation){
        this.pickingOrders = new HashMap<>();
        this.remainingOrders = new HashSet<>();
        this.activeSKU = new HashSet<>();
        this.curOrderDemand = new HashSet<>();
        this.skuRemain = new HashSet<>();
        this.orderComeSeq = new ArrayList<>();
        this.nextSKUIndex = toteCapByStation;
        this.fitness = Double.MAX_VALUE;
        this.skuComeSeq = new ArrayList<>();
    }

    public void updateFitness(Set<Integer>[] skuByOrder){
        // fitness越大越差
        this.fitness = remainingOrders.size();
        for(int key:pickingOrders.keySet()){
            this.fitness+= (double)pickingOrders.get(key).size() / (double)skuByOrder[key].size();

        }

        // 目前用掉的sku数量越多，越不好
//        this.fitness += nextSKUIndex;
    }


    public Object clone() throws CloneNotSupportedException{
        State clone = new State(nextSKUIndex);
        Iterator<Integer> iter = this.pickingOrders.keySet().iterator();
        while(iter.hasNext()){
            int key = iter.next();
            Set<Integer> skuByO = new HashSet<>();
            skuByO.addAll(this.pickingOrders.get(key));
            clone.pickingOrders.put(key, skuByO);
        }
        clone.skuRemain = new HashSet<>(skuRemain);
        clone.remainingOrders = new HashSet<>(remainingOrders);
        clone.activeSKU = new HashSet<>(activeSKU);
        clone.nextSKUIndex = this.nextSKUIndex;
        clone.fitness = this.fitness;
        clone.curOrderDemand = new HashSet<>(curOrderDemand);
        clone.orderComeSeq = new ArrayList<>(orderComeSeq);
        clone.skuComeSeq = new ArrayList<>(skuComeSeq);
        return clone;
    }

}
