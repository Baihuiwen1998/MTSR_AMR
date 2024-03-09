package algorithm;

import algorithm.cal_ftness.CalFitness;
import gurobi.GRBException;
import instance_generation.Instance;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/*
创建一个初始解/
采用贪婪算法，每次都选取和前一个订单相似度最高的订单作为下一个订单
 */

public class InitiateSolution {
    double[][] similarity;
    Set<Integer> remainingOrders;

    public Solution genInitialSolution(Instance instance, CalFitness cf) throws IOException, GRBException {
        Solution initialSolution = new Solution();
        initialSolution.objVal =10000000000.0;
        int firstOrderTry = 0;
        List<Integer> orderWithOverOneSKU = new ArrayList<>();
        int orderWithOneSKUNum = instance.orderWithOneSKU.size();
        for(int i=0;i<instance.orderNum; i++){
            if(!instance.orderWithOneSKU.contains(i)){
                orderWithOverOneSKU.add(i);
            }
        }
        Solution bestInitialSolution = new Solution();
        bestInitialSolution.objVal = 10000000000.0;
        while(firstOrderTry<instance.orderNum) {
            int[] orderSeq = new int[instance.orderNum];
            Set<Integer> Orders = new HashSet<>();

            for (int i = 0; i < instance.orderNum; i++) {
                Orders.add(i);
            }
            remainingOrders = new HashSet<>();
            remainingOrders.addAll(Orders);
            similarity = instance.orderSimilarity;

            int orderIdx = 0;
            int curOrder;
            //第一个订单选择SKU数量为1的订单,否则随机选择一个订单
            if (instance.orderWithOneSKU.size() > firstOrderTry) {
                curOrder = instance.orderWithOneSKU.get(firstOrderTry);
                firstOrderTry = firstOrderTry+1;
            } else {
                curOrder = orderWithOverOneSKU.get(firstOrderTry-orderWithOneSKUNum);
                firstOrderTry=firstOrderTry+1;
            }
            orderSeq[orderIdx] = curOrder;
            orderIdx++;
            remainingOrders.remove(curOrder);

            //根据订单相似度选择下一个订单
            while (orderIdx < instance.orderNum) {
                curOrder = getMostSimilarOrder(curOrder, instance);
                orderSeq[orderIdx] = curOrder;
                orderIdx++;
            }
            initialSolution = new Solution();
            initialSolution.orderSeq = orderSeq;
            initialSolution = cf.cal(initialSolution);
            if(bestInitialSolution.objVal >= initialSolution.objVal){
                bestInitialSolution = initialSolution;
            }
        }
        return bestInitialSolution;
    }

    //已知当前订单，根据相似度矩阵，选择下一个订单
    public int getMostSimilarOrder(int curOrder, Instance instance){
        double[] sim = similarity[curOrder];
        double maxSimlarity = -Double.MIN_VALUE;
        int nextOrder = -1;
        // 判断是否有sku只有一个tote
        int skuWithOneOrder = -1;
        for(int s:instance.skuSetByOrder[curOrder]){
            if(instance.toteSetBySKU[s].size()==1){
                skuWithOneOrder = s;
                break;
            }
        }
        if(skuWithOneOrder == -1) {
            for (int order : remainingOrders) {
                if (sim[order] > maxSimlarity) {
                    nextOrder = order;
                    maxSimlarity = sim[order];
                }
            }
            remainingOrders.remove(nextOrder);
            return nextOrder;
        }else{
            for (int order : remainingOrders) {
                if(instance.skuSetByOrder[order].contains(skuWithOneOrder)){
                    if(sim[order]+100>maxSimlarity){
                        nextOrder = order;
                        maxSimlarity = sim[order];
                    }
                }else{
                    if(sim[order]>maxSimlarity){
                        nextOrder = order;
                        maxSimlarity = sim[order];
                    }
                }
            }
            remainingOrders.remove(nextOrder);
            return nextOrder;
        }
    }
}
