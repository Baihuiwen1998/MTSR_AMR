package algorithm.order_sequencing;

import algorithm.AlgorithmParams;
import algorithm.Operators;
import algorithm.Solution;
import algorithm.cal_ftness.CalFitness;
import gurobi.GRBException;
import instance_generation.Instance;

import java.io.IOException;
import java.util.Set;

public class VNSBi {
    CalFitness cf;
    Instance instance;
    Operators operator;

    public VNSBi(Instance instance, CalFitness cf){
        this.cf = cf;
        this.instance = instance;
        this.operator = new Operators();
        operator.init(instance);
    }

    public Solution calVNS(Solution initialSolution) throws IOException, GRBException {
        Solution bestSolution = initialSolution;
        int iter = 0;
        Solution shakedSolution, neighSolution, shakedImprSolution;
        int[] shakedOrderSeq;

        double bestObj = bestSolution.objVal;
        while(iter <= AlgorithmParams.maxVNSIteration){
            iter++;
            // shaking
            shakedOrderSeq = operator.shaking(bestSolution.orderSeq, 1);
            shakedSolution = new Solution();
            shakedSolution.orderSeq = shakedOrderSeq;
            shakedSolution = cf.cal(shakedSolution);
            // VND
            int neigh = 1; // 定义邻域的序号
            while(neigh<=AlgorithmParams.maxNeighbourhoodNum){
                neighSolution = findBestNeighbor(shakedOrderSeq, neigh, Math.min(instance.orderNum, AlgorithmParams.neighSearchSize));
                if(neighSolution.objVal < shakedSolution.objVal){
                    shakedSolution = neighSolution;
                    neigh = 1;
                }else{
                    neigh++;
                }
            }
            if(bestObj >= shakedSolution.objVal & shakedSolution.skuComeSeq!=null){
                bestObj = shakedSolution.objVal;
                bestSolution = shakedSolution;
            }
        }

        return bestSolution;
    }

    public Solution findBestNeighbor(int[] orderSeq, int neigh, int neighborSize) throws IOException, GRBException {
        // 声明变量
        int[] neighOrderSeq;
        Solution neighSolution;
        Solution bestNeighSolution = null;
        int[] combi;
        Set<Integer> combiSet;

        if(neigh != 4){
            combiSet = operator.randomSetGeneration(neighborSize, instance.combinationNum);
        }else{
            combiSet = operator.randomSetGeneration(neighborSize, instance.orderNum);
        }
        if(neigh==1) {
            for (int idx : combiSet) {
                combi = instance.orderCombination[idx];
                neighOrderSeq = this.operator.twoOpt(combi[0], combi[1], orderSeq);
                neighSolution = new Solution();
                neighSolution.orderSeq = neighOrderSeq;
                neighSolution = this.cf.cal(neighSolution);
                if (bestNeighSolution == null) {
                    bestNeighSolution = neighSolution;
                }else if (bestNeighSolution.objVal > neighSolution.objVal) {
                    bestNeighSolution = neighSolution;
                }
            }
        } else if (neigh ==2) {
            for (int idx : combiSet) {
                combi = instance.orderCombination[idx];
                neighOrderSeq = this.operator.reversePart(combi[0], combi[1], orderSeq);
                neighSolution = new Solution();
                neighSolution.orderSeq = neighOrderSeq;
                neighSolution = this.cf.cal(neighSolution);
                if (bestNeighSolution == null) {
                    bestNeighSolution = neighSolution;
                }else if (bestNeighSolution.objVal > neighSolution.objVal) {
                    bestNeighSolution = neighSolution;
                }
            }
        } else if (neigh == 3) {
            for (int idx : combiSet) {
                combi = instance.orderCombination[idx];
                neighOrderSeq = this.operator.twoForward(combi[0], combi[1], orderSeq);
                neighSolution = new Solution();
                neighSolution.orderSeq = neighOrderSeq;
                neighSolution = this.cf.cal(neighSolution);
                if (bestNeighSolution == null) {
                    bestNeighSolution = neighSolution;
                }else if (bestNeighSolution.objVal > neighSolution.objVal) {
                    bestNeighSolution = neighSolution;
                }
            }
        }else{
            for (int idx : combiSet) {
                neighOrderSeq = this.operator.partForward(idx, orderSeq);
                neighSolution = new Solution();
                neighSolution.orderSeq = neighOrderSeq;
                neighSolution = this.cf.cal(neighSolution);
                if (bestNeighSolution == null) {
                    bestNeighSolution = neighSolution;
                }else if (bestNeighSolution.objVal > neighSolution.objVal) {
                    bestNeighSolution = neighSolution;
                }
            }
        }

        return bestNeighSolution;
    }

    public boolean metrospolis(double f, double f_new, double T){
        // 模拟退火算法接受准则
        if(f_new<=f){
            System.out.println("f="+f+",f_new="+f_new);
            return true;
        }else{
            double p = Math.exp((f-f_new)/T);
            System.out.println("f="+f+",f_new="+f_new+",probability = "+p);
            if(Math.random()<p){
                return true;
            }else{
                return false;
            }
        }
    }

}
