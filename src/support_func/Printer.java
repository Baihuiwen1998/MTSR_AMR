package support_func;

import gurobi.GRB;
import gurobi.GRBException;
import gurobi.GRBVar;
import instance_generation.Instance;

import java.util.Arrays;
import java.util.Comparator;

public class Printer {
    public static void printSKUComeSeq(int K, int S, GRBVar[][] x) throws GRBException {
        System.out.print("SKU visiting sequence = [");
        for(int k=0;k<K;k++){
            for(int s=0;s<S;s++){
                double xValue = x[s][k].get(GRB.DoubleAttr.X);
                if (xValue > 0) {
                    System.out.print( k+":"+s +", ");
                }
            }
        }
        System.out.println( "]");
    }

    public static void printSKULeaveSeq(int K, int S, GRBVar[][] y) throws GRBException {
        System.out.print("SKU leaving sequence = [");
        for(int k=0;k<K;k++){
            for(int s=0;s<S;s++){
                double yValue = y[s][k].get(GRB.DoubleAttr.X);
                if (yValue > 0) {
                    System.out.print(  k+":"+s +", ");
                }
            }
        }
        System.out.println( "]");
    }

    public static void printOrderOpenSeq(Instance instance, int O, int K, GRBVar[][] alpha) throws GRBException {
        // 订单开始顺序
        int[][] orderStartK = new int[O][2];
        for(int o =0;o<O;o++){
            int min_k = Integer.MAX_VALUE;
            // 输出订单内的sku信息
            System.out.print("SKU Set = [");
            for (int s:instance.skuSetByOrder[o]){
                System.out.print(s+",");
            }
            System.out.println("]");
            // 输出订单的服务时间范围
            System.out.print("Order  "+ o + " is open in k = [");
            for(int k= 0;k<K;k++){
                double alphaValue = alpha[o][k].get(GRB.DoubleAttr.X);
                if (alphaValue > 0) {
                    System.out.print(k +",");
                    min_k = Math.min(min_k, k);
                }
            }
            System.out.println("]");
            orderStartK[o][0] = o;
            orderStartK[o][1] = min_k;
        }
        Arrays.sort(orderStartK, new Comparator<int[]>() {
            @Override
            public int compare(int[] o1, int[] o2) {
                return o1[1]-o2[1];
            }
        });
        System.out.print("orderSeq = [");
        for(int idx = 0;idx<O;idx++){
            System.out.print(orderStartK[idx][0]+",");
        }
        System.out.println("]");
    }

    public static void printOrderCloseSeq(int O, int K, GRBVar[][] beta) throws GRBException {
        // 订单关闭顺序
        for(int o =0;o<O;o++){
            for(int k=0;k<K;k++){
                double betaValue = beta[o][k].get(GRB.DoubleAttr.X);
                if (betaValue > 0) {
                    System.out.println("order "+ o +" is closed in the " +k +"th turn");
                }
            }
        }
    }

    public static void printRobotRoute(Instance instance, int N, int R, GRBVar[][][] z, GRBVar[] mu) throws GRBException {
        // 机器人路径
        for (int r = 0;r < R;r++){
            System.out.println("route r="+r);
            for (int i = 0; i < N+1; i++) {
                for (int j = 0; j < N+1; j++) {
                    double zValue = z[i][j][r].get(GRB.DoubleAttr.X);
                    if (zValue > 0) {
                        System.out.println("robot visits from " + i + " to " + j + " with distance "+instance.disBetweenTotes[i][j]+", zValue="+zValue);
                        if(i<N){
                            double muValue = mu[i].get(GRB.DoubleAttr.X);
                            if (muValue > 0) {
                                System.out.println(", robot visits " + i +  " in route " + r + " in the " + muValue +"th turn");
                            }
                        }
                    }
                }
            }
            System.out.println();
        }
    }
}
