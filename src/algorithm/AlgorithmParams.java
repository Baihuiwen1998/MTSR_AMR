package algorithm;

public class AlgorithmParams {
    public static int maxVNSIteration = 100;
    public static int maxNeighbourhoodNum = 4;
    public static int neighSearchSize = 500;
    public static int forwardOrderNum = 3; // 给定订单顺序下，对料箱顺序排序时，把未来k个排序的订单纳入考虑
    public static int maxSSAImprIteraion = 50; // 获取greedy料箱顺序后，对其进行改进的最大次数
    public static int maxSSAImprPairs = 100; // 改进skuComeSeq时，挑选的skuPair数量

    public static int calFitnessMode = 0; // 0-stage 1&2 (order and sku scheduling) 和 stage 3&4 ( tote selection and robot routing) 两个阶段分离； 1-嵌套求解

    public static int skuSchedulingAlgMode = 1; //  0-MIP求解, 1-greedy算法求解,
    public static int toteSelectionAlgMode = 1; // 0-MIP求解, 1-LSA5_2 算法

    public static int delta_k_add = 2;
    public static int delta_k_minus = 2;
    public static int exp_no = 0;

}
