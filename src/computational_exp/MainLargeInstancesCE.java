package computational_exp;

import algorithm.AlgorithmParams;
import algorithm.InitiateSolution;
import algorithm.Solution;
import algorithm.cal_ftness.*;
import algorithm.cal_solution.CS;
import algorithm.cal_solution.CSImpr;
import algorithm.cal_solution.StartSolution;
import algorithm.order_sequencing.VNSBi;
import algorithm.order_sequencing.VNSFCFS;
import algorithm.order_sequencing.VNSImpr;
import algorithm.sku_sequencing.SSImprTabuBS;
import gurobi.GRBException;
import instance_generation.Instance;
import instance_generation.InstanceDeserialize;
import model.ImprModel;
import support_func.CallRouteLen;
import support_func.Output;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static algorithm.cal_ftness.CFImpr.listToCntMap;

public class MainLargeInstancesCE {
    private static final long serialVersionUID = 1L;
    public static void main(String[] args) throws IOException, GRBException, ClassNotFoundException, CloneNotSupportedException {
        Output output = new Output();
        output.init();
        String output_dir = ".//data/output//";
        String input_dir = ".//data//input//";
        String out_put_flag = "cmp";
        int end_i = 10;
        for (String instanceName : new String[]{
                // 大规模
                "Set_L_1","Set_L_2",
        }) {

            boolean isOutput = true; // 是否输出到csv
            // init algorithms
            CalFitness cf, cfImpr;
            InitiateSolution initiateSolution;
            long start, end_init, end_vns;
            long start_s0FCFS, end_s0FCFS, start_s0FCFO, end_s0FCFO, start_Bi, end_Bi, start_tabu_bs, end_tabu_bs, start_tb_greedy, end_tb_greedy,start_FCFS, end_FCFS;
            VNSImpr vnsImpr;
            List<Integer> skuLeaveSeq;
            int[][] orderSeq, pickK;
            Instance instance;

            for (int i =0; i <10; i++) {
                System.out.println("-------------------------------------" + instanceName + "_" + (i + 1) + "-------------------------------------");
                instance = InstanceDeserialize.deserialize(input_dir + instanceName + "//" + (i + 1) + ".txt");
                int requiredSKUNum = output.calRequiredSKUNum(instance);
                System.out.println("epsilon="+instance.epsilon);
                System.out.println("requiredSKUNum="+requiredSKUNum);
                instance.orderBinCap = 6;
                instance.toteCapByStation = 6;

                if (AlgorithmParams.calFitnessMode == 0) {
                    cf = new CFBasic();
                } else {
                    cf = new CFImpr();
                }
                cfImpr = new CFImpr();
                initiateSolution = new InitiateSolution();

                // start alg
                start = System.currentTimeMillis();
                // initiate instance
                cf.init(instance);
                cfImpr.init(instance);
                // --------- initial ----------
                Solution initialSolution = initiateSolution.genInitialSolution(instance, cf);
                end_init = System.currentTimeMillis();
                if (initialSolution.skuComeSeq != null) {
                    System.out.println("initial obj = " + initialSolution.objVal + ", tote come num = " + initialSolution.skuComeSeq.size());
                } else {
                    System.out.println("initial obj = " + initialSolution.objVal + ", infeasible");
                }
                // --------- vns ----------
                vnsImpr = new VNSImpr(instance, cf, cfImpr);
                Solution vnsSolution = vnsImpr.calVNS(initialSolution, requiredSKUNum);
                end_vns = System.currentTimeMillis();
                if (vnsSolution.skuComeSeq != null) {
                    System.out.print("best obj = " + vnsSolution.objVal + ", run time =" + (end_vns - start) + "ms, tote come num = " + vnsSolution.skuComeSeq.size() + ", skuComeSeq = ");
                    System.out.println(vnsSolution.skuComeSeq.toString());
                }
                // prepare for improvement
                CSImpr csImpr = new CSImpr();
                csImpr.init(instance);
                if(vnsSolution.skuComeSeq == null){
                    continue;
                }
                StartSolution ss = csImpr.calStartSolution(vnsSolution.skuComeSeq);
                if (ss != null) {
                    skuLeaveSeq = ss.skuLeaveSeq;
                    orderSeq = ss.orderOpenIntervals;
                    pickK = ss.pickRecord;
                } else {
                    CS cs = new CS();
                    cs.init(instance);
                    ss = cs.calStartSolution(vnsSolution.orderSeq);
                    skuLeaveSeq = ss.skuLeaveSeq;
                    orderSeq = ss.orderOpenIntervals;
                    pickK = ss.pickRecord;
                }
                // 顺序的修正
                // 对于只有一个sku需求的订单，必然在一轮内拣选完毕
                for (int o = 0; o < instance.orderNum; o++) {
                    if (instance.skuSetByOrder[o].size() == 1) {
                        orderSeq[o][0] = orderSeq[o][1];
                    }
                }
                if (vnsSolution.skuComeSeq == null) {
                    continue;
                }
                // --------- improvement ----------
                ImprModel imprModel = new ImprModel();
                HashMap<Integer, Integer> skuUseNumMap = listToCntMap(vnsSolution.skuComeSeq);
                imprModel.buildModel(instance, vnsSolution.skuComeSeq, skuUseNumMap, skuLeaveSeq, orderSeq, pickK, 1);
                List<List<Integer>> imprSKUComeRoutes = imprModel.solveMIPModel(skuLeaveSeq, 300.0-(end_vns - start) / 1000.0);
                List<Integer> imprSKUComeSeq = vnsSolution.skuComeSeq;
                if (imprSKUComeRoutes != null) {
                    imprSKUComeSeq = imprSKUComeRoutes.get(imprSKUComeRoutes.size() - 1);
                    imprSKUComeRoutes.remove(imprSKUComeRoutes.size() - 1);
                    CallRouteLen callRouteLen = new CallRouteLen(instance);
                    double routeLen = callRouteLen.calRouteLen(imprSKUComeRoutes, listToCntMap(imprSKUComeSeq));
                    imprModel.objVal += routeLen / instance.moveSpeed;
                    System.out.println("impr="+imprModel.objVal);
                }

//                vnsSolution.skuComeSeq = imprSKUComeSeq;
//                SSImprTabu ssImprTabu = new SSImprTabu();
//                ssImprTabu.init(instance);
//                vnsSolution = ssImprTabu.cal(vnsSolution);
//                end_vns = System.currentTimeMillis();
//                if (vnsSolution.skuComeSeq != null) {
//                    System.out.print("best obj = " + vnsSolution.objVal + ", run time =" + (end_vns - start) + "ms, tote come num = " + vnsSolution.skuComeSeq.size() + ", skuComeSeq = ");
//                    System.out.println(vnsSolution.skuComeSeq.toString());
//                }


//                // --------- S0-FCFS ----------
//                AlgorithmParams.skuSchedulingAlgMode = 2;
//                CalFitness cfFCFS = new CFBasic();
//                cfFCFS.init(instance);
//
//                InitiateSolution s0FCFS = new InitiateSolution();
//                start_s0FCFS = System.currentTimeMillis();
//                Solution FCFSSolution = s0FCFS.genInitialSolution(instance, cfFCFS);
//                if(FCFSSolution.skuComeSeq == null){
//                    AlgorithmParams.skuSchedulingAlgMode = 1;
//                    FCFSSolution.objVal=-1;
//                    FCFSSolution.skuComeSeq = new ArrayList<>();
//                }
//                end_s0FCFS = System.currentTimeMillis();
//
//                System.out.println("FCFS solution="+FCFSSolution.objVal);
                // --------- S0-FCFO ----------
                AlgorithmParams.skuSchedulingAlgMode = 3;
                CalFitness cfFCFO = new CFBasic();
                cfFCFO.init(instance);
                InitiateSolution s0FCFO = new InitiateSolution();
                start_s0FCFO= System.currentTimeMillis();
                Solution FCFOSolution = s0FCFO.genInitialSolution(instance, cfFCFO);
                if(FCFOSolution.skuComeSeq == null){
                    AlgorithmParams.skuSchedulingAlgMode = 1;
                    FCFOSolution.objVal=-1;
                    FCFOSolution.skuComeSeq = new ArrayList<>();
                }
                end_s0FCFO = System.currentTimeMillis();
                AlgorithmParams.skuSchedulingAlgMode = 1;
                System.out.println("FCFO solution="+FCFOSolution.objVal);

                // --------- Bi level ----------
                CalFitness cfBi = new CFBi();
                cfBi.init(instance);
                InitiateSolution initiateSolutionBi = new InitiateSolution();
                start_Bi = System.currentTimeMillis();
                Solution initialSolutionBi = initiateSolutionBi.genInitialSolution(instance, cfBi);
                Solution biSolution;
                double objBi;
                if(initialSolutionBi.skuComeSeq == null){
                    biSolution = new Solution();
                    biSolution.objVal = -1;
                    biSolution.skuComeSeq = new ArrayList<>();
                    objBi = -1;
                }else {
                    System.out.println("IniBiLevel=" + initialSolutionBi.objVal);
                    VNSBi vnsBi = new VNSBi(instance, cfBi);
                    biSolution = vnsBi.calVNS(initialSolutionBi);
                    System.out.println("BestBiLevel=" + biSolution.objVal);
                    List<List<Integer>> skuComeRoutes = new ArrayList<>();
                    List<Integer> skuComeR = new ArrayList<>();
                    for (int k = 0; k < biSolution.skuComeSeq.size(); k++) {
                        if (k % instance.toteCapByRobot == 0 & k != 0) {
                            skuComeRoutes.add(skuComeR);
                            skuComeR = new ArrayList<>();
                        }
                        skuComeR.add(biSolution.skuComeSeq.get(k));
                    }
                    if (skuComeR.size() > 0) {
                        skuComeRoutes.add(skuComeR);
                    }

                    CallRouteLen callRouteLen = new CallRouteLen(instance);
                    double routeLenBi = callRouteLen.calRouteLen(skuComeRoutes, listToCntMap(biSolution.skuComeSeq));
                    System.out.println("routeLen=" + routeLenBi);
                    objBi = biSolution.skuComeSeq.size() * instance.pickTime + routeLenBi / instance.moveSpeed;
                    System.out.println("BiLevel=" + objBi);
                }
                end_Bi = System.currentTimeMillis();
//
//                // --------- R ----------
//                if(initialSolution.skuComeSeq == null){
//                    continue;
//                }
//                List<Integer> skuLeaveSeqR;
//                int[][] orderSeqR, pickKR;
//                StartSolution ssR = csImpr.calStartSolution(initialSolution.skuComeSeq);
//                if (ssR != null) {
//                    skuLeaveSeqR = ssR.skuLeaveSeq;
//                    orderSeqR = ssR.orderOpenIntervals;
//                    pickKR = ssR.pickRecord;
//                } else {
//                    CS csR = new CS();
//                    csR.init(instance);
//                    ssR = csR.calStartSolution(initialSolution.orderSeq);
//                    skuLeaveSeqR = ssR.skuLeaveSeq;
//                    orderSeqR = ssR.orderOpenIntervals;
//                    pickKR = ssR.pickRecord;
//                }
//                 // 顺序的修正
//                 // 对于只有一个sku需求的订单，必然在一轮内拣选完毕
//                for (int o = 0; o < instance.orderNum; o++) {
//                    if (instance.skuSetByOrder[o].size() == 1) {
//                        orderSeqR[o][0] = orderSeqR[o][1];
//                    }
//                }
//                // --------- improvement ----------
//                ImprModel imprModelR = new ImprModel();
//                HashMap<Integer, Integer> skuUseNumMapR = listToCntMap(initialSolution.skuComeSeq);
//
//                imprModelR.buildModel(instance, initialSolution.skuComeSeq, skuUseNumMapR, skuLeaveSeqR, orderSeqR, pickKR);
//                List<List<Integer>> imprSKUComeRoutesR = imprModelR.solveMIPModel(skuLeaveSeqR, 1800.0-(end_init - start) / 1000.0);
//                if (imprSKUComeRoutesR != null) {
//                    List<Integer> imprSKUComeSeqR = imprSKUComeRoutesR.get(imprSKUComeRoutesR.size() - 1);
//                    imprSKUComeRoutesR.remove(imprSKUComeRoutesR.size() - 1);
//                    callRouteLen = new CallRouteLen(instance);
//                    double routeLenR = callRouteLen.calRouteLen(imprSKUComeRoutesR, listToCntMap(imprSKUComeSeqR));
//                    imprModelR.objVal += routeLenR / instance.moveSpeed;
//                }
//
//                System.out.println("R="+imprModelR.objVal);
//
//                // --------- Beam search ----------
                Solution tabuBSSolution;
                start_tabu_bs =  System.currentTimeMillis();
                if(initialSolution.skuComeSeq == null){
                    tabuBSSolution = new Solution();
                    tabuBSSolution.objVal = -1;
                    tabuBSSolution.skuComeSeq = new ArrayList<>();
                }else{
                    SSImprTabuBS ssImprBS = new SSImprTabuBS();
                    ssImprBS.init(instance);
                    tabuBSSolution = ssImprBS.cal(initialSolution);
                }
                end_tabu_bs =  System.currentTimeMillis();
                System.out.println("beam search="+tabuBSSolution.objVal+", run time =" + (end_tabu_bs - start_tabu_bs) );


////
////                // --------- Tabu ----------
//                Solution tabuGreedySolution;
//                if(initialSolution.skuComeSeq == null) {
//                    tabuGreedySolution = new Solution();
//                    tabuGreedySolution.objVal = 0;
//                    tabuGreedySolution.skuComeSeq = new ArrayList<>();
//                    start_tb_greedy = System.currentTimeMillis();
//                    end_tb_greedy = System.currentTimeMillis();
//                }else {
//                    SSImprTabuGreedy ssImprTabu = new SSImprTabuGreedy();
//                    ssImprTabu.init(instance);
//                    start_tb_greedy = System.currentTimeMillis();
//                    tabuGreedySolution = ssImprTabu.cal(initialSolution);
//                    end_tb_greedy = System.currentTimeMillis();
//                    System.out.println("tabuGreedy=" + tabuGreedySolution.objVal + ", run time =" + (end_tb_greedy - start_tb_greedy));
//                }
//
                // --------- VNSFCFS ----------
                AlgorithmParams.skuSchedulingAlgMode = 2;
                CFBasic cfFCFS = new CFBasic();
                cfFCFS.init(instance);
                AlgorithmParams.skuSchedulingAlgMode = 1;
                start_FCFS = System.currentTimeMillis();
                VNSFCFS vnsFCFS = new VNSFCFS(instance, cfFCFS, cfImpr);
                Solution vnsFCFSSolution = vnsFCFS.calVNS(initialSolution);
                end_FCFS = System.currentTimeMillis();
                if(vnsFCFSSolution == null){
                    vnsFCFSSolution = new Solution();
                    vnsFCFSSolution.objVal=-1;
                    vnsFCFSSolution.skuComeSeq = new ArrayList<>();
                }
                System.out.println("vnsFCFS="+vnsFCFSSolution.objVal+", run time =" + (end_FCFS - start_FCFS) );

                // --------- output ----------
                if (isOutput) {
                    output.addInstanceInfo(instance, instanceName + "_" + (i + 1));
                    output.addLargeInstanceInfo(initialSolution, (double) (end_init - start) / 1000.0,
                            vnsSolution, (double) (end_vns - start) / 1000.0, imprModel,
//                            ,
//                            FCFSSolution, (double)(end_s0FCFS - start_s0FCFS)/1000.0,
                            FCFOSolution,(double)(end_s0FCFO - start_s0FCFO)/1000.0,
                            objBi,biSolution, (double)(end_Bi - start_Bi)/1000.0,
//                            imprModelR,
                            tabuBSSolution, (double)(end_tabu_bs-start_tabu_bs)/1000.0,
//                            tabuGreedySolution,(double)(end_tb_greedy - start_tb_greedy)/1000.0
                            vnsFCFSSolution, (double)(end_FCFS - start_FCFS)/1000.0
                    );
                }
                output.printLargeInstanceToCSV(output_dir  + "large_all" + out_put_flag + ".csv");

            }

        }
    }


}
