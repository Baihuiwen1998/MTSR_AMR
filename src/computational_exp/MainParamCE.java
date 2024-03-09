package computational_exp;

import algorithm.AlgorithmParams;
import algorithm.InitiateSolution;
import algorithm.Solution;
import algorithm.cal_ftness.CFBasic;
import algorithm.cal_ftness.CFImpr;
import algorithm.cal_ftness.CalFitness;
import algorithm.cal_solution.CS;
import algorithm.cal_solution.CSImpr;
import algorithm.cal_solution.StartSolution;
import algorithm.order_sequencing.VNSImpr;
import gurobi.GRBException;
import instance_generation.Instance;
import instance_generation.InstanceDeserialize;
import model.ImprModel;
import support_func.CallRouteLen;
import support_func.MinRobotNoWait;
import support_func.Output;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import static algorithm.cal_ftness.CFImpr.listToCntMap;

public class MainParamCE {
    private static final long serialVersionUID = 1L;


    public static void main(String[] args) throws IOException, GRBException, ClassNotFoundException, CloneNotSupportedException {
        Output output = new Output();
        Output output_simulate = new Output();
        output.init();
        output_simulate.init();
        String output_dir = ".//data/output//";
        String input_dir = ".//data//input//Param//";
        String output_flag = "ce";
        for (String instanceName : new String[]{"Set_A_3"}){
            Instance inst;

            for (int i = 0; i < 10; i++) {
                System.out.println("-------------------------------------" + instanceName + "_" + (i + 1) + "-------------------------------------");
                inst = InstanceDeserialize.deserialize(input_dir + instanceName + "//" + (i + 1) + ".txt");
                int requiredSKUNum = output.calRequiredSKUNum(inst);
                System.out.println("epsilon=" + inst.epsilon);
                System.out.println("requiredSKUNum=" + requiredSKUNum);
                inst.orderBinCap = 3;
                inst.toteCapByStation = 3;
                inst.toteCapByRobot = 6;
                for (int mo = 2; mo <= 10; mo++) {
                    inst.orderBinCap = mo;
                    calAlgorihtm(inst, requiredSKUNum, output, output_simulate, instanceName, i, output_dir, output_flag);
                }
                inst.orderBinCap = 3;
                for (int mt = 2; mt <= 10; mt++) {
                    inst.toteCapByStation = mt;
                    calAlgorihtm(inst, requiredSKUNum, output, output_simulate, instanceName, i, output_dir, output_flag);
                }
                inst.toteCapByStation = 3 ;
                for(int cr = 4;cr<=10;cr++){
                    inst.toteCapByRobot = cr;
                    calAlgorihtm(inst, requiredSKUNum, output,output_simulate,instanceName, i, output_dir, output_flag);

                }
            }

        }

    }

    public static void calAlgorihtm(Instance instance, int requiredSKUNum, Output output, Output output_simulate, String instanceName, int i, String output_dir,String output_flag) throws IOException, GRBException, CloneNotSupportedException {
        boolean isOutput = true; // 是否输出到csv

        CalFitness cf, cfImpr;
        InitiateSolution initiateSolution;
        VNSImpr vnsImpr;
        List<Integer> skuLeaveSeq;
        int[][] orderOpenInterval, pickK;
        int[] orderSeq;
        long start, end_vns;
        MinRobotNoWait minRobotNoWait;
        if (AlgorithmParams.calFitnessMode == 0) {
            cf = new CFBasic();
        } else {
            cf = new CFImpr();
        }
        cfImpr = new CFImpr();
        initiateSolution = new InitiateSolution();
        minRobotNoWait = new MinRobotNoWait();
        // start alg
        start = System.currentTimeMillis();
        // initiate instance
        cf.init(instance);
        cfImpr.init(instance);

        // --------- initial ----------
        Solution initialSolution = initiateSolution.genInitialSolution(instance, cf);

        if (initialSolution.skuComeSeq != null) {
            System.out.println("initial obj = " + initialSolution.objVal + ", tote come num = " + initialSolution.skuComeSeq.size());
        } else {
            System.out.println("initial obj = " + initialSolution.objVal + ", infeasible");
        }
        // --------- vns ----------
        vnsImpr = new VNSImpr(instance, cf, cfImpr);
        Solution vnsSolution = vnsImpr.calVNS(initialSolution, requiredSKUNum);
        end_vns = System.currentTimeMillis();
        // prepare for improvement
        CSImpr csImpr = new CSImpr();
        csImpr.init(instance);
        if(vnsSolution.skuComeSeq == null){
            return;
        }
        StartSolution ss = csImpr.calStartSolution(vnsSolution.skuComeSeq);
        if (ss != null) {
            skuLeaveSeq = ss.skuLeaveSeq;
            orderOpenInterval = ss.orderOpenIntervals;
            pickK = ss.pickRecord;
            orderSeq = ss.orderSeq;
        } else {
            CS cs = new CS();
            cs.init(instance);
            ss = cs.calStartSolution(vnsSolution.orderSeq);
            skuLeaveSeq = ss.skuLeaveSeq;
            orderOpenInterval = ss.orderOpenIntervals;
            pickK = ss.pickRecord;
            orderSeq = ss.orderSeq;
        }
        // 顺序的修正
        // 对于只有一个sku需求的订单，必然在一轮内拣选完毕
        for (int o = 0; o < instance.orderNum; o++) {
            if (instance.skuSetByOrder[o].size() == 1) {
                orderOpenInterval[o][0] = orderOpenInterval[o][1];
            }
        }
        if (vnsSolution.skuComeSeq == null) {
            return;
        }
        // --------- improvement ----------
        ImprModel imprModel = new ImprModel();
        HashMap<Integer, Integer> skuUseNumMap = listToCntMap(vnsSolution.skuComeSeq);
        imprModel.buildModel(instance, vnsSolution.skuComeSeq, skuUseNumMap, skuLeaveSeq, orderOpenInterval, pickK, 1);
        List<List<Integer>> imprSKUComeRoutes = imprModel.solveMIPModel(skuLeaveSeq, 300.0-(end_vns - start) / 1000.0);
        List<Integer> imprSKUComeSeq = vnsSolution.skuComeSeq;
        List<Double> routeLenList = null;
        if (imprSKUComeRoutes != null) {
            imprSKUComeSeq = imprSKUComeRoutes.get(imprSKUComeRoutes.size() - 1);
            imprSKUComeRoutes.remove(imprSKUComeRoutes.size() - 1);
            CallRouteLen callRouteLen = new CallRouteLen(instance);
            routeLenList = callRouteLen.calRouteLenList(imprSKUComeRoutes, listToCntMap(imprSKUComeSeq));
            double sumRouteLen = 0.0;
            for(double routeLen:routeLenList){
                sumRouteLen += routeLen;
            }
            imprModel.objVal += sumRouteLen / instance.moveSpeed;
        }
        List<List<Integer>> soluSKUComeRoutes;
        int[] soluOrderSeq;
        List<Integer> soluSKUComeSeq, soluSKULeaveSeq;
        List<Double> solurouteLenList;
        if(imprModel.objVal != -1){
            soluSKUComeRoutes = imprSKUComeRoutes;
            solurouteLenList = routeLenList;
            soluOrderSeq = imprModel.soluOrderSeq;
            soluSKUComeSeq = imprModel.soluSKUComeSeq;
            soluSKULeaveSeq = imprModel.soluSKULeaveSeq;
        }else{
            soluSKUComeRoutes = new ArrayList<>();
            solurouteLenList = new ArrayList<>();
            List<Integer> skuComeR = new ArrayList<>();
            for(int r = 0;r<vnsSolution.routeLen.length;r++){
                if(vnsSolution.routeLen[r] >0) {
                    solurouteLenList.add(vnsSolution.routeLen[r]);
                }
                skuComeR.addAll(vnsSolution.skuComeSeq.subList(r*instance.toteCapByRobot,Math.min((r+1)*instance.toteCapByRobot, vnsSolution.skuComeSeq.size())));
                soluSKUComeRoutes.add(skuComeR);
                skuComeR = new ArrayList<>();
            }
            soluOrderSeq = orderSeq;
            soluSKUComeSeq = vnsSolution.skuComeSeq;
            soluSKULeaveSeq = skuLeaveSeq;
        }
        System.out.println(Arrays.toString(soluOrderSeq));
        System.out.println(soluSKUComeSeq.toString());
        System.out.println(soluSKULeaveSeq.toString());
        System.out.println(solurouteLenList.toString());
        System.out.println(soluSKUComeRoutes.toString());
        for(int pickT = 2;pickT<=6;pickT++) {
            minRobotNoWait = new MinRobotNoWait();
            minRobotNoWait.init(instance, (double)pickT, 0.2, 0.2, 5);
            List<Double> pickerWaitTByRobotNum = minRobotNoWait.calMinimalRobotNoWait(soluOrderSeq, soluSKUComeSeq, soluSKULeaveSeq, soluSKUComeRoutes, solurouteLenList);
            System.out.println(pickerWaitTByRobotNum.toString());
            output_simulate.addInstanceInfo(instance, instanceName + "_" + (i + 1));
            output_simulate.addSimCEInstanceInfo(pickerWaitTByRobotNum, pickT);
            output_simulate.printSimCEInstanceToCSV(output_dir  + "sim_" + output_flag + ".csv");
        }

        if(isOutput){
            output.addInstanceInfo(instance, instanceName + "_" + (i + 1));
            output.addParamCEInstanceInfo(
                    vnsSolution, (double) (end_vns - start) / 1000.0, imprModel
            );
            output.printParamCEInstanceToCSV(output_dir  + "param_ce_" + output_flag + ".csv");
        }
    }
}
