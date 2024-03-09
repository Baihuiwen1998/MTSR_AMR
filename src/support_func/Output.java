package support_func;

import algorithm.AlgorithmParams;
import algorithm.Solution;
import gurobi.*;
import instance_generation.Instance;
import model.ImprModel;
import model.P0Model;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

public class Output {
    List<Integer> orderNumList, toteNumList, skuNumList, requiredSKUNumList, moList, mtList, crList;
    List<Double> orderEpsilonList, initialObjList, vnsObjList, vnsRObjList, runTimeList, vnsRunTimeList, initialRunTimeList, lowerBoundList;
    List<Integer> initialToteVisitNumList, vnsToteVisitNumList, vnsRToteVisitNumList;
    List<String> instanceNameList;
    List<Double> biObjList, biRunTimeList, FCFOObjList, FCFORunTimeList, FCFSObjList, FCFSRunTimeList, vnsFCFSObjList, vnsFCFSRunTimeList;
    List<Integer> biToteVisitNumList, FCFOToteVisitNumList, FCFSToteVisitNumList, vnsFCFSToteVisitNumList;
    List<Double> rObjList, rRunTimeList, tabuGreedyObjList, tabuGreedyRunTimeList, tabuBSObjList, tabuBSRunTimeList;
    List<Integer> rToteVisitNumList, tabuGreedyToteVisitNumList, tabuBSToteVisitNumList;
    List<List<Double>> pickerWaitTByRobotNumList;
    List<Double> pickTList;

    List<Double> R0_ObjList, R2_ObjList, R0_RunTimeList, R2_RunTimeList;
    List<Integer> R0_ToteVisitNumList, R2_ToteVisitNumList;

    List<Integer> paramList;


    public void init(){
        this.orderNumList = new ArrayList<>();
        this.toteNumList = new ArrayList<>();
        this.skuNumList = new ArrayList<>();
        this.requiredSKUNumList = new ArrayList<>();
        this.moList = new ArrayList<>();
        this.mtList = new ArrayList<>();
        this.crList = new ArrayList<>();
        this.orderEpsilonList = new ArrayList<>();
        this.initialObjList = new ArrayList<>();
        this.vnsRObjList = new ArrayList<>();
        this.vnsObjList = new ArrayList<>();
        this.runTimeList = new ArrayList<>();
        this.vnsRunTimeList = new ArrayList<>();
        this.initialRunTimeList = new ArrayList<>();
        this.lowerBoundList = new ArrayList<>();
        this.initialToteVisitNumList = new ArrayList<>();
        this.vnsToteVisitNumList = new ArrayList<>();
        this.vnsRToteVisitNumList = new ArrayList<>();
        this.instanceNameList = new ArrayList<>();
        this.FCFOObjList = new ArrayList<>();
        this.FCFORunTimeList = new ArrayList<>();
        this.FCFOToteVisitNumList = new ArrayList<>();
        this.FCFSObjList = new ArrayList<>();
        this.FCFSRunTimeList = new ArrayList<>();
        this.FCFSToteVisitNumList = new ArrayList<>();
        this.biObjList = new ArrayList<>();
        this.biRunTimeList = new ArrayList<>();
        this.biToteVisitNumList = new ArrayList<>();
        this.rObjList = new ArrayList<>();
        this.rRunTimeList = new ArrayList<>();
        this.rToteVisitNumList = new ArrayList<>();
        this.tabuBSObjList = new ArrayList<>();
        this.tabuBSRunTimeList = new ArrayList<>();
        this.tabuBSToteVisitNumList = new ArrayList<>();
        this.tabuGreedyObjList = new ArrayList<>();
        this.tabuGreedyRunTimeList = new ArrayList<>();
        this.tabuGreedyToteVisitNumList = new ArrayList<>();
        this.vnsFCFSObjList = new ArrayList<>();
        this.vnsFCFSRunTimeList = new ArrayList<>();
        this.vnsFCFSToteVisitNumList = new ArrayList<>();
        this.pickerWaitTByRobotNumList = new ArrayList<>();
        this.pickTList = new ArrayList<>();
        this.R0_ObjList = new ArrayList<>();
        this.R2_ObjList = new ArrayList<>();
        this.R0_RunTimeList  = new ArrayList<>();
        this.R2_RunTimeList  = new ArrayList<>();
        this.R0_ToteVisitNumList = new ArrayList<>();
        this.R2_ToteVisitNumList = new ArrayList<>();
        this.paramList = new ArrayList<>();
    }
    public void addInstanceInfo(Instance instance, String fileName) {
        this.orderNumList.add(instance.orderNum);
        this.toteNumList.add(instance.toteNum);
        this.skuNumList.add(instance.skuNum);
        this.requiredSKUNumList.add(calRequiredSKUNum(instance));
        this.orderEpsilonList.add(instance.epsilon);
        this.moList.add(instance.orderBinCap);
        this.mtList.add(instance.toteCapByStation);
        this.crList.add(instance.toteCapByRobot);
        this.instanceNameList.add(fileName);
    }

    public void addNFMIPInfo(P0Model model) throws GRBException {
        if(model.objVal > 0) {
            this.vnsRObjList.add(model.objVal);
            this.runTimeList.add(model.runTime);
            this.lowerBoundList.add(model.lowerBound);
            this.vnsRToteVisitNumList.add(model.toteVisitNum);
        }else{
            this.vnsRObjList.add(-1.0);
            this.runTimeList.add(-1.0);
            this.lowerBoundList.add(-1.0);
            this.vnsRToteVisitNumList.add(-1);
        }
    }
//    public void addMIPNFInfo(NFFullModel model) throws GRBException {
//        if(model.objVal > 0) {
//            this.bestObjList.add(model.objVal);
//            this.runTimeList.add(model.runTime);
//            this.lowerBoundList.add(model.lowerBound);
//            this.bestToteVisitNumList.add(model.toteVisitNum);
//        }else{
//            this.bestObjList.add(-1.0);
//            this.runTimeList.add(-1.0);
//            this.lowerBoundList.add(-1.0);
//            this.bestToteVisitNumList.add(-1);
//        }
//    }
    public void printMIPToCSV(String fileName) throws IOException {
        File file = new File(fileName);
        OutputStreamWriter ow = new OutputStreamWriter(new FileOutputStream(file),"gbk");
        ow.write("orderNum");
        ow.write(",");
        ow.write("skuNum");
        ow.write(",");
        ow.write("toteNum");
        ow.write(",");
        ow.write("MO");
        ow.write(",");
        ow.write("MT");
        ow.write(",");
        ow.write("CR");
        ow.write(",");
        ow.write("instanceNo");
        ow.write(",");
        ow.write("epsilon");
        ow.write(",");
        ow.write("requiredSKUNum");
        ow.write(",");
        ow.write("MIPObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("lowerBound");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write("\r\n");
        for(int i=0;i<this.orderNumList.size();i++){
            ow.write(String.valueOf(orderNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(skuNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(toteNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(moList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(mtList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(crList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(instanceNameList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(orderEpsilonList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(requiredSKUNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",runTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(lowerBoundList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRToteVisitNumList.get(i)));
            ow.write("\r\n");
        }
        ow.flush();
        ow.close();
    }

//    public void addBCHInfo(Solution bestSolution, double runTime, SubModelOSBySKUSet bch_model, double bestObj){
//        this.initialObjList.add(bestSolution.objVal);
//        if(bestSolution.skuComeSeq != null){
//            this.initialToteVisitNumList.add(bestSolution.skuComeSeq.size());
//        }else{
//            this.bestToteVisitNumList.add(-1);
//        }
//        this.initialRunTimeList.add(runTime);
//        this.bestObjList.add(Math.max(bestObj, bch_model.objVal));
//        this.bestToteVisitNumList.add(bch_model.toteVisitNum);
//        this.lowerBoundList.add(bch_model.lowerBound);
//        this.runTimeList.add(bch_model.runTime+runTime);
//    }

    public void printBCHToCSV(String fileName) throws IOException {
        File file = new File(fileName);
        OutputStreamWriter ow = new OutputStreamWriter(new FileOutputStream(file),"gbk");
        ow.write("orderNum");
        ow.write(",");
        ow.write("skuNum");
        ow.write(",");
        ow.write("toteNum");
        ow.write(",");
        ow.write("MO");
        ow.write(",");
        ow.write("MT");
        ow.write(",");
        ow.write("CR");
        ow.write(",");
        ow.write("instanceNo");
        ow.write(",");
        ow.write("epsilon");
        ow.write(",");
        ow.write("requiredSKUNum");
        ow.write(",");
        ow.write("IniObjByVNS");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("BCHObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("LB");
        ow.write("\r\n");
        for(int i=0;i<this.orderNumList.size();i++){
            ow.write(String.valueOf(orderNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(skuNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(toteNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(moList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(mtList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(crList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(instanceNameList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(orderEpsilonList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(requiredSKUNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(initialObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",initialRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(initialToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",runTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(lowerBoundList.get(i)));
            ow.write("\r\n");
        }
        ow.flush();
        ow.close();
    }

    public void addAlgInfo(Solution initialSolution, Solution bestSolution,double initRunTime, double runTime) throws GRBException{
        this.initialObjList.add(initialSolution.objVal);
        if(initialSolution.skuComeSeq != null){
            this.initialToteVisitNumList.add(initialSolution.skuComeSeq.size());
        }else{
            this.initialToteVisitNumList.add(-1);
        }
        this.vnsRObjList.add(bestSolution.objVal);
        if(bestSolution.skuComeSeq != null){
            this.vnsRToteVisitNumList.add(bestSolution.skuComeSeq.size());
        }else{
            this.vnsRToteVisitNumList.add(-1);
        }
        this.initialRunTimeList.add(initRunTime);
        this.runTimeList.add(runTime);
    }

    public void printAlgToCSV(String fileName) throws IOException {
        File file = new File(fileName);
        OutputStreamWriter ow = new OutputStreamWriter(new FileOutputStream(file),"gbk");
        ow.write("orderNum");
        ow.write(",");
        ow.write("skuNum");
        ow.write(",");
        ow.write("toteNum");
        ow.write(",");
        ow.write("MO");
        ow.write(",");
        ow.write("MT");
        ow.write(",");
        ow.write("CR");
        ow.write(",");
        ow.write("instanceNo");
        ow.write(",");
        ow.write("epsilon");
        ow.write(",");
        ow.write("requiredSKUNum");
        ow.write(",");
        ow.write("IniObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("AlgObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write("\r\n");
        for(int i=0;i<this.orderNumList.size();i++){
            ow.write(String.valueOf(orderNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(skuNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(toteNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(moList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(mtList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(crList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(instanceNameList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(orderEpsilonList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(requiredSKUNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(initialObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",initialRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(initialToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",runTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRToteVisitNumList.get(i)));
            ow.write("\r\n");
        }
        ow.flush();
        ow.close();
    }

    public void addImprInfo(Solution initialSolution, Solution stage1Solution, double initRunTime, double stage1RunTime, ImprModel impr_model){
        this.initialObjList.add(initialSolution.objVal);
        if(initialSolution.skuComeSeq != null){
            this.initialToteVisitNumList.add(initialSolution.skuComeSeq.size());
        }else{
            this.initialToteVisitNumList.add(-1);
        }
        this.initialRunTimeList.add(initRunTime);

        this.vnsObjList.add(stage1Solution.objVal);
        if(stage1Solution.skuComeSeq != null){
            this.vnsToteVisitNumList.add(stage1Solution.skuComeSeq.size());
        }else{
            this.vnsToteVisitNumList.add(-1);
        }
        this.vnsRunTimeList.add(stage1RunTime);

        if(impr_model.objVal != -1){
            this.vnsRObjList.add(impr_model.objVal);
            this.vnsRToteVisitNumList.add(impr_model.toteVisitNum);
        }else{
            this.vnsRObjList.add(stage1Solution.objVal);
            this.vnsRToteVisitNumList.add(stage1Solution.skuComeSeq.size());
        }
        this.lowerBoundList.add(impr_model.lowerBound);
        this.runTimeList.add(impr_model.runTime+stage1RunTime);
    }

    public void printImprToCSV(String fileName) throws IOException {
        File file = new File(fileName);
        OutputStreamWriter ow = new OutputStreamWriter(new FileOutputStream(file),"gbk");
        ow.write("orderNum");
        ow.write(",");
        ow.write("skuNum");
        ow.write(",");
        ow.write("toteNum");
        ow.write(",");
        ow.write("MO");
        ow.write(",");
        ow.write("MT");
        ow.write(",");
        ow.write("CR");
        ow.write(",");
        ow.write("instanceNo");
        ow.write(",");
        ow.write("epsilon");
        ow.write(",");
        ow.write("requiredSKUNum");
        ow.write(",");
        ow.write("InitialObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("Stage1Obj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("BCHObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("LB");
        ow.write("\r\n");
        for(int i=0;i<this.orderNumList.size();i++){
            ow.write(String.valueOf(orderNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(skuNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(toteNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(moList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(mtList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(crList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(instanceNameList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(orderEpsilonList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(requiredSKUNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(initialObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",initialRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(initialToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f", vnsRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",runTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(lowerBoundList.get(i)));
            ow.write("\r\n");
        }
        ow.flush();
        ow.close();
    }


    public void addLargeInstanceInfo(Solution initialSolution,double initRunTime,
                                     Solution vnsSolution,  double stage1RunTime,
                                     ImprModel imprModel
            ,
//                                     Solution FCFSSolution, double FCFSRunTime,
                                    Solution FCFOSolution, double FCFORunTime,
                                    double biObj, Solution biSolution, double biRunTime,
//                                    ImprModel imprModelR,
                                    Solution tabuBSSolution, double tabuBSRunTime,
//                                    Solution tabuGreedySolution,double tabuGreedyRunTime
                                     Solution vnsFCFSSolution,double vnsFCFSRunTime
){
        this.initialObjList.add(initialSolution.objVal);
        if(initialSolution.skuComeSeq != null){
            this.initialToteVisitNumList.add(initialSolution.skuComeSeq.size());
        }else{
            this.initialToteVisitNumList.add(-1);
        }
        this.initialRunTimeList.add(initRunTime);


        this.vnsObjList.add(vnsSolution.objVal);
        if(vnsSolution.skuComeSeq != null){
            this.vnsToteVisitNumList.add(vnsSolution.skuComeSeq.size());
        }else{
            this.vnsToteVisitNumList.add(-1);
        }
        this.vnsRunTimeList.add(stage1RunTime);

        if(imprModel.objVal != -1){
            this.vnsRObjList.add(imprModel.objVal);
            this.vnsRToteVisitNumList.add(imprModel.toteVisitNum);
        }else{
            this.vnsRObjList.add(vnsSolution.objVal);
            this.vnsRToteVisitNumList.add(vnsSolution.skuComeSeq.size());
        }
        this.lowerBoundList.add(imprModel.lowerBound);
        this.runTimeList.add(imprModel.runTime+stage1RunTime);

//        this.FCFSObjList.add(FCFSSolution.objVal);
//        this.FCFSRunTimeList.add(FCFSRunTime+initRunTime);
//        this.FCFSToteVisitNumList.add(FCFSSolution.skuComeSeq.size());
//
        this.FCFOObjList.add(FCFOSolution.objVal);
        this.FCFORunTimeList.add(FCFORunTime+initRunTime);
        this.FCFOToteVisitNumList.add(FCFOSolution.skuComeSeq.size());

        this.biObjList.add(biObj);
        this.biRunTimeList.add(biRunTime);
        this.biToteVisitNumList.add(biSolution.skuComeSeq.size());
//
//        this.rObjList.add(imprModelR.objVal);
//        this.rRunTimeList.add(initRunTime+imprModelR.runTime);
//        this.rToteVisitNumList.add(vnsSolution.skuComeSeq.size());
        this.tabuBSObjList.add(tabuBSSolution.objVal);
        this.tabuBSRunTimeList.add(tabuBSRunTime);
        this.tabuBSToteVisitNumList.add(tabuBSSolution.skuComeSeq.size());
//
//
//        this.tabuGreedyObjList.add(tabuGreedySolution.objVal);
//        this.tabuGreedyRunTimeList.add(tabuGreedyRunTime);
//        this.tabuGreedyToteVisitNumList.add(tabuGreedySolution.skuComeSeq.size());
//

        this.vnsFCFSObjList.add(vnsFCFSSolution.objVal);
        this.vnsFCFSRunTimeList.add(vnsFCFSRunTime+initRunTime);
        this.vnsFCFSToteVisitNumList.add(vnsFCFSSolution.skuComeSeq.size());
    }

    public void printLargeInstanceToCSV(String fileName) throws IOException {
        File file = new File(fileName);
        OutputStreamWriter ow = new OutputStreamWriter(new FileOutputStream(file),"gbk");
        ow.write("orderNum");
        ow.write(",");
        ow.write("skuNum");
        ow.write(",");
        ow.write("toteNum");
        ow.write(",");
        ow.write("MO");
        ow.write(",");
        ow.write("MT");
        ow.write(",");
        ow.write("CR");
        ow.write(",");
        ow.write("instanceNo");
        ow.write(",");
        ow.write("epsilon");
        ow.write(",");
        ow.write("requiredSKUNum");
        ow.write(",");
        ow.write("InitialObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("VNSObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("VNSRObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("LB");
        ow.write(",");
//        ow.write("FCFSObj");
//        ow.write(",");
//        ow.write("runTime");
//        ow.write(",");
//        ow.write("toteVisitNum");
//        ow.write(",");
        ow.write("FCFOObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("biObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
//        ow.write("RObj");
//        ow.write(",");
//        ow.write("runTime");
//        ow.write(",");
//        ow.write("toteVisitNum");
//        ow.write(",");
        ow.write("TabuBeamSearchObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
//        ow.write(",");
//        ow.write("TabuGreedyObj");
//        ow.write(",");
//        ow.write("runTime");
//        ow.write(",");
//        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("vnsFCFSObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write("\r\n");
        for(int i=0;i<this.orderNumList.size();i++){
            ow.write(String.valueOf(orderNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(skuNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(toteNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(moList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(mtList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(crList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(instanceNameList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(orderEpsilonList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(requiredSKUNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(initialObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",initialRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(initialToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f", vnsRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",runTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(lowerBoundList.get(i)));
            ow.write(",");
//            ow.write(String.valueOf(FCFSObjList.get(i)));
//            ow.write(",");
//            ow.write(String.format("%.3f",FCFSRunTimeList.get(i)));
//            ow.write(",");
//            ow.write(String.valueOf(FCFSToteVisitNumList.get(i)));
//            ow.write(",");
            ow.write(String.valueOf(FCFOObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",FCFORunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(FCFOToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(biObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",biRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(biToteVisitNumList.get(i)));
            ow.write(",");
//            ow.write(String.valueOf(rObjList.get(i)));
//            ow.write(",");
//            ow.write(String.format("%.3f",rRunTimeList.get(i)));
//            ow.write(",");
//            ow.write(String.valueOf(rToteVisitNumList.get(i)));
//            ow.write(",");
            ow.write(String.valueOf(tabuBSObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",tabuBSRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(tabuBSToteVisitNumList.get(i)));
            ow.write(",");
//            ow.write(String.valueOf(tabuGreedyObjList.get(i)));
//            ow.write(",");
//            ow.write(String.format("%.3f",tabuGreedyRunTimeList.get(i)));
//            ow.write(",");
//            ow.write(String.valueOf(tabuGreedyToteVisitNumList.get(i)));
//            ow.write(",");
            ow.write(String.valueOf(vnsFCFSObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f",vnsFCFSRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsFCFSToteVisitNumList.get(i)));
            ow.write("\r\n");
        }
        ow.flush();
        ow.close();
    }



    public void addReschedulingInfo(Solution vnsSolution,  double stage1RunTime,
                                       ImprModel imprModel, ImprModel imprModel_no_cut, ImprModel imprModel_with_rule
    ){
        this.vnsObjList.add(vnsSolution.objVal);
        if(vnsSolution.skuComeSeq != null){
            this.vnsToteVisitNumList.add(vnsSolution.skuComeSeq.size());
        }else{
            this.vnsToteVisitNumList.add(-1);
        }
        this.vnsRunTimeList.add(stage1RunTime);
        if(imprModel.objVal != -1){
            this.vnsRObjList.add(imprModel.objVal);
            this.vnsRToteVisitNumList.add(imprModel.toteVisitNum);
        }else{
            this.vnsRObjList.add(-1.0);
            this.vnsRToteVisitNumList.add(-1);
        }
        this.runTimeList.add(imprModel.runTime+stage1RunTime);

        if(imprModel_no_cut.objVal != -1){
            this.R0_ObjList.add(imprModel_no_cut.objVal);
            this.R0_ToteVisitNumList.add(imprModel_no_cut.toteVisitNum);
        }else{
            this.R0_ObjList.add(-1.0);
            this.R0_ToteVisitNumList.add(-1);
        }
        this.R0_RunTimeList.add(imprModel_no_cut.runTime+stage1RunTime);

        if(imprModel_with_rule.objVal != -1){
            this.R2_ObjList.add(imprModel_with_rule.objVal);
            this.R2_ToteVisitNumList.add(imprModel_with_rule.toteVisitNum);
        }else{
            this.R2_ObjList.add(-1.0);
            this.R2_ToteVisitNumList.add(-1);
        }
        this.R2_RunTimeList.add(imprModel_with_rule.runTime+stage1RunTime);

    }

    public void printReschedulingToCSV(String fileName) throws IOException {
        File file = new File(fileName);
        OutputStreamWriter ow = new OutputStreamWriter(new FileOutputStream(file),"gbk");
        ow.write("orderNum");
        ow.write(",");
        ow.write("skuNum");
        ow.write(",");
        ow.write("toteNum");
        ow.write(",");
        ow.write("MO");
        ow.write(",");
        ow.write("MT");
        ow.write(",");
        ow.write("CR");
        ow.write(",");
        ow.write("instanceNo");
        ow.write(",");
        ow.write("epsilon");
        ow.write(",");
        ow.write("requiredSKUNum");
        ow.write(",");
        ow.write("VNSObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("VNSRObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("R0Obj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("R2Obj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write("\r\n");
        for(int i=0;i<this.orderNumList.size();i++){

            ow.write(String.valueOf(orderNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(skuNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(toteNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(moList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(mtList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(crList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(instanceNameList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(orderEpsilonList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(requiredSKUNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f", vnsRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f", runTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(R0_ObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f", R0_RunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(R0_ToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(R2_ObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f", R2_RunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(R2_ToteVisitNumList.get(i)));
            ow.write("\r\n");

        }
        ow.flush();
        ow.close();
    }

    public void addParamCEInstanceInfo(Solution vnsSolution,  double stage1RunTime,
                                     ImprModel imprModel
    ){
        this.vnsObjList.add(vnsSolution.objVal);
        this.vnsToteVisitNumList.add(vnsSolution.skuComeSeq.size());
        this.vnsRunTimeList.add(stage1RunTime);
        if(imprModel.objVal != -1){
            this.vnsRObjList.add(imprModel.objVal);
            this.vnsRToteVisitNumList.add(imprModel.toteVisitNum);
        }else{
            this.vnsRObjList.add(vnsSolution.objVal);
            this.vnsRToteVisitNumList.add(vnsSolution.skuComeSeq.size());
        }
        this.lowerBoundList.add(imprModel.lowerBound);
        this.runTimeList.add(imprModel.runTime+stage1RunTime);
        this.paramList.add(AlgorithmParams.delta_k_add);
        this.paramList.add(AlgorithmParams.delta_k_minus);

    }

    public void printParamCEInstanceToCSV(String fileName) throws IOException {
        File file = new File(fileName);
        OutputStreamWriter ow = new OutputStreamWriter(new FileOutputStream(file),"gbk");
        ow.write("orderNum");
        ow.write(",");
        ow.write("skuNum");
        ow.write(",");
        ow.write("toteNum");
        ow.write(",");
        ow.write("MO");
        ow.write(",");
        ow.write("MT");
        ow.write(",");
        ow.write("CR");
        ow.write(",");
        ow.write("instanceNo");
        ow.write(",");
        ow.write("epsilon");
        ow.write(",");
        ow.write("requiredSKUNum");
        ow.write(",");
        ow.write("VNSObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("VNSRObj");
        ow.write(",");
        ow.write("runTime");
        ow.write(",");
        ow.write("toteVisitNum");
        ow.write(",");
        ow.write("LB");
        ow.write(",");
        ow.write("delta_k_add");
        ow.write(",");
        ow.write("delta_k_minus");
        ow.write("\r\n");
        for(int i=0;i<this.orderNumList.size();i++){

            ow.write(String.valueOf(orderNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(skuNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(toteNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(moList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(mtList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(crList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(instanceNameList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(orderEpsilonList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(requiredSKUNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f", vnsRunTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRObjList.get(i)));
            ow.write(",");
            ow.write(String.format("%.3f", runTimeList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(vnsRToteVisitNumList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(lowerBoundList.get(i)));
            ow.write(",");
            ow.write(String.valueOf(paramList.get(2*i)));
            ow.write(",");
            ow.write(String.valueOf(paramList.get(2*i+1)));
            ow.write("\r\n");

        }
        ow.flush();
        ow.close();
    }

    public void addSimCEInstanceInfo(List<Double> pickerWaitTByRobotNum, double pickT){
        this.pickerWaitTByRobotNumList.add(pickerWaitTByRobotNum);
        this.pickTList.add(pickT);
    }

    public void printSimCEInstanceToCSV(String fileName) throws IOException {
        File file = new File(fileName);
        OutputStreamWriter ow = new OutputStreamWriter(new FileOutputStream(file),"gbk");
        ow.write("orderNum");
        ow.write(",");
        ow.write("skuNum");
        ow.write(",");
        ow.write("toteNum");
        ow.write(",");
        ow.write("MO");
        ow.write(",");
        ow.write("MT");
        ow.write(",");
        ow.write("CR");
        ow.write(",");
        ow.write("instanceNo");
        ow.write(",");
        ow.write("epsilon");
        ow.write(",");
        ow.write("requiredSKUNum");
        ow.write(",");
        ow.write("pickT");
        ow.write(",");
        ow.write("robotNum");
        ow.write(",");
        ow.write("waitTime");
        ow.write(",");
        ow.write("leastRobotNumNoWait");
        ow.write("\r\n");
        for(int i=0;i<this.orderNumList.size();i++){
            for(int r = 0;r<this.pickerWaitTByRobotNumList.get(i).size();r++) {
                ow.write(String.valueOf(orderNumList.get(i)));
                ow.write(",");
                ow.write(String.valueOf(skuNumList.get(i)));
                ow.write(",");
                ow.write(String.valueOf(toteNumList.get(i)));
                ow.write(",");
                ow.write(String.valueOf(moList.get(i)));
                ow.write(",");
                ow.write(String.valueOf(mtList.get(i)));
                ow.write(",");
                ow.write(String.valueOf(crList.get(i)));
                ow.write(",");
                ow.write(String.valueOf(instanceNameList.get(i)));
                ow.write(",");
                ow.write(String.valueOf(orderEpsilonList.get(i)));
                ow.write(",");
                ow.write(String.valueOf(requiredSKUNumList.get(i)));
                ow.write(",");
                ow.write(String.valueOf(pickTList.get(i)));
                ow.write(",");
                ow.write(String.valueOf(r + 1));
                ow.write(",");
                ow.write(String.valueOf(this.pickerWaitTByRobotNumList.get(i).get(r)));
                ow.write(",");
                ow.write(String.valueOf(this.pickerWaitTByRobotNumList.get(i).size()));
                ow.write("\r\n");
            }
        }
        ow.flush();
        ow.close();
    }

    public int calRequiredSKUNum(Instance instance){
        /*
        计算实际需要的sku种类数/
         */
        int requiredSKUNum = 0;
        for(int s = 0; s<instance.skuNum;s++){
            if(instance.orderSetBySKU[s].size() >0){
                requiredSKUNum++;
            }
        }
        return requiredSKUNum;
    }
}
