package model;

import gurobi.*;
import instance_generation.Instance;

import java.io.IOException;
import java.util.*;

/*
SubModel G
求解给定SKU处理顺序（多个route）下，最小化机器人路径长度
 */
public class SubModelRouteBySCS {
    Instance instance;
    List<HashMap<Integer, Integer>> skuRequireTimesByRoute;
    List<Integer> toteList;
    List<Integer> nodeList;
    GRBEnv env;
    public GRBModel Model;
    GRBVar[][][]         z; // the instance.robot routing decision variable
    GRBVar[]           mu; // the instance.robot routing eliminate subsidiary

    public double objVal = 0; // objective value of the MIPModel
    public double runTime = 0;
    public double lowerBound = 0;
    public List<Double> routeLenList;
    int S,N,R;
    GRBLinExpr obj;

    public void robotRoutingSubProb() throws GRBException{
        this.z = new GRBVar[N+1][N+1][R];
        for (int i:nodeList) {
            for (int j:nodeList) {
                for (int r = 0; r < R; r++) {
                    String varName = "z_" + i + "_" + j + "_" + r;
                    z[i][j][r] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
                }
            }
        }

        this.mu = new GRBVar[N];
        for(int i:toteList) {
            String varName = "mu_" + i;
            mu[i] = this.Model.addVar(0, instance.toteCapByRobot + 1, 0, GRB.CONTINUOUS, varName);
        }

        // robot routing constraints
        for(int i:toteList){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int r=0;r<R;r++) {
                for (int j:nodeList) {
                    expr1.addTerm(1.0, z[i][j][r]);
                }
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "robot routing constraint: each tote can be visited by one tote or the pick station ");
        }
        for(int j:toteList){
            for (int r= 0;r<R;r++) {
                GRBLinExpr expr1 = new GRBLinExpr();
                for (int i : nodeList) {
                    expr1.addTerm(1.0, z[i][j][r]);
                    expr1.addTerm(-1.0, z[j][i][r]);
                }
                this.Model.addConstr(expr1, GRB.EQUAL, 0, "robot routing constraint: each tote only visit one tote after visiting one site");
            }
        }
        for(int r = 0;r<R;r++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int i: nodeList){
                for(int j : nodeList){
                    expr1.addTerm(1.0,z[i][j][r]);
                }
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, instance.toteCapByRobot+1, "robot routing constraint: tote capacity of each robot");
        }

        for (int i:toteList) {
            for (int r= 0; r < R; r++) {
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0, z[i][i][r]);
                this.Model.addConstr(expr1, GRB.EQUAL, 0, "robot routing constraint: no self visiting");
            }
        }

        for (int r= 0; r < R; r++) {
            GRBLinExpr expr1 = new GRBLinExpr();
            for (int i:nodeList) {
                expr1.addTerm(1.0,z[N][i][r]);
            }
            this.Model.addConstr(expr1, GRB.EQUAL, 1, "robot routing constraint: robot departs from the pick station");
        }

        for (int r= 0; r < R; r++) {
            GRBLinExpr expr1 = new GRBLinExpr();
            for (int i:nodeList) {
                expr1.addTerm(1.0,z[i][N][r]);
            }
            this.Model.addConstr(expr1, GRB.EQUAL, 1, "robot routing constraint: robot returns to the pick station");
        }
        for(int i:toteList){
            for(int j:toteList){
                for(int r = 0;r<R;r++){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0,mu[i]);
                    expr1.addTerm(-1.0,mu[j]);
                    expr1.addTerm(instance.toteCapByRobot+1,z[i][j][r]);
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, instance.toteCapByRobot, "robot routing constraint:eliminate cycle");
                }
            }
        }
        for(int i:toteList){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int j:nodeList){
                for(int r= 0;r<R;r++){
                    expr1.addTerm(1.0,z[i][j][r]);
                }
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "robot routing constraint: each tote is used at most once");
            }
        }

        HashMap<Integer, Integer> skuRequireTimes;
        /////// connection constraint /////
        for(int r = 0;r<R;r++) {
            skuRequireTimes = this.skuRequireTimesByRoute.get(r);
            for(int s:skuRequireTimes.keySet()){
                GRBLinExpr expr1 = new GRBLinExpr();
                for (int i : instance.toteSetBySKU[s]) {
                    for (int j : nodeList) {
                        expr1.addTerm(1.0, z[i][j][r]);
                    }
                }
                this.Model.addConstr(expr1, GRB.EQUAL, skuRequireTimes.get(s), "connection constraint: each tote should be planned in the route if it's scheduled");
            }
        }
    }
    public void buildModel(Instance instance, List<Integer> skuComeSeq) throws IOException, GRBException {
        this.instance = instance; // this step is necessary
        this.env = new GRBEnv("MIP_model.log");
        this.Model = new GRBModel(this.env);

        S = instance.skuNum;
        N = instance.toteNum;
        R = skuComeSeq.size()/instance.toteCapByRobot;
        if(skuComeSeq.size() % instance.toteCapByRobot>0){
            R++;
        }
        this.skuRequireTimesByRoute = new ArrayList<>();
        HashMap<Integer,Integer> skuRequireTimes;
        for(int r=0;r<R;r++){
            skuRequireTimes = new HashMap<>();
            for(int s:skuComeSeq.subList(instance.toteCapByRobot*r, Math.min(instance.toteCapByRobot*(r+1),skuComeSeq.size()))){
                skuRequireTimes.put(s, skuRequireTimes.getOrDefault(s, 0)+1);
            }
            this.skuRequireTimesByRoute.add(skuRequireTimes);
        }
        Set<Integer> skuComeSet = new HashSet<>(skuComeSeq);
        this.toteList = new ArrayList<>();
        for(int s:skuComeSet){
            toteList.addAll(instance.toteSetBySKU[s]);
        }
        this.nodeList = new ArrayList<>();
        this.nodeList.addAll(toteList);
        this.nodeList.add(N);

        /* OBJECTIVE FUNCTION */
        //minimize the robot routing distance

        robotRoutingSubProb();
        obj = new GRBLinExpr();

        for (int i:nodeList) {
            for (int j:nodeList) {
                for(int r= 0;r<R;r++){
                    obj.addTerm(instance.disBetweenTotes[i][j], z[i][j][r]);
                }
            }
        }
        this.Model.setObjective(obj, GRB.MINIMIZE);
    }

    public void setGurobiParam(int outPutFlag) throws GRBException {
        // STOP CRITERIA
        if(outPutFlag == 1) {
            this.Model.set(GRB.DoubleParam.TimeLimit, 3600);
        }
        this.Model.set(GRB.IntParam.OutputFlag, outPutFlag);
    }

    public double solveMIPModel(int outPutFlag) throws GRBException, IOException {
        /* SOLVING */
        setGurobiParam(outPutFlag);
        this.Model.optimize();
        if (this.Model.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL|| this.Model.get(GRB.IntAttr.Status) == GRB.Status.TIME_LIMIT) {
            this.objVal = this.Model.get(GRB.DoubleAttr.ObjVal);
            this.runTime = this.Model.get(GRB.DoubleAttr.Runtime);
            this.lowerBound = this.Model.get(GRB.DoubleAttr.ObjBound);
//            this.routeLenList = new ArrayList<>();
//            List<List<Integer>> toteComeSetList = new ArrayList<>();
//            List<List<Integer>> skuComeSetList = new ArrayList<>();
//            List<Integer> toteComeList;
//            List<Integer> skuComeList;
//            for(int r=0;r<R;r++){
//                double routeLen = 0.0;
//                toteComeList = new ArrayList<>();
//                skuComeList = new ArrayList<>();
//                System.out.print("route_"+r+":");
//                for (int i:nodeList) {
//                    for (int j:nodeList) {
//                        double zValue = this.z[i][j][r].get(GRB.DoubleAttr.X);
//                        if (zValue > 0) {
//                            routeLen += instance.disBetweenTotes[i][j];
//                            if(j != N){
////                                toteComeList.add(j);
//                                skuComeList.add(instance.skuByTote[j]);
////                                System.out.print(i+"_"+j+"("+instance.skuByTote[j]+"),");
////                                System.out.print(instance.skuByTote[j]+"_");
//                            }
//                        }
//                    }
//                }
////                System.out.println(routeLen);
////                toteComeSetList.add(toteComeList);
////                skuComeSetList.add(skuComeList);
//                this.routeLenList.add(routeLen);
//            }

            this.Model.dispose();
            this.env.dispose();
//            return toteComeSetList;
//            return skuComeSetList;
            return this.objVal;
        }
        else {
//            this.Model.computeIIS();
//            this.Model.write("model.ilp");
            System.out.println("SubModelBySCS not solved");
            // this.MIPModel.dispose();
            // this.env.dispose();
            return -1.0;
        }
    }
}