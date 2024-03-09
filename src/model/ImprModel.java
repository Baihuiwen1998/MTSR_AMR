package model;

import algorithm.AlgorithmParams;
import gurobi.*;
import instance_generation.Instance;

import java.io.IOException;
import java.util.*;

/*
改进模型
给定SKU的到达顺序，求解可行的订单顺序 + 机器人的路径，允许sku不到达
 */
public class ImprModel {
    public GRBModel Model;
    Instance instance;
    GRBEnv env;
    GRBVar[] x;
    int[] fixedX;
    GRBVar[][] y;
    GRBVar[][] alpha;
    GRBVar[][] beta;
    GRBVar[][][] gamma;
    GRBVar[]                U; //auxiliary variable
    GRBVar[][]              W;
    GRBVar[][]              w;
    GRBVar[][]              v;
    GRBVar[]                kappa; // first come sequence of order, continuous

    GRBVar[][]              hat_y; // last leave sequence of sku, binary


    public double objVal = 0; // objective value of the MIPModel
    public double runTime = 0;
    public double lowerBound = 0;
    public int toteVisitNum=0;
    int O,S,N,K;
    GRBLinExpr obj;
    List<Integer> skuComeSeq;
    Set<Integer> skuSet;
    HashMap<Integer, Integer> skuUseNumMap;
    List<Integer> toteList;
    List<Integer> nodeList;

    int[] earliestCloseKofOrder; // order最早可以完成拣选的k
    int[] lastComeSKUofOrder;
    int[] lastSKUVisitKofOrder; // order所需SKU中，最后一个到达的k（包括重复访问）
    int[] firstOpenKofOrder;
    int[] firstOpenKofSKU;
    HashMap<Integer, List<Integer>> ordersLastRequiringSKU;
    public List<Integer> soluSKUComeSeq, soluSKULeaveSeq;
    public int[] soluOrderSeq;

    public void orderPickingSubProb(List<Integer> skuLeaveSeq, int[][] orderSeq) throws GRBException {
        /* DECISION VARIABLES */
        /*
         * GRBVar addVar ( double lb, double ub, double obj, char type, String name )
         */
        this.x = new GRBVar[this.skuComeSeq.size()];
        this.fixedX = new int[this.skuComeSeq.size()];
        for(int k = 0;k<this.skuComeSeq.size();k++){
            int s = this.skuComeSeq.get(k);
            if(skuUseNumMap.get(s)>1){
                String varName = "x_"+ k;
                x[k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
                x[k].set(GRB.DoubleAttr.Start,1.0);
                fixedX[k] = 0;
            }else{
                fixedX[k] = 1;
            }
        }

        this.y = new GRBVar[S][K];

        for(int s:this.skuSet){
            for(int k=0;k<K;k++){
                String varName = "y_" + s + "_" + k;
                y[s][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
                if(k>=instance.toteCapByStation-1 & k<skuLeaveSeq.size()+instance.toteCapByStation-1){
                    if(s==skuLeaveSeq.get(k-instance.toteCapByStation+1)){
                        y[s][k].set(GRB.DoubleAttr.Start, 1.0);
                    }else{
                        y[s][k].set(GRB.DoubleAttr.Start, 0.0);
                    }
                }
            }
        }

        this.alpha =new GRBVar[O][K];
        for(int o =0;o<O;o++){
            orderSeq[o][0] = Math.max(orderSeq[o][0],firstOpenKofOrder[o]);
            for(int k = 0;k<K;k++){
                String varName = "alpha_" + o + "_" + k;
                alpha[o][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
                if(k<=orderSeq[o][1] && k>=orderSeq[o][0]){
                    alpha[o][k].set(GRB.DoubleAttr.Start, 1.0);
                }

            }
        }

        this.beta =new GRBVar[O][K];
        for(int o =0;o<O;o++){
            for(int k = 0;k<K;k++){
                String varName = "beta_" + o + "_" + k;
                beta[o][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
            }
        }

        this.gamma = new GRBVar[S][O][K];
        for (int o = 0;o<O;o++){
            for(int s:this.instance.skuSetByOrder[o]){
                for(int k = 0;k<K;k++){
                    String varName = "gamma_" + s + "_" + o + "_" + k;
                    gamma[s][o][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
                }
            }
        }

        this.U = new GRBVar[K];
        for(int k=0;k<K;k++){
            String varName = "U_" + k;
            U[k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
        }

        this.W = new GRBVar[3][K];
        for(int i = 0;i<3;i++){
            for(int k = 1;k<K-1;k++){
                String varName = "W_" + i+"_"+ k;
                W[i][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
            }
        }

        this.w = new GRBVar[O][K];
        for(int o = 0;o<O;o++){
            for(int k = 1;k<K-1;k++){
                String varName = "w_" + o+"_"+ k;
                w[o][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
            }
        }

        this.v = new GRBVar[O][K];
        for(int o = 0;o<O;o++){
            for(int k = 1;k<K-1;k++){
                String varName = "v_" + o+"_"+k;
                v[o][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
            }
        }

        /* CONSTRAINTS */
        // y_sk在sku第一次来之前必然为0
//        for(int s:this.skuSet){
//            int idx = this.skuComeSeq.indexOf(s);
//            for(int k = 0;k<idx;k++){
//                GRBLinExpr expr1 = new GRBLinExpr();
//                expr1.addTerm(1.0,y[s][k]);
//                this.Model.addConstr(expr1, GRB.EQUAL , 0, "sku scheduling constraints: sku cannot leave before coming");
//            }
//        }
        // 如果sku来多次，则至少有一个x_k为1
//        for(int s:this.skuSet){
//            if(this.skuUseNumMap.get(s)>1){
//                GRBLinExpr expr1 = new GRBLinExpr();
//                for(int k = 0;k<skuComeSeq.size();k++){
//                    if(this.skuComeSeq.get(k)==s){
//                        expr1.addTerm(1.0,x[k]);
//                    }
//                }
//                this.Model.addConstr(expr1, GRB.GREATER_EQUAL, 1, "sku scheduling constraint: sku must come at least once");
//            }
//        }


        // SKU scheduling constraints
        for (int k = 0;k<K;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int s:this.skuSet){
                expr1.addTerm(1.0,y[s][k]);
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "sku scheduling constraint: each leaving sequence can only be assigned to one sku");
        }

        for(int k = instance.toteCapByStation;k<this.skuComeSeq.size();k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            int fixedKNum = 0;
            for (int kk = 0; kk < k; kk++) {
                if(fixedX[kk] ==0) {
                    expr1.addTerm(1.0, x[kk]);
                }else{
                    fixedKNum+=1;
                }
                for(int s:this.skuSet) {
                    expr1.addTerm(-1.0, y[s][kk]);
                }
            }
            if(fixedX[k] ==0) {
                expr1.addTerm(1.0, x[k]);
            }else{
                fixedKNum+=1;
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, instance.toteCapByStation-fixedKNum, "sku scheduling constraint: max tote space in the pick station");
        }


        for(int s:skuSet){
            GRBLinExpr expr1 = new GRBLinExpr();
            int fixedKNum = 0;
            for(int k = 0;k<this.skuComeSeq.size();k++){
                if(this.skuComeSeq.get(k) == s){
                    if(fixedX[k] == 0) {
                        expr1.addTerm(1.0, x[k]);
                    }else{
                        fixedKNum +=1;
                    }
                }
            }
            for(int k = 0;k<K;k++){
                expr1.addTerm(-1.0,y[s][k]);
            }
            this.Model.addConstr(expr1, GRB.EQUAL, -fixedKNum, "tote scheduling constraint: each tote should leave the station once it comes");
        }


        for(int s:this.skuSet){
            for(int k = 0;k<K;k++){
                GRBLinExpr expr1 = new GRBLinExpr();
                int fixedKNum = 0;
                for(int kk = 0;kk<=k;kk++) {
                    if(kk<this.skuComeSeq.size()) {
                        if(this.skuComeSeq.get(kk) == s) {
                            if(fixedX[kk] == 0) {
                                expr1.addTerm(-1.0, x[kk]);
                            }else{
                                fixedKNum += 1;
                            }
                        }
                    }
                    expr1.addTerm(1.0, y[s][kk]);
                }
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, fixedKNum, "sku scheduling constraint: the sku can only leave the station if its in the station now");
            }
        }

        /* Order sequencing constraints */
        for(int o=0;o<O;o++){
            for(int s:instance.skuSetByOrder[o]) {
                for (int k = 1; k < K; k++) {
                    GRBLinExpr expr1 = new GRBLinExpr();

                    int fixedKNum = 0;
                    expr1.addTerm(2.0, gamma[s][o][k]);

                    for (int kk = 0; kk < k; kk++) {
                        if(kk<this.skuComeSeq.size()){
                            if(this.skuComeSeq.get(kk) == s){
                                if(fixedX[kk] == 0) {
                                    expr1.addTerm(-1.0, x[kk]);
                                }else{
                                    fixedKNum+=1;
                                }
                            }
                        }
                        expr1.addTerm(1.0, y[s][kk]);
                    }
                    if(k<this.skuComeSeq.size()) {
                        if (this.skuComeSeq.get(k) == s) {
                            if(fixedX[k] == 0) {
                                expr1.addTerm(-1.0, x[k]);
                            }else{
                                fixedKNum+=1;
                            }
                        }
                    }
                    expr1.addTerm(-1.0, alpha[o][k]);
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, fixedKNum, "Order sequencing constraint: required sku can be picked only when the tote is serving the station and order is open");

                    GRBLinExpr expr2 = new GRBLinExpr();
                    expr2.addTerm(1.0, gamma[s][o][k]);
                    expr2.addTerm(-1.0, alpha[o][k]);
                    this.Model.addConstr(expr2, GRB.LESS_EQUAL, 0, "Order sequencing constraint: required sku can be picked only when the order is open");
                }
            }
        }

        for(int o=0;o<O;o++) {
            for (int s : instance.skuSetByOrder[o]) {
                GRBLinExpr expr1 = new GRBLinExpr();
                int fixedKNum = 0;
                expr1.addTerm(2.0, gamma[s][o][0]);
                if(this.skuComeSeq.get(0) ==s){
                    if(fixedX[0] == 0) {
                        expr1.addTerm(-1.0, x[0]);
                    }else{
                        fixedKNum+=1;
                    }
                }
                expr1.addTerm(-1.0, alpha[o][0]);
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, fixedKNum, "Order sequencing constraint: required sku can be picked only when the tote is serving the station 2");
            }
        }

        for(int o=0;o<O;o++) {
            for (int s : instance.skuSetByOrder[o]) {
                GRBLinExpr expr1 = new GRBLinExpr();
                for (int k = 0; k < K; k++) {
                    expr1.addTerm(1.0, gamma[s][o][k]);
                }
                this.Model.addConstr(expr1, GRB.EQUAL, 1, "Order sequencing constraint: each required sku should be picked");
            }
        }

        for(int k = 0;k<K;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int o=0;o<O;o++){
                expr1.addTerm(1.0, alpha[o][k]);
                expr1.addTerm(-1.0, beta[o][k]);
            }
            expr1.addTerm(-1.0, U[k]);
            this.Model.addConstr(expr1, GRB.LESS_EQUAL,instance.orderBinCap-1, "Order sequencing constraint: max order picking capacity");
        }

        for(int o = 0;o<O;o++){
            for(int k=0;k<K;k++){
                if(instance.skuSetByOrder[o].size()==1){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0, alpha[o][k]);
                    expr1.addTerm(-1.0, beta[o][k]);
                    this.Model.addConstr(expr1, GRB.EQUAL,0, "Order sequencing constraint: orders with only one sku demand should be selected in one sequence");
                }
            }
        }


        for(int o = 0;o<O;o++){
            for(int k=0;k<K;k++){
                if(instance.skuSetByOrder[o].size()==1){
                    for(int s:instance.skuSetByOrder[o]){
                        GRBLinExpr expr1 = new GRBLinExpr();
                        expr1.addTerm(1.0, alpha[o][k]);
                        expr1.addTerm(-1.0, gamma[s][o][k]);
                        this.Model.addConstr(expr1, GRB.EQUAL,0, "Order sequencing constraint: orders with only one sku demand should be selected in one sequence");
                        GRBLinExpr expr2 = new GRBLinExpr();
                        expr2.addTerm(1.0, beta[o][k]);
                        expr2.addTerm(-1.0, gamma[s][o][k]);
                        this.Model.addConstr(expr2, GRB.EQUAL,0, "Order sequencing constraint: orders with only one sku demand should be selected in one sequence");
                    }
                }
            }
        }

        for(int o = 0;o<O;o++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int k = 0;k<K;k++){
                expr1.addTerm(1.0, beta[o][k]);
            }
            this.Model.addConstr(expr1, GRB.EQUAL,1, "Order sequencing constraint: every order must be completed");
        }

        for(int o=0;o<O;o++){
            if(instance.skuSetByOrder[o].size()>1){
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0, beta[o][0]);
                for(int k=1;k<K;k++){
                    for(int s:this.instance.skuSetByOrder[o]){
                        expr1.addTerm(1.0, gamma[s][o][k]);
                    }
                }
                this.Model.addConstr(expr1, GRB.GREATER_EQUAL,1, "Order sequencing constraint: an order is closed during the first picking turn if no required SKU of that order is selected in the future");
            }
        }

        for(int o=0;o<O;o++){
            if(instance.skuSetByOrder[o].size()>1){
                for(int k=1;k<K-1;k++) {
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0, beta[o][k]);
                    for (int kk = 0; kk < k; kk++) {
                        expr1.addTerm(1.0, beta[o][kk]);
                    }
                    for (int s :this.instance.skuSetByOrder[o]) {
                        for (int kk = k + 1; kk < K; kk++) {
                            expr1.addTerm(1.0, gamma[s][o][kk]);
                        }
                    }
                    this.Model.addConstr(expr1, GRB.GREATER_EQUAL, 1, "Order sequencing constraint: an open order is closed during the k′th picking turn if no required SKU of that order is selected in any future picking turn");
                }
            }
        }

        for(int o=0;o<O;o++){
            if(instance.skuSetByOrder[o].size()>1){
                for(int k=0;k<K;k++){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0, alpha[o][k]);
                    for(int s:this.instance.skuSetByOrder[o]){
                        for(int kk=0;kk<=k;kk++){
                            expr1.addTerm(-1.0, gamma[s][o][kk]);
                        }
                    }
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "Order sequencing constraint: ensures that the order cannot be opened if there is no required SKU ever been selected");
                }
            }
        }


        for(int o=0;o<O;o++){
            if(instance.skuSetByOrder[o].size()>1){
                for(int k=1;k<K;k++){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0, alpha[o][k]);
                    for(int kk=0;kk<k;kk++) {
                        expr1.addTerm(1.0, beta[o][kk]);
                    }
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "Order sequencing constraint: once the order has been closed it cannot be open again");
                }
            }
        }

        for(int o=0;o<O;o++){
            if(instance.skuSetByOrder[o].size()>1){
                for(int k=0;k<K-1;k++){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0, alpha[o][k]);
                    expr1.addTerm(-1.0, beta[o][k]);
                    expr1.addTerm(-1.0, alpha[o][k+1]);
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "Order sequencing constraint: once the order starts selection the order stays open until it is closed.\n");
                }
            }
        }

        //Constraints to specify different work bin conditions//
        for(int k=1;k<K-1;k++){
            for(int c= 0;c<3;c++){
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0, W[c][k]);
                expr1.addTerm(-1.0,U[k]);
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "Order sequencing constraint:  Uk = 1 if one or more of the three above conditions is satisfied");
            }
        }

        for(int k=1;k<K-1;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            expr1.addTerm(1.0, U[k]);
            for(int c= 0;c<3;c++){
                expr1.addTerm(-1.0,W[c][k]);
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "Order sequencing constraint:  Uk = 1 if one or more of the three above conditions is satisfied");
        }

        for(int k=1;k<K-1;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            expr1.addTerm(1, W[0][k]);
            for(int o= 0;o<O;o++){
                expr1.addTerm(1.0,alpha[o][k-1]);
                expr1.addTerm(-1.0,beta[o][k-1]);
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, instance.orderBinCap, "Order sequencing constraint: wik = 1 order i is open when passing from the (k − 1)′th picking turn to the k′th picking turn and closed with that turn.");
        }

        for(int k=1;k<K-1;k++){
            for(int o = 0;o<O;o++){
                if(instance.skuSetByOrder[o].size()>1){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0,alpha[o][k-1]);
                    expr1.addTerm(1.0,beta[o][k]);
                    expr1.addTerm(-1.0,w[o][k]);
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "Order sequencing constraint: wik = 1 order i is open when passing from the (k − 1)′th picking turn to the k′th picking turn and closed with that turn");
                }
            }
        }

        for(int k=1;k<K-1;k++){
            for(int o = 0;o<O;o++){
                if(instance.skuSetByOrder[o].size()>1){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(-1.0,alpha[o][k-1]);
                    expr1.addTerm(1.0,w[o][k]);
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "Order sequencing constraint: wik = 1 order i is open when passing from the (k − 1)′th picking turn to the k′th picking turn and closed with that turn");
                }
            }
        }

        for(int k=1;k<K-1;k++){
            for(int o = 0;o<O;o++){
                if(instance.skuSetByOrder[o].size()>1){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(-1.0,beta[o][k]);
                    expr1.addTerm(1.0,w[o][k]);
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "Order sequencing constraint: wik = 1 order i is open when passing from the (k − 1)′th picking turn to the k′th picking turn and closed with that turn");
                }
            }
        }

        for(int k=1;k<K-1;k++){
            for(int o = 0;o<O;o++){
                if(instance.skuSetByOrder[o].size()>1){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(-1.0,W[1][k]);
                    expr1.addTerm(1.0,w[o][k]);
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "Order sequencing constraint: W2k = 1 if there is at least one order satisfying wik = 1, zero otherwise");
                }
            }
        }


        for(int k=1;k<K-1;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            expr1.addTerm(-1.0,W[1][k]);
            for(int o = 0;o<O;o++){
                if(instance.skuSetByOrder[o].size()>1){
                    expr1.addTerm(1.0,w[o][k]);
                }
            }
            this.Model.addConstr(expr1, GRB.GREATER_EQUAL, 0, "Order sequencing constraint: W2k = 1 if there is at least one order satisfying wik = 1, zero otherwise");
        }


        for(int k=1;k<K-1;k++){
            for(int o = 0;o<O;o++){
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0,v[o][k]);
                expr1.addTerm(-1.0,beta[o][k]);
                expr1.addTerm(1.0,alpha[o][k-1]);
                this.Model.addConstr(expr1, GRB.GREATER_EQUAL, 0, "Order sequencing constraint: ensure that vik = 1 order i is not opened and closed within the k′th picking turn.");
            }
        }
        for(int k=1;k<K-1;k++){
            for(int o = 0;o<O;o++){
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0,v[o][k]);
                expr1.addTerm(1.0,alpha[o][k-1]);
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "Order sequencing constraint: ensure that vik = 1 order i is not opened and closed within the k′th picking turn.");
            }
        }
        for(int k=1;k<K-1;k++){
            for(int o = 0;o<O;o++){
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0,v[o][k]);
                expr1.addTerm(-1.0,beta[o][k]);
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "Order sequencing constraint: ensure that vik = 1 order i is not opened and closed within the k′th picking turn.");
            }
        }

        for(int k=1;k<K-1;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            expr1.addTerm(1.0,W[2][k]);
            for(int o = 0;o<O;o++){
                expr1.addTerm(1.0,v[o][k]);
            }
            this.Model.addConstr(expr1, GRB.GREATER_EQUAL, 1, "Order sequencing constraint: ensure that W3k = 1 if there is no order satisfying wik = 1");
        }


        for(int k=1;k<K-1;k++){
            for(int o = 0;o<O;o++) {
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0, W[2][k]);
                expr1.addTerm(1.0, v[o][k]);
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "Order sequencing constraint: ensure that W3k = 1 if there is no order satisfying wik = 1");
            }
        }

        GRBLinExpr expr1 = new GRBLinExpr();
        expr1.addTerm(1.0, U[0]);
        this.Model.addConstr(expr1, GRB.EQUAL, 1, "Order sequencing constraint: all MO work bins are empty in the first and last picking turn");

        GRBLinExpr expr2 = new GRBLinExpr();
        expr2.addTerm(1.0, U[K-1]);
        this.Model.addConstr(expr2, GRB.EQUAL, 1, "Order sequencing constraint: all MO work bins are empty in the first and last picking turn");

    }

    public void orderPickingRules(int[][] orderSeq, List<Integer> skuLeaveSeq) throws GRBException {
        /*
        添加规则/
         */
        // ---- kappa_o: the open time of order ----
        this.kappa =new GRBVar[O];
        for(int o =0;o<O;o++){
            String varName = "kappa_" + o;
            kappa[o] = this.Model.addVar(0, K, 0, GRB.CONTINUOUS, varName);
        }

        /* 订单第一次到达相关约束 */
        for(int o=0;o<O;o++){
            for(int k = 1;k<K;k++){
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0, kappa[o]);
                expr1.addTerm(-k, alpha[o][k]);
                expr1.addTerm(k, alpha[o][k-1]);
                this.Model.addConstr(expr1, GRB.GREATER_EQUAL, 0, "order scheduling constraint: cal the first turn k when order o is open");
            }
        }

        for(int o =0;o<O;o++){
            for(int k = 0;k<K;k++){
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0, kappa[o]);
                expr1.addTerm(K-k, alpha[o][k]);
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, K, "order scheduling constraint: cal the first turn k when order o is open");
            }
        }
        // 添加约束：订单打开的时间必须<= 所需sku的最晚离开时间
        for(int o =0;o<O;o++){
            for(int s:instance.skuSetByOrder[o]){
                // 如果该sku只来一次
                if(skuUseNumMap.get(s) == 1){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0, kappa[o]);
                    for(int k = 0;k<K;k++) {
                        expr1.addTerm(-k, y[s][k]);
                    }
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "order scheduling constraint: order should be open before the closeness of needed sku");
                }
                else{
                    // 如果sku会来多次
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0, kappa[o]);
                    for(int k = 0;k<K;k++) {
                        expr1.addTerm(-k, hat_y[s][k]);
                    }
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "order scheduling constraint: order should be open before the closeness of needed sku");
                }
            }
        }
        // 要求只包含非重复访问sku的订单固定打开的相对顺序
        List<Integer> ordersWithSKUsVisitOnce = new ArrayList<>();
        for(int o=0;o<instance.orderNum;o++){
            boolean allSKUVisitOnce = true;
            for(int s:instance.skuSetByOrder[o]){
                if(this.skuUseNumMap.get(s) >1){
                    allSKUVisitOnce = false;
                    break;
                }
            }
            if(allSKUVisitOnce) {
                ordersWithSKUsVisitOnce.add(o);
            }
        }
        for(int o1:ordersWithSKUsVisitOnce){
            for(int o2:ordersWithSKUsVisitOnce){
                if(o1<o2){
                    GRBLinExpr expr = new GRBLinExpr();
                    expr.addTerm(1.0 , kappa[o1]);
                    expr.addTerm(-1.0 , kappa[o2]);
                    if(orderSeq[o1][0] <= orderSeq[o2][0]){
                        this.Model.addConstr(expr, GRB.LESS_EQUAL, 0, "保持订单处理相对顺序");
                    }else{
                        this.Model.addConstr(expr, GRB.GREATER_EQUAL, 0, "保持订单处理相对顺序");

                    }
                }
            }
        }


        // 寻找和重复访问sku相关的集合
        int[] lastKofCorrSKU = new int[instance.skuNum];
        Set<Integer> skuSetRelatedToRevisit = new HashSet<>();
        for(int s:skuSet){
            lastKofCorrSKU[s] = skuComeSeq.lastIndexOf(s);
        }
        for(int s:skuSet){
            if(this.skuUseNumMap.get(s)>1){
                for(int o:instance.orderSetBySKU[s]){
                    skuSetRelatedToRevisit.addAll(instance.skuSetByOrder[o]);
                    for(int ss:instance.skuSetByOrder[o]){
                        lastKofCorrSKU[ss] = Math.max(skuComeSeq.lastIndexOf(s), lastKofCorrSKU[ss]);
                    }
                }
            }
        }

        // 离开顺序在原离开顺序前后2个范围内
        int leave_k;
        for(int s:skuSet){
            if(!skuSetRelatedToRevisit.contains(s)){
                if(skuLeaveSeq.contains(s)) {
                    leave_k = skuLeaveSeq.indexOf(s) + instance.toteCapByStation - 1;
                }else{
                    leave_k = skuLeaveSeq.size() + instance.toteCapByStation - 1;
                }
                GRBLinExpr expr = new GRBLinExpr();

                for(int k = Math.max(0, leave_k- AlgorithmParams.delta_k_minus); k<Math.min(leave_k+AlgorithmParams.delta_k_add, K); k++){
                    expr.addTerm(1.0, y[s][k]);
                }
                this.Model.addConstr(expr, GRB.GREATER_EQUAL, 1, "在原离开顺序的前后3单位内离开");
            }else if(this.skuUseNumMap.get(s)==1){
                if(skuLeaveSeq.contains(s)) {
                    leave_k = skuLeaveSeq.indexOf(s) + instance.toteCapByStation - 1;
                }else{
                    leave_k = skuLeaveSeq.size() + instance.toteCapByStation - 1;
                }
                GRBLinExpr expr = new GRBLinExpr();
                for (int k = Math.max(0, leave_k); k < Math.min(leave_k+5, K); k++) {
                    expr.addTerm(1.0, y[s][k]);
                }
                this.Model.addConstr(expr, GRB.GREATER_EQUAL, 1, "在原离开顺序的前后5单位内离开");
            }
        }
    }

    public void generate_hat_y(List<Integer> skuLeaveSeq) throws  GRBException{
                /*
        在原模型基础上添加有效不等式加速子问题求解
         */

        // ----- hat_y_sk: the last leave sequence of sku s
        this.hat_y = new GRBVar[S][K];
        for(int s:this.skuSet){
            if(skuUseNumMap.get(s)>1){
                for(int k = 0;k<K;k++){
                    String varName = "hat_y_"+s+"_"+k;
                    hat_y[s][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
                    if(k == skuLeaveSeq.lastIndexOf(s) + instance.toteCapByStation-1){
                        hat_y[s][k].set(GRB.DoubleAttr.Start, 1.0);
                    }else{
                        hat_y[s][k].set(GRB.DoubleAttr.Start, 0.0);
                    }
                }
            }
        }


        /* sku的最后一次离开相关约束 */
        for(int s:this.skuSet){
            if(skuUseNumMap.get(s) >1){
                GRBLinExpr expr1 = new GRBLinExpr();
                for(int k = 0;k<K;k++) {
                    expr1.addTerm(1, hat_y[s][k]);
                }
                this.Model.addConstr(expr1, GRB.EQUAL, 1, "sku last leave sequence constraint: the last leave sequence can only be assigned to one k");
            }
        }

        for(int s:this.skuSet){
            if(skuUseNumMap.get(s) >1){
                for(int k = 0;k<K;k++) {
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1, hat_y[s][k]);
                    expr1.addTerm(-1, y[s][k]);
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "sku last leave sequence constraint: the last leave seq cant be larget than the max k");
                }
            }
        }

        for(int s:this.skuSet){
            if(skuUseNumMap.get(s) >1){
                GRBLinExpr expr1 = new GRBLinExpr();
                for(int k = 0;k<K;k++) {
                    for(int kk = 0;kk<K;kk++) {
                        expr1.addTerm(kk, hat_y[s][kk]);
                    }
                    expr1.addTerm(-k, y[s][k]);
                }
                this.Model.addConstr(expr1, GRB.GREATER_EQUAL, 0, "sku last leave sequence constraint: the last leave seq must be the max k");
            }
        }

    }

    public void orderPickingCut(List<Integer> skuLeaveSeq) throws GRBException {


        // ---- alpha_ok 和 y_sk 的关系 ----
        for(int s:this.skuSet){
            if(this.skuUseNumMap.get(s) == 1){ // sku只访问一次
                for(int o:instance.orderSetBySKU[s]) {
                    if(instance.skuSetByOrder[o].size()==1){
                        continue;
                    }
                    if(s != lastComeSKUofOrder[o]) {// sku不是订单所需sku中最后一个访问拣选站的
//                        System.out.println("sku="+s+", order="+o+" with open K = " +orderSeq[o][0]+ ", leave K="+orderSeq[o][1]+", and leave time of sku="+ (skuLeaveSeq.indexOf(s)+instance.toteCapByStation-1));
                        for(int k = 0; k< earliestCloseKofOrder[o]; k++) {
                            GRBLinExpr expr = new GRBLinExpr();
                            expr.addTerm(1, y[s][k]);
                            expr.addTerm(-1, alpha[o][k]);
                            this.Model.addConstr(expr, GRB.LESS_EQUAL, 0, "当sku s离开拣选站时，订单o必须保持打开状态");
                            expr = new GRBLinExpr();
                            expr.addTerm(1, y[s][k]);
                            for(int kk = this.skuComeSeq.indexOf(s);kk<=k;kk++){
                                expr.addTerm(-1, gamma[s][o][kk]);
                            }
                            this.Model.addConstr(expr, GRB.LESS_EQUAL, 0, "当sku s离开拣选站时，订单o内的s必然拣选完毕");

                            for(int kk = k+1;kk<K;kk++){
                                expr = new GRBLinExpr();
                                expr.addTerm(1, y[s][k]);
                                expr.addTerm(1, gamma[s][o][kk]);
                                this.Model.addConstr(expr, GRB.LESS_EQUAL, 1, "sku s最终离开拣选站以后，就不能再被拣选了");
                            }
                        }
                    }
                }
            }
        }

        for(int o=0;o<O;o++){
            for(int k = 0; k<earliestCloseKofOrder[o]; k++){
                GRBLinExpr expr = new GRBLinExpr();
                expr.addTerm(1.0, alpha[o][k+1]);
                expr.addTerm(-1.0, alpha[o][k]);
                this.Model.addConstr(expr, GRB.GREATER_EQUAL, 0, "订单在最早离开拣选站时间之前，一旦处于打开状态，就必须持续处于打开状态");
            }
        }

        // 在最早关闭时间之前beta_ok都取0
        for(int o=0;o<O;o++){
            for(int k = 0; k< earliestCloseKofOrder[o]; k++){
                GRBLinExpr expr = new GRBLinExpr();
                expr.addTerm(1.0, beta[o][k]);
                this.Model.addConstr(expr, GRB.EQUAL, 0, "订单在最后一个所需SKU到达之前不能关闭");
            }
        }



        // 在最早sku到达之前，alpha_ok都取0
        for(int o=0;o<O;o++){
            for(int k= 0;k<firstOpenKofOrder[o];k++){
                GRBLinExpr expr = new GRBLinExpr();
                expr.addTerm(1.0, alpha[o][k]);
                this.Model.addConstr(expr, GRB.EQUAL, 0, "最优解情况下，订单在第一个SKU到达之前不会打开");
            }
        }

        for(int o=0;o<O;o++){
            for(int s:instance.skuSetByOrder[o]) {
                for (int k = 0; k < firstOpenKofSKU[s]; k++) {
                    GRBLinExpr expr = new GRBLinExpr();
                    expr.addTerm(1.0, gamma[s][o][k]);
                    this.Model.addConstr(expr, GRB.EQUAL, 0, "订单在所需SKU第一次到达之前不会有任何拣选");
                }
            }
        }

        // 对于sku需求数量>toteCap的订单，必然在最后一个SKU到达时(要求没有SKU存在重复访问），完成拣选
        for(int o=0;o<O;o++){
            if(instance.skuSetByOrder[o].size() > instance.toteCapByStation){
                boolean is_fixed_close = true;
                for(int s:instance.skuSetByOrder[o]){
                    if(this.skuUseNumMap.get(s) > 1){
                        is_fixed_close = false;
                        break;
                    }
                }
                if(is_fixed_close){

                    GRBLinExpr expr = new GRBLinExpr();
                    expr.addTerm(1.0, beta[o][earliestCloseKofOrder[o]]);
                    this.Model.addConstr(expr, GRB.GREATER_EQUAL, 1, "对于sku需求数量大于tote容量的订单，且不存在重复sku访问的情况下，必然在最后一个sku到达时完成拣选");
                }
            }
        }

//      // 对于sku需求数量>toteCap的订单，必然在最后一个SKU到达时或之前(考虑SKU的重复访问）就完成拣选
        for(int o=0;o<O;o++){
            if(instance.skuSetByOrder[o].size() > instance.toteCapByStation){

                for(int k = lastSKUVisitKofOrder[o]+1; k<K; k++){
                    GRBLinExpr expr = new GRBLinExpr();
                    expr.addTerm(1.0, beta[o][k]);
                    this.Model.addConstr(expr, GRB.LESS_EQUAL, 0, "对于sku需求数量大于toteCap的订单而言，必然在最后一个SKU到达时或之前(考虑SKU的重复访问）就完成拣选");
                }
            }
            else{
                // 对于需求数量<=toteCap的订单，只能是以下两种情况之一：
                // 1. 在最后一个SKU到达时或之前(考虑SKU的重复访问）就完成拣选 2. 在最后一个SKU到达之前(考虑SKU的重复访问）都没有打开过
                GRBLinExpr expr = new GRBLinExpr();
                for(int k = lastSKUVisitKofOrder[o]+1; k<K; k++){
                    expr.addTerm(1.0, beta[o][k]);
                }
                for(int k=0;k<lastSKUVisitKofOrder[o];k++){
                    expr.addTerm(1.0/(double)lastSKUVisitKofOrder[o], alpha[o][k]);
                }
                this.Model.addConstr(expr, GRB.LESS_EQUAL, 1, "对于需求数量<=toteCap的订单，只能是以上两种情况之一");
            }
        }

        // 对于一个SKU而言，如果它访问的最后一次序，是某些订单所需的SKU中最后一个访问次序，则可以对其进行枚举
        for(int s:ordersLastRequiringSKU.keySet()){
            List<Integer> orderList = ordersLastRequiringSKU.get(s);
            if(orderList.size()>1){
                for(int idx1 = 0;idx1<orderList.size();idx1++){
                    int o1 = orderList.get(idx1);
                    Set<Integer> sku_set_1 = instance.skuSetByOrder[o1];
                    if(sku_set_1.size()>instance.toteCapByStation){
                        continue;
                    }
                    for(int idx2 = idx1+1;idx2<orderList.size();idx2++){
                        int o2 = orderList.get(idx2);
                        Set<Integer> sku_set_2 = instance.skuSetByOrder[o2];
                        if(sku_set_2.size()>instance.toteCapByStation){
                            continue;
                        }
                        if(sku_set_2.size()+sku_set_1.size()>instance.toteCapByStation){
                            Set<Integer> union_set = new HashSet<>();
                            union_set.addAll(sku_set_1);
                            union_set.addAll(sku_set_2);
                            if(union_set.size()>instance.toteCapByStation){
                                GRBLinExpr expr = new GRBLinExpr();
                                for(int k = lastSKUVisitKofOrder[o1]+1; k<K; k++) {
                                    expr.addTerm(1.0, beta[o1][k]);
                                    expr.addTerm(1.0, beta[o2][k]);
                                }
                                this.Model.addConstr(expr, GRB.LESS_EQUAL, 1, "对于最后一个访问SKU，两两枚举所需该SKU的订单，检查是否合并时，超过toteCap，此时其中必然有一个订单需要在该次序之前or当前次序完成拣选");
                            }
                        }
                    }
                }
            }
        }

        // 针对每toteCap个SKU，在下一个SKU到来时，必然有某个SKU需要离开拣选站，此时，该SKU对应的订单需求（if该SKU只访问一次），必须已经满足，即该订单在下一个k次序要么处于打开状态，要么已经完成拣选
        int cur_k = 0;
        List<Integer> subSKUList = new ArrayList<>();
        Set<Integer> orderSetBySubSKUs;
        int min_order_num;
        while(cur_k < this.skuComeSeq.size()){
            int s = this.skuComeSeq.get(cur_k);
            if(this.skuUseNumMap.get(s) > 1){
                cur_k++;
                continue;
            }
            subSKUList.add(s);
            if(subSKUList.size()==instance.toteCapByStation){
                // 添加约束
                orderSetBySubSKUs = new HashSet<>();
                min_order_num = instance.orderNum;
                for(int ss:subSKUList){
                    orderSetBySubSKUs.addAll(instance.orderSetBySKU[ss]);
                    min_order_num = Math.min(min_order_num, instance.orderSetBySKU[ss].size());
                }
                GRBLinExpr expr = new GRBLinExpr();
                for(int o:orderSetBySubSKUs){
                    expr.addTerm(1.0, alpha[o][cur_k+1]);
                    for(int kk = 0;kk<cur_k+1;kk++){
                        expr.addTerm(1.0, beta[o][kk]);
                    }
                }

                this.Model.addConstr(expr, GRB.GREATER_EQUAL, min_order_num, "对于集合subSKUList对应的订单集合，在新的SKU到来时，必然某个SKU离开，此时需要该SKU的订单必然处于完成拣选or仍打开状态");
                // 去掉第一个
                subSKUList.remove(0);
            }
            cur_k++;
        }

        // sku 如果在k之前根本没有出现过，则y_sk必须等于0
        for(int s:this.skuSet){
            int firstComeIdx = this.skuComeSeq.indexOf(s);
            for(int k = 0;k<firstComeIdx;k++){
                GRBLinExpr expr = new GRBLinExpr();
                expr.addTerm(1.0, y[s][k]);
                this.Model.addConstr(expr, GRB.EQUAL, 0, "对于在k之前必然不会到达的sku，也必然不会在k之前离开");
            }
        }

        // 对于在k之前无法完成拣选的订单，beta_ok都等于0
        for(int o=0;o<O;o++){
            for(int k = 0;k<earliestCloseKofOrder[o];k++){
                GRBLinExpr expr = new GRBLinExpr();
                expr.addTerm(1.0, beta[o][k]);
                this.Model.addConstr(expr, GRB.EQUAL, 0, "对于在k之前无法完成拣选的订单，beta_ok都等于0");
            }
        }
    }

    public void calLeastFinishedOrderNum() throws GRBException, IOException {
        // 计算在k次序的sku到达之前，至少需要完成多少个订单的拣选
        int[] leastFinishedOrderNum = new int[skuComeSeq.size()];
        Set<Integer> preOrderSet = new HashSet<>();
        for(int k = 0;k<this.instance.toteCapByStation;k++){
            int s = skuComeSeq.get(k);
            if(skuComeSeq.lastIndexOf(s) == k){
                preOrderSet.addAll(instance.orderSetBySKU[s]);
            }
        }
        for(int k = this.instance.toteCapByStation;k<skuComeSeq.size();k++){
            // 在第k个次序的sku到达之前，需要从前k个次序中，筛选出 k-toteCap 个sku 离开拣选站
            // 对于重复访问的sku，如果不是最后一次访问，则必须完成的订单集合为空集
            int s = skuComeSeq.get(k);
            if(skuComeSeq.lastIndexOf(s) == k){
                preOrderSet.addAll(instance.orderSetBySKU[s]);
            }
            SubModelLeastFinishONum subModelLeastFinishONum = new SubModelLeastFinishONum();
            subModelLeastFinishONum.buildModel(k, instance, preOrderSet, skuComeSeq);
            double leastOrderNum = subModelLeastFinishONum.solveMIPModel();
            leastFinishedOrderNum[k] = (int)leastOrderNum;
            // 添加约束
            GRBLinExpr expr = new GRBLinExpr();
            for(int o:preOrderSet){
                for(int kk=0;kk<k;kk++){
                    expr.addTerm(1.0, beta[o][kk]);
                }
            }
            this.Model.addConstr(expr, GRB.GREATER_EQUAL, leastOrderNum-instance.orderBinCap, "在k次序的sku到达之前，需要筛选出部分sku离开拣选站");
        }
    }


//    public void robotRoutingSubProb() throws GRBException{
//        this.z = new GRBVar[N+1][N+1][R];
//        for (int i:nodeList) {
//            for (int j:nodeList) {
//                for (int r = 0; r < R; r++) {
//                    String varName = "z_" + i + "_" + j + "_" + r;
//                    z[i][j][r] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
//                }
//            }
//        }
//
//        this.mu = new GRBVar[N][R];
//        for(int i:toteList) {
//            for (int r = 0; r < R; r++) {
//                String varName = "mu_" + i + "_" + r;
//                mu[i][r] = this.Model.addVar(0, instance.toteCapByRobot + 1, 0, GRB.CONTINUOUS, varName);
//            }
//        }
//
//        this.omega = new GRBVar[this.skuComeSeq.size()][R];
//        for(int k = 0;k<this.skuComeSeq.size();k++){
//            for(int r= 0;r<R;r++){
//                String varName = "omega"+k+"_"+r;
//                omega[k][r] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
//            }
//        }
//
//        // robot routing constraints
//        for(int i:toteList){
//            for(int r=0;r<R;r++) {
//                GRBLinExpr expr1 = new GRBLinExpr();
//                for (int j:nodeList) {
//                    expr1.addTerm(1.0, z[i][j][r]);
//                }
//                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "robot routing constraint: each tote can be visited by one tote or the pick station ");
//            }
//        }
//        for(int j:toteList){
//            for (int r= 0;r<R;r++) {
//                GRBLinExpr expr1 = new GRBLinExpr();
//                for (int i : nodeList) {
//                    expr1.addTerm(1.0, z[i][j][r]);
//                    expr1.addTerm(-1.0, z[j][i][r]);
//                }
//                this.Model.addConstr(expr1, GRB.EQUAL, 0, "robot routing constraint: each tote only visit one tote after visiting one site");
//            }
//        }
//        for(int r = 0;r<R;r++){
//            GRBLinExpr expr1 = new GRBLinExpr();
//            for(int i: nodeList){
//                for(int j : nodeList){
//                    expr1.addTerm(1.0,z[i][j][r]);
//                }
//            }
//            this.Model.addConstr(expr1, GRB.LESS_EQUAL, instance.toteCapByRobot+1, "robot routing constraint: tote capacity of each robot");
//        }
//
//        for (int i:toteList) {
//            for (int r= 0; r < R; r++) {
//                GRBLinExpr expr1 = new GRBLinExpr();
//                expr1.addTerm(1.0, z[i][i][r]);
//                this.Model.addConstr(expr1, GRB.EQUAL, 0, "robot routing constraint: no self visiting");
//            }
//        }
//
//        for (int r= 0; r < R; r++) {
//            GRBLinExpr expr1 = new GRBLinExpr();
//            for (int i:nodeList) {
//                expr1.addTerm(1.0,z[N][i][r]);
//            }
//            this.Model.addConstr(expr1, GRB.EQUAL, 1, "robot routing constraint: robot departs from the pick station");
//        }
//
//        for (int r= 0; r < R; r++) {
//            GRBLinExpr expr1 = new GRBLinExpr();
//            for (int i:nodeList) {
//                expr1.addTerm(1.0,z[i][N][r]);
//            }
//            this.Model.addConstr(expr1, GRB.EQUAL, 1, "robot routing constraint: robot returns to the pick station");
//        }
//        for(int i:toteList){
//            for(int j:toteList){
//                for(int r = 0;r<R;r++){
//                    GRBLinExpr expr1 = new GRBLinExpr();
//                    expr1.addTerm(1.0,mu[i][r]);
//                    expr1.addTerm(-1.0,mu[j][r]);
//                    expr1.addTerm(instance.toteCapByRobot+1,z[i][j][r]);
//                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, instance.toteCapByRobot, "robot routing constraint:eliminate cycle");
//                }
//            }
//        }
//        for(int i:toteList){
//            GRBLinExpr expr1 = new GRBLinExpr();
//            for(int j:nodeList){
//                for(int r= 0;r<R;r++){
//                    expr1.addTerm(1.0,z[i][j][r]);
//                }
//                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "robot routing constraint: each tote is used at most once");
//            }
//        }
//
//
//        // connection constraint //
//        for (int r = 0;r<R;r++){
//            for(int s:this.skuSet) {
//                GRBLinExpr expr1 = new GRBLinExpr();
//                for (int k = 0; k < this.skuComeSeq.size(); k++) {
//                    if(this.skuComeSeq.get(k) == s) {
//                        expr1.addTerm(1.0, omega[k][r]);
//                    }
//                }
//                for (int i : instance.toteSetBySKU[s]) {
//                    for (int j : nodeList) {
//                        expr1.addTerm(-1.0, z[i][j][r]);
//                    }
//                }
//                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "connection constraint: each tote should be planned in the route if it's scheduled");
//            }
//        }
//
//        for(int k = 0;k<this.skuComeSeq.size();k++){
//            GRBLinExpr expr1 = new GRBLinExpr();
//            for(int r = 0;r<R;r++){
//                expr1.addTerm(1.0, omega[k][r]);
//            }
//            expr1.addTerm(-1.0, x[k]);
//            this.Model.addConstr(expr1, GRB.EQUAL, 0, "connection constraint: the route num r of sequence kth sku is assigned to");
//        }
//
//        for(int k = 0;k<this.skuComeSeq.size()-1;k++){
//            for(int kk = k+1;kk<this.skuComeSeq.size();kk++) {
//                GRBLinExpr expr1 = new GRBLinExpr();
//                for (int r = 0; r < R; r++) {
//                    expr1.addTerm(r, omega[k][r]);
//                    expr1.addTerm(-r, omega[kk][r]);
//                }
//                expr1.addTerm(R, x[kk]);
//                this.Model.addConstr(expr1, GRB.LESS_EQUAL, R, "connection constraint: route sequence follows the sku come sequence");
//            }
//        }
//    }

//    public void addLBBDVarAndCut() throws GRBException {
//        String varName = "route length";
//        this.z = this.Model.addVar(0, Double.MAX_VALUE, 0, GRB.CONTINUOUS, varName);
//    }
    public void setObj() throws GRBException{
        /* OBJECTIVE FUNCTION */
        obj = new GRBLinExpr();
        for(int k = 0;k<this.skuComeSeq.size();k++){
            if(fixedX[k]==0) {
                obj.addTerm(instance.pickTime, x[k]);
            }else{
                obj.addConstant(instance.pickTime);
            }
        }
//        obj.addTerm(1/instance.moveSpeed, z);
//        for (int i:this.nodeList) {
//            for (int j:this.nodeList) {
//                for(int r= 0;r<R;r++){
//                    obj.addTerm(instance.disBetweenTotes[i][j]/instance.moveSpeed, z[i][j][r]);
//                }
//            }
//        }
        this.Model.setObjective(obj, GRB.MINIMIZE);
    }

    public void buildModel(Instance instance, List<Integer> skuComeSeq, HashMap<Integer, Integer> skuUseNumMap, List<Integer> skuLeaveSeq, int[][] orderSeq,int[][] pickK, int cut_type) throws IOException, GRBException {
        this.instance = instance; // this step is necessary

        O = instance.orderNum;
        S = instance.skuNum;
        K = skuComeSeq.size() + instance.toteCapByStation;
//        R = skuComeSeq.size()/instance.toteCapByRobot;
        N = instance.toteNum;

        this.skuComeSeq = skuComeSeq; //给定的skuComeSeq
        this.skuSet = new HashSet<>(this.skuComeSeq);
        this.env = new GRBEnv("MIP_model.log");
        this.Model = new GRBModel(this.env);
        this.skuUseNumMap = skuUseNumMap;
        this.toteList = new ArrayList<>();
        for(int s:skuUseNumMap.keySet()){
            toteList.addAll(instance.toteSetBySKU[s]);
        }
        this.nodeList = new ArrayList<>();
        this.nodeList.addAll(toteList);
        this.nodeList.add(N);


        // 计算每个订单最早关闭时间
        earliestCloseKofOrder = new int[O]; // order最早可以完成拣选的k
        lastComeSKUofOrder = new int[O];
        lastSKUVisitKofOrder = new int[O]; // order所需SKU中，最后一个到达的k（包括重复访问）
        firstOpenKofOrder = new int[O];
        firstOpenKofSKU= new int[S];
        ordersLastRequiringSKU = new HashMap<>();
        for(int s:skuSet){
            firstOpenKofSKU[s] = this.skuComeSeq.indexOf(s);
        }

        for(int o = 0;o<O;o++){
            int closeIndexK = -1;
            int skuLastIndex;
            int lastComeSKU = -1;
            int openIndexK = K;

            int skuLastIndexAll;
            int finalCloseK = -1;
            int lastComeSKUAll = -1;
            for(int s:instance.skuSetByOrder[o]) {
                // 无论是到达1次or多次的sku，都是其到达的第一次以后，订单才允许结束拣选
                skuLastIndex = this.skuComeSeq.indexOf(s);
                skuLastIndexAll = this.skuComeSeq.lastIndexOf(s);
                if (closeIndexK < skuLastIndex) {
                    closeIndexK = skuLastIndex;
                    lastComeSKU = s;
                }
                if(finalCloseK<skuLastIndexAll){
                    finalCloseK = skuLastIndexAll;
                    lastComeSKUAll = s;
                }
                openIndexK = Math.min(openIndexK, skuLastIndex);
            }
            earliestCloseKofOrder[o]= closeIndexK;
            lastComeSKUofOrder[o] = lastComeSKU;
            firstOpenKofOrder[o] = openIndexK;
            lastSKUVisitKofOrder[o] = finalCloseK;
            List<Integer> orderList = ordersLastRequiringSKU.getOrDefault(lastComeSKUAll, new ArrayList<>());
            orderList.add(o);
            ordersLastRequiringSKU.put(lastComeSKUAll, orderList);
        }



        /* OBJECTIVE FUNCTION */
        //minimize the robot routing distance
        orderPickingSubProb(skuLeaveSeq, orderSeq);
        if(cut_type == 1){
            generate_hat_y(skuLeaveSeq);
            orderPickingCut(skuLeaveSeq);
            orderPickingRules(orderSeq, skuLeaveSeq);
        }else if(cut_type == 2){
            generate_hat_y(skuLeaveSeq);
            orderPickingRules(orderSeq, skuLeaveSeq);
        }
        setObj();
    }

    public void setGurobiParam(double timeLimit) throws GRBException {
        // STOP CRITERIA
        this.Model.set(GRB.IntParam.OutputFlag, 1);
        this.Model.set(GRB.DoubleParam.TimeLimit, timeLimit);
        this.Model.set(GRB.IntParam.SubMIPNodes, 20000);
        this.Model.set(GRB.IntParam.MIPFocus, 1);

    }

    public List<List<Integer>> solveMIPModel(List<Integer> skuLeaveSeq, double timeLimit) throws GRBException, IOException {
        /* SOLVING */
        setGurobiParam(timeLimit);

        this.Model.optimize();

        if (this.Model.get(GRB.IntAttr.SolCount)>0) {
            this.objVal = this.Model.get(GRB.DoubleAttr.ObjVal);
            this.runTime = this.Model.get(GRB.DoubleAttr.Runtime);
            this.lowerBound = this.Model.get(GRB.DoubleAttr.ObjBound);
            this.toteVisitNum = 0;
            this.soluSKUComeSeq = new ArrayList<>();
            this.soluSKULeaveSeq = new ArrayList<>();
            this.soluOrderSeq = new int[instance.orderNum];
            List<List<Integer>> skuComeRoutes = new ArrayList<>();
            List<Integer> skuComeR = new ArrayList<>();

            for(int k = 0;k<this.skuComeSeq.size();k++){
                if(k%instance.toteCapByRobot == 0 & k!=0){
                    skuComeRoutes.add(skuComeR);
                    skuComeR = new ArrayList<>();
                }
                if(this.fixedX[k] ==1){
                    this.toteVisitNum+=1;
                    this.soluSKUComeSeq.add(this.skuComeSeq.get(k));
                    skuComeR.add(this.skuComeSeq.get(k));
                }else{
                    if(this.x[k].get(GRB.DoubleAttr.X)>0){
                        this.toteVisitNum++;
                        this.soluSKUComeSeq.add(this.skuComeSeq.get(k));
                        skuComeR.add(this.skuComeSeq.get(k));
                    }

                }
            }
            if(skuComeR.size()>0) {
                skuComeRoutes.add(skuComeR);
            }
            skuComeRoutes.add(this.soluSKUComeSeq);
            for(int k = 0;k<K;k++){
                for(int s:this.skuSet){
                    if(this.y[s][k].get(GRB.DoubleAttr.X)>0){
                        this.soluSKULeaveSeq.add(s);
                    }
                }
            }
            int orderIdx = 0;
            Set<Integer> openedOrderSet = new HashSet<>();
            List<Integer> orderOpenAtOneKList = new ArrayList<>();
            for(int k = 0;k<K;k++){
                for(int o=0;o<O;o++){
                    if(this.alpha[o][k].get(GRB.DoubleAttr.X)>0 && (!openedOrderSet.contains(o))){
                        openedOrderSet.add(o);
                        orderOpenAtOneKList.add(o);
                    }
                }
                if(orderOpenAtOneKList.size() == 1){
                    this.soluOrderSeq[orderIdx] = orderOpenAtOneKList.get(0);
                    orderIdx++;
                }else{
                    List<Integer> orderStaysForNextTurn = new ArrayList<>();
                    for(int o:orderOpenAtOneKList) {
                        if (this.beta[o][k].get(GRB.DoubleAttr.X) > 0){
                            this.soluOrderSeq[orderIdx] = o;
                            orderIdx++;
                        }else{
                            orderStaysForNextTurn.add(o);
                        }
                    }
                    for(int o:orderStaysForNextTurn){
                        this.soluOrderSeq[orderIdx] = o;
                        orderIdx++;
                    }
                }
                orderOpenAtOneKList.clear();
            }
            this.Model.dispose();
            this.env.dispose();
            return skuComeRoutes;
        }
        else {
//            this.Model.computeIIS();
//            this.Model.write("model.ilp");
//            System.out.println("Model not solved");
            this.objVal = -1;
            this.runTime = this.Model.get(GRB.DoubleAttr.Runtime);
            this.lowerBound = this.Model.get(GRB.DoubleAttr.ObjBound);
            return null;
        }
    }



}


