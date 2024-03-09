package model;

import gurobi.*;
import instance_generation.Instance;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/*
SubModel A
固定订单处理顺序
求解给定订单处理顺序下，最小化需要料箱数量的SKU到达和离开顺序
松弛不考虑机器人的路径
 */
public class SubModelSCSByOS {
    Instance instance;
    int[] orderSeq;
    GRBEnv env;
    public GRBModel Model;

    GRBVar[][] x;
    GRBVar[][] y;
    GRBVar[][] alpha;
    GRBVar[][] beta;
    GRBVar[][][] gamma;
    GRBVar[]                U; //auxiliary variable
    GRBVar[][]              W;
    GRBVar[][]              w;
    GRBVar[][]              v;
//    GRBVar[][]                kappa;
    GRBVar[]                kappa;
    int O,S,K;
    GRBLinExpr obj;

    public void orderPickingSubProb() throws GRBException {
        /* DECISION VARIABLES */
        /*
         * GRBVar addVar ( double lb, double ub, double obj, char type, String name )
         */
        this.x = new GRBVar[S][K];
        for(int s=0;s<S;s++){
            for(int k=0;k<K;k++){
                String varName = "x_" + s + "_" + k;
                x[s][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
            }
        }

        this.y = new GRBVar[S][K];
        for(int s=0;s<S;s++){
            for(int k=0;k<K;k++){
                String varName = "y_" + s + "_" + k;
                y[s][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
            }
        }

        this.alpha =new GRBVar[O][K];
        for(int o =0;o<O;o++){
            for(int k = 0;k<K;k++){
                String varName = "alpha_" + o + "_" + k;
                alpha[o][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
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
        for (int s= 0;s<S;s++){
            for(int o = 0;o<O;o++){
                for(int k = 0;k<K;k++){
                    String varName = "gamma_" + String.valueOf(s) + "_" + String.valueOf(o) + "_" + String.valueOf(k);
                    gamma[s][o][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
                }
            }
        }

        this.U = new GRBVar[K];
        for(int k=0;k<K;k++){
            String varName = "U_" + String.valueOf(k);
            U[k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
        }

        this.W = new GRBVar[3][K];
        for(int i = 0;i<3;i++){
            for(int k = 1;k<K-1;k++){
                String varName = "W_" + String.valueOf(i)+"_"+ String.valueOf(k);
                W[i][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
            }
        }

        this.w = new GRBVar[O][K];
        for(int o = 0;o<O;o++){
            for(int k = 1;k<K-1;k++){
                String varName = "w_" + String.valueOf(o)+"_"+ String.valueOf(k);
                w[o][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
            }
        }

        this.v = new GRBVar[O][K];
        for(int o = 0;o<O;o++){
            for(int k = 1;k<K-1;k++){
                String varName = "v_" + String.valueOf(o)+"_"+ String.valueOf(k);
                v[o][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
            }
        }

        this.kappa =new GRBVar[O];
        for(int o =0;o<O;o++){
            String varName = "kappa_" + o;
            kappa[o] = this.Model.addVar(0, K, 0, GRB.CONTINUOUS, varName);
        }
//        this.kappa =new GRBVar[O][K];
//        for(int o =0;o<O;o++){
//            for(int k = 0;k<K;k++){
//                String varName = "kappa_" + o+ "_" + k;
//                kappa[o][k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
//            }
//        }

        /* CONSTRAINTS */
        //SKU scheduling constraints
        for (int k = 0 ;k<K;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int s = 0;s<S;s++){
                expr1.addTerm(1.0,x[s][k]);
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "sku scheduling constraint: each visiting sequence can only be assigned to one sku");
        }


        for (int k = 0;k<K;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int s = 0;s<S;s++){
                expr1.addTerm(1.0,y[s][k]);
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "sku scheduling constraint: each leaving sequence can only be assigned to one sku");
        }

        for(int k = 1;k<K;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int s = 0;s<S;s++){
                expr1.addTerm(1.0,x[s][k]);
                expr1.addTerm(-1.0,x[s][k-1]);
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "sku scheduling constraint: sku visiting sequence should be connected");
        }

        for(int k = instance.toteCapByStation;k<K;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int s = 0; s<S;s++) {
                for (int kk = 0; kk < k; kk++) {
                    expr1.addTerm(1.0, x[s][kk]);
                    expr1.addTerm(-1.0, y[s][kk]);
                }
                expr1.addTerm(1.0, x[s][k]);
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, instance.toteCapByStation, "sku scheduling constraint: max tote space in the pick station");
        }


        for(int s = 0;s<S;s++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int k = 0;k<K;k++){
                expr1.addTerm(1.0,x[s][k]);
                expr1.addTerm(-1.0,y[s][k]);
            }
            this.Model.addConstr(expr1, GRB.EQUAL, 0, "tote scheduling constraint: each tote should leave the station once it comes");
        }


        for(int s = 0;s<S;s++){
            for(int k = 0;k<K;k++){
                GRBLinExpr expr1 = new GRBLinExpr();
                for(int kk = 0;kk<=k;kk++) {
                    expr1.addTerm(-1.0, x[s][kk]);
                    expr1.addTerm(1.0, y[s][kk]);
                }
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "sku scheduling constraint: the sku can only leave the station if its in the station now");
            }
        }



        // fixed order sequence given kappa be continuous
        for(int idx = 0;idx<O-1;idx++){
            GRBLinExpr expr1 = new GRBLinExpr();
            expr1.addTerm(1.0, kappa[orderSeq[idx]]);
            expr1.addTerm(-1.0, kappa[orderSeq[idx+1]]);
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "order scheduling constraint: fixed order sequence");
        }
        for(int o = 0;o<O;o++){
            for(int k = 1;k<K;k++){
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0, kappa[o]);
                expr1.addTerm(-k, alpha[o][k]);
                expr1.addTerm(k, alpha[o][k-1]);
                this.Model.addConstr(expr1, GRB.GREATER_EQUAL, 0, "order scheduling constraint: cal the first turn k when order o is open");
            }
        }
        for(int o = 0;o<O;o++){
            for(int k = 0;k<K;k++){
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0, kappa[o]);
                expr1.addTerm(K-k, alpha[o][k]);
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, K, "order scheduling constraint: cal the first turn k when order o is open");
            }
        }

        // kappa: integer
//        for(int idx = 0;idx<O-1;idx++){
//            GRBLinExpr expr1 = new GRBLinExpr();
//            for(int k = 0;k<K;k++) {
//                expr1.addTerm(k, kappa[orderSeq[idx]][k]);
//                expr1.addTerm(-k, kappa[orderSeq[idx + 1]][k]);
//            }
//            this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "order scheduling constraint: fixed order sequence");
//        }
//
//        for(int o = 0;o<O;o++){
//            for(int k = 0;k<K;k++){
//                GRBLinExpr expr1 = new GRBLinExpr();
//                for(int kk=0;kk<=k;kk++){
//                    expr1.addTerm(1.0, kappa[o][kk]);
//                    if(kk>0){
//                        expr1.addTerm(-1.0, beta[o][kk-1]);
//                    }
//                }
//                expr1.addTerm(-1, alpha[o][k]);
//                this.Model.addConstr(expr1, GRB.EQUAL, 0, "order scheduling constraint: cal the first turn k when order o is open");
//            }
//        }
//
//        for(int o = 0;o<O;o++){
//            GRBLinExpr expr1 = new GRBLinExpr();
//            for(int k = 0;k<K;k++){
//                expr1.addTerm(1.0, kappa[o][k]);
//            }
//            this.Model.addConstr(expr1, GRB.EQUAL, 1, "order scheduling constraint: cal the first turn k when order o is open");
//        }


        /* Order sequencing constraints */
        for(int o=0;o<O;o++){
            for(int s:instance.skuSetByOrder[o]) {
                for (int k = 1; k < K; k++) {
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(2.0, gamma[s][o][k]);
                    for (int kk = 0; kk < k; kk++) {
                        expr1.addTerm(-1.0, x[s][kk]);
                        expr1.addTerm(1.0, y[s][kk]);
                    }
                    expr1.addTerm(-1.0, x[s][k]);
                    expr1.addTerm(-1.0, alpha[o][k]);
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "Order sequencing constraint: required sku can be picked only when the tote is serving the station 1");
                }
            }
        }

        for(int o=0;o<O;o++) {
            for (int s : instance.skuSetByOrder[o]) {
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(2.0, gamma[s][o][0]);
                expr1.addTerm(-1.0, x[s][0]);
                expr1.addTerm(-1.0, alpha[o][0]);
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 0, "Order sequencing constraint: required sku can be picked only when the tote is serving the station 2");
            }
        }

        for(int o=0;o<O;o++) {
            for (int s : instance.skuSetByOrder[o]) {
                GRBLinExpr expr1 = new GRBLinExpr();
                for (int k = 0; k < K; k++) {
                    expr1.addTerm(1.0, gamma[s][o][k]);
                }
                this.Model.addConstr(expr1, GRB.GREATER_EQUAL, 1, "Order sequencing constraint: each required sku should be picked");
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
                    for(int s= 0;s<S;s++){
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
                    for (int s = 0; s < S; s++) {
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
                    for(int s= 0;s<S;s++){
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
            expr1.addTerm(-instance.orderBinCap, W[0][k]);
            for(int o= 0;o<O;o++){
                expr1.addTerm(-1.0,alpha[o][k-1]);
                expr1.addTerm(1.0,beta[o][k-1]);
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, -instance.orderBinCap, "Order sequencing constraint: wik = 1 order i is open when passing from the (k − 1)′th picking turn to the k′th picking turn and closed with that turn.");
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

    public void buildModel(Instance instance, int[] orderSeq) throws IOException, GRBException {
        this.instance = instance; // this step is necessary
        this.orderSeq = orderSeq;
        this.env = new GRBEnv("MIP_model.log");
        this.Model = new GRBModel(this.env);

        O = instance.orderNum;
        S = instance.skuNum;
        K = instance.toteMaxReqNum;

        /* OBJECTIVE FUNCTION */
        orderPickingSubProb();
        obj = new GRBLinExpr();

        for(int k = 0;k<K;k++){
            for(int s=0;s<S;s++){
                obj.addTerm(1.0, x[s][k]);
            }
        }
        this.Model.setObjective(obj, GRB.MINIMIZE);
    }

    public void setGurobiParam() throws GRBException {
        // STOP CRITERIA
//        this.Model.set(GRB.DoubleParam.TimeLimit, 3600);
        this.Model.set(GRB.IntParam.OutputFlag, 0);
    }

    public List<Integer> solveMIPModel() throws GRBException, IOException {
        /* SOLVING */
        setGurobiParam();
        this.Model.optimize();
        List<Integer> skuSeq = new ArrayList<>();

        if (this.Model.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL|| this.Model.get(GRB.IntAttr.Status) == GRB.Status.TIME_LIMIT) {

            for(int k=0;k<K;k++){
                for(int s=0;s<S;s++){
                    double xValue = this.x[s][k].get(GRB.DoubleAttr.X);
                    if (xValue > 0) {
                        skuSeq.add(s);
                    }
                }
            }
            this.Model.dispose();
            this.env.dispose();
        }
        else {
            this.Model.computeIIS();
            this.Model.write("model.ilp");
            System.out.println("Model not solved");
            // this.MIPModel.dispose();
            // this.env.dispose();
        }
        return skuSeq;
    }
}