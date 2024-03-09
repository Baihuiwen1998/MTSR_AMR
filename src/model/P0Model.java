package model;

import gurobi.*;
import instance_generation.Instance;
import support_func.Printer;

import java.io.IOException;

public class P0Model {
    Instance instance;
    GRBEnv env;
    GRBModel Model;
    GRBVar[][] x;
    GRBVar[][] y;
    GRBVar[][] alpha;
    GRBVar[][] beta;
    GRBVar[][][] gamma;
    GRBVar[]                U; //auxiliary variable
    GRBVar[][]              W;
    GRBVar[][]              w;
    GRBVar[][]              v;

    GRBVar[][]              l; // the load decision variable
    GRBVar[][][]         z; // the instance.robot routing decision variable
    GRBVar[]               mu; // the instance.robot routing eliminate subsidiary
    public double objVal = 0;
    public double runTime = 0;
    public double lowerBound = 0;
    public int toteVisitNum = 0;
    int O,S,N,K,R;
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

    public void robotRoutingSubProb() throws GRBException{
        this.z = new GRBVar[N+1][N+1][R];
        for (int i = 0; i < N+1; i++) {
            for (int j = 0; j < N+1; j++) {
                for (int r = 0;r<R;r++){
                    String varName = "z_" + i + "_" + j + "_" + r;
                    z[i][j][r] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
                }
            }
        }

        this.mu = new GRBVar[N];
        for(int i = 0;i<N;i++){
            String varName = "mu_" + i ;
            mu[i] = this.Model.addVar(0, instance.toteCapByRobot+1, 0, GRB.CONTINUOUS, varName);
        }

        this.l = new GRBVar[K][R];
        for(int k =0;k<K;k++){
            for(int r = k/instance.toteCapByRobot;r<R;r++){
                String varName = "l_"+k+"_"+r;
                l[k][r] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
            }
        }

        // robot routing constraints
        for(int i = 0;i<N;i++){
            for (int r= 0;r<R;r++) {
                GRBLinExpr expr1 = new GRBLinExpr();
                for (int j = 0; j < N+1; j++) {
                    expr1.addTerm(1.0, z[i][j][r]);
                }
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "robot routing constraint: each tote can be visited by one tote or the pick station ");
            }
        }
        for(int j = 0;j<N;j++){
            for (int r= 0;r<R;r++) {
                GRBLinExpr expr1 = new GRBLinExpr();
                for (int i = 0; i < N+1; i++) {
                    expr1.addTerm(1.0, z[i][j][r]);
                    expr1.addTerm(-1.0, z[j][i][r]);
                }
                this.Model.addConstr(expr1, GRB.EQUAL, 0, "robot routing constraint: each tote only visit one tote after visiting one site");
            }
        }
        for(int r = 0;r<R;r++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int i = 0;i<N+1;i++){
                for(int j = 0;j<N+1;j++){
                    expr1.addTerm(1.0,z[i][j][r]);
                }
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, instance.toteCapByRobot+1, "robot routing constraint: tote capacity of each robot");
        }

        for (int i = 0; i < N; i++) {
            for (int r= 0; r < R; r++) {
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1.0,z[i][i][r]);
                this.Model.addConstr(expr1, GRB.EQUAL, 0, "robot routing constraint: no self visiting");
            }
        }

        for (int r= 0; r < R; r++) {
            GRBLinExpr expr1 = new GRBLinExpr();
            for (int i = 0; i < N+1; i++) {
                expr1.addTerm(1.0,z[N][i][r]);
            }
            this.Model.addConstr(expr1, GRB.EQUAL, 1, "robot routing constraint: robot departs from the pick station");
        }

        for (int r= 0; r < R; r++) {
            GRBLinExpr expr1 = new GRBLinExpr();
            for (int i = 0; i < N+1; i++) {
                expr1.addTerm(1.0,z[i][N][r]);
            }
            this.Model.addConstr(expr1, GRB.EQUAL, 1, "robot routing constraint: robot returns to the pick station");
        }

        for(int i = 0;i<N;i++){
            for(int j = 0;j<N;j++){
                for(int r = 0;r<R;r++){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0,mu[i]);
                    expr1.addTerm(-1.0,mu[j]);
                    expr1.addTerm(instance.toteCapByRobot+1,z[i][j][r]);
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, instance.toteCapByRobot, "robot routing constraint:eliminate cycle");
                }
            }
        }

        for(int i =0;i<N;i++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int j = 0;j<N+1;j++){
                for(int r= 0;r<R;r++){
                    expr1.addTerm(1.0,z[i][j][r]);
                }
                this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "robot routing constraint: each tote is used at most once");
            }
        }

        /////// connection constraint /////
        GRBLinExpr expr = new GRBLinExpr();
        expr.addTerm(1, l[0][0]);
        this.Model.addConstr(expr, GRB.EQUAL, 1, "connection constraint-the first sku visitation starts from route 0");


        for(int k = 0;k<K;k++){
            if(k>0) {
                GRBLinExpr expr1 = new GRBLinExpr();
                for (int r = k / instance.toteCapByRobot; r < R; r++) {
                    expr1.addTerm(1, l[k][r]);
                }
                for (int s = 0; s < S; s++) {
                    expr1.addTerm(-1, x[s][k]);
                }
                this.Model.addConstr(expr1, GRB.EQUAL, 0, "connection constraint-each sku come seq must be assigned to a route");
            }else{
                GRBLinExpr expr1 = new GRBLinExpr();
                expr1.addTerm(1, l[0][0]);
                for (int s = 0; s < S; s++) {
                    expr1.addTerm(-1, x[s][0]);
                }
                this.Model.addConstr(expr1, GRB.EQUAL, 0, "connection constraint-each sku come seq must be assigned to a route");
            }
        }


        for(int k = 0;k<K-1;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int r = k/instance.toteCapByRobot;r<R;r++){
                expr1.addTerm(r, l[k][r]);
            }
            for(int r = (k+1)/instance.toteCapByRobot;r<R;r++){
                expr1.addTerm(R-r, l[k+1][r]);
            }
            this.Model.addConstr(expr1, GRB.LESS_EQUAL, R, "connection constraint: sku come route follows the come seq");
        }

        for(int k = 0;k<K-1;k++){
            GRBLinExpr expr1 = new GRBLinExpr();
            for(int r = k/instance.toteCapByRobot;r<R;r++){
                expr1.addTerm(r, l[k][r]);
            }
            for(int r = (k+1)/instance.toteCapByRobot;r<R;r++){
                expr1.addTerm(-r, l[k+1][r]);
            }
            this.Model.addConstr(expr1, GRB.GREATER_EQUAL, -1, "connection constraint: sku come route follows the come seq");
        }

        for(int k = 0;k<K;k++){
            for(int s = 0;s<S;s++){
                for (int r = k/instance.toteCapByRobot;r<R;r++){
                    GRBLinExpr expr1 = new GRBLinExpr();
                    expr1.addTerm(1.0, x[s][k]);
                    expr1.addTerm(1.0, l[k][r]);
                    for(int i:instance.toteSetBySKU[s]){
                        for(int j =0;j<N+1;j++) {
                            expr1.addTerm(-1.0, z[i][j][r]);
                        }
                    }
                    this.Model.addConstr(expr1, GRB.LESS_EQUAL, 1, "connection constraint: each tote should be planned in the route if it's scheduled");
                }
            }
        }
    }

    public void setObj() throws GRBException{
        /* OBJECTIVE FUNCTION */
        obj = new GRBLinExpr();
        for(int k = 0;k<K;k++){
            for(int s=0;s<S;s++){
                obj.addTerm(instance.pickTime, x[s][k]);
            }
        }
        for (int i = 0; i < N+1; i++) {
            for (int j = 0; j < N+1; j++) {
                for(int r= 0;r<R;r++){
                    obj.addTerm(instance.disBetweenTotes[i][j]/instance.moveSpeed, z[i][j][r]);
                }
            }
        }
        this.Model.setObjective(obj, GRB.MINIMIZE);
    }
    public void buildModel(Instance instance) throws IOException, GRBException {
        this.instance = instance; // this step is necessary
        this.env = new GRBEnv("MIP_model.log");
        this.Model = new GRBModel(this.env);

        O = instance.orderNum;
        S = instance.skuNum;
        N = instance.toteNum;
        K = (int)(instance.epsilon * O);
        R = instance.robotMaxRouteNum-2;

        orderPickingSubProb();
        robotRoutingSubProb();
        setObj();
        setGurobiParam();
    }

    public void setGurobiParam() throws GRBException {
        // STOP CRITERIA
        this.Model.set(GRB.DoubleParam.TimeLimit, 3600);
        this.Model.set(GRB.IntParam.OutputFlag, 1);
    }

    public void solveMIPModel(boolean isPrint) throws GRBException, IOException {
        /* SOLVING */
        this.Model.optimize();
        if (this.Model.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL|| this.Model.get(GRB.IntAttr.Status) == GRB.Status.TIME_LIMIT) {
            this.objVal = this.Model.get(GRB.DoubleAttr.ObjVal);
            this.runTime = this.Model.get(GRB.DoubleAttr.Runtime);
            this.lowerBound = this.Model.get(GRB.DoubleAttr.ObjBound);
            this.toteVisitNum = 0;
            for(int k = 0;k<K;k++){
                for(int s = 0;s<S;s++){
                    if(this.x[s][k].get(GRB.DoubleAttr.X)>0){
                        this.toteVisitNum++;
                        break;
                    }
                }
            }
            if(isPrint){
                Printer.printSKUComeSeq(K, S, x);
                Printer.printSKULeaveSeq(K, S, y);
                Printer.printOrderOpenSeq(instance, O, K, alpha);
                Printer.printOrderCloseSeq(O,K, beta);
                Printer.printRobotRoute(instance, N, R, z, mu);
            }
        } else {
            this.objVal = -1;
            this.Model.computeIIS();
            this.Model.write("model.ilp");
            System.out.println("Model not solved");
        }
        this.Model.dispose();
        this.env.dispose();
    }
}