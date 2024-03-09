package model;

import gurobi.*;
import instance_generation.Instance;

import java.io.IOException;
import java.util.List;
import java.util.Set;

public class SubModelLeastFinishONum {
    GRBEnv env;
    public GRBModel Model;
    Instance instance;
    GRBVar[] x;
    GRBVar[] y;

    int O;

    public void buildModel(int comeK, Instance instance, Set<Integer> orderSet, List<Integer> skuComeSeq) throws GRBException {
        O = instance.orderNum;
        this.env = new GRBEnv("MIP_model.log");
        this.Model = new GRBModel(this.env);

        // 构建模型
        this.x = new GRBVar[comeK]; // 表示第k个sku是否在comeK之前离开拣选站
        for(int k = 0;k<comeK;k++){
            String varName = "x_" + k;
            x[k] = this.Model.addVar(0, 1, 0, GRB.BINARY, varName);
        }
        this.y = new GRBVar[O];
        for(int o:orderSet){
            String varName = "y_"+o;
            y[o] = this.Model.addVar(0,1,0,GRB.BINARY, varName);
        }
        // 添加约束
        GRBLinExpr expr = new GRBLinExpr();
        for(int k=0;k<comeK;k++){
            expr.addTerm(1.0, x[k]);
        }
        this.Model.addConstr(expr, GRB.EQUAL, comeK-instance.toteCapByStation+1, "select comeK-toteCapByStation+1 number of skus leave the station");

        for(int k=0;k<comeK;k++){
            int s = skuComeSeq.get(k);
            if(k==skuComeSeq.lastIndexOf(s)) {
                for (int o:instance.orderSetBySKU[s]) {
                    expr = new GRBLinExpr();
                    expr.addTerm(1.0, y[o]);
                    expr.addTerm(-1.0, x[k]);
                    this.Model.addConstr(expr, GRB.GREATER_EQUAL, 0, "order selection by idx");
                }
            }
        }

        // 添加目标
        GRBLinExpr obj = new GRBLinExpr();
        for(int o:orderSet){
            obj.addTerm(1.0, y[o]);
        }
        this.Model.setObjective(obj, GRB.MINIMIZE);
    }
    public void setGurobiParam() throws GRBException {
        // STOP CRITERIA
        this.Model.set(GRB.IntParam.OutputFlag, 0);
    }

    public double solveMIPModel() throws GRBException, IOException {
        /* SOLVING */
        setGurobiParam();
        this.Model.optimize();
        if (this.Model.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL) {
            return this.Model.get(GRB.DoubleAttr.ObjVal);
        } else {
//            System.out.println("Feasiblity Model not solved");
            return 0.0;
        }
    }
}
