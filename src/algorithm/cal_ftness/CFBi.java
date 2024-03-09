package algorithm.cal_ftness;
import algorithm.AlgorithmParams;
import algorithm.Solution;
import algorithm.sku_sequencing.*;
import algorithm.tote_selection.TSAlg;
import algorithm.tote_selection.TSMIP;
import algorithm.tote_selection.ToteSelection;
import gurobi.GRBException;
import instance_generation.Instance;

import java.io.IOException;

public class CFBi implements CalFitness{
    Instance instance;
    SKUScheduling skuScheduling;
    ToteSelection toteSelection;

    public void init(Instance instance){
        this.instance = instance;
        if(AlgorithmParams.skuSchedulingAlgMode == 0){
            skuScheduling = new SSMIP();
        }else if (AlgorithmParams.skuSchedulingAlgMode == 1) {
            skuScheduling = new SSGreedy();
        }else if (AlgorithmParams.skuSchedulingAlgMode == 2) {
            skuScheduling = new SSFCFS();
        }else if(AlgorithmParams.skuSchedulingAlgMode == 3) {
            skuScheduling = new SSFCFO();
        }
        skuScheduling.init(instance);
        if(AlgorithmParams.toteSelectionAlgMode == 0){
            toteSelection = new TSMIP();
        } else if (AlgorithmParams.toteSelectionAlgMode == 1) {
            toteSelection = new TSAlg();
        }
        toteSelection.init(instance);
    }

    public Solution cal(Solution solu) throws IOException, GRBException {
        solu.skuComeSeq = skuScheduling.getSKUSchedule(solu.orderSeq);
        if(solu.skuComeSeq == null){
            solu.objVal = 10000000000.0;
            solu.skuComeNum = 100000;
        }else{
            solu.objVal = solu.skuComeSeq.size();
            solu.skuComeNum = solu.skuComeSeq.size();
        }
        return solu;
    }

}
