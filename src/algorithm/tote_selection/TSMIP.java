package algorithm.tote_selection;
import algorithm.Solution;
import gurobi.GRBException;
import instance_generation.Instance;
import model.SubModelRouteBySCS;

import java.io.IOException;
import java.util.List;

public class TSMIP implements ToteSelection{
    Instance instance;
    @Override
    public void init(Instance instance){
        this.instance = instance;
    }
    @Override
    public void cal(Solution solution) throws IOException, GRBException {
        List<Integer> skuComeSeq = solution.skuComeSeq;
        SubModelRouteBySCS subModelRouteBySCS = new SubModelRouteBySCS();
        subModelRouteBySCS.buildModel(instance, skuComeSeq);
        subModelRouteBySCS.solveMIPModel(0);
        solution.objVal = skuComeSeq.size()*instance.pickTime + subModelRouteBySCS.objVal/instance.moveSpeed;
    }
    @Override
    public double calTotesAndRoute(List<Integer> subSKUComeSeq, int r) throws IOException, GRBException {
       return 0.0;
    };
    public double calTotesAndRouteIndividually(List<Integer> subSKUComeSeq){
        return 0.0;
    }
}
