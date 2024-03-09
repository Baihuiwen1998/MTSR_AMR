package algorithm.sku_sequencing;

import gurobi.GRBException;
import instance_generation.Instance;
import model.SubModelSCSByOS;

import java.io.IOException;
import java.util.List;

public class SSMIP implements SKUScheduling{
    Instance instance;
    @Override
    public void init(Instance instance){
        this.instance = instance;
    }
    @Override
    public List<Integer> getSKUSchedule(int[] orderSeq) throws IOException, GRBException {
        SubModelSCSByOS model = new SubModelSCSByOS();
        model.buildModel(instance, orderSeq);
        return model.solveMIPModel();
    }
}
