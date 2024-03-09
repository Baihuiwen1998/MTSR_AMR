package algorithm.sku_sequencing;

import gurobi.GRBException;
import instance_generation.Instance;

import java.io.IOException;
import java.util.List;

public interface SKUScheduling {
    public void init(Instance instance);
    public List<Integer> getSKUSchedule(int[] orderSeq) throws IOException, GRBException;
}
