package algorithm.cal_ftness;

import algorithm.Solution;
import gurobi.GRBException;
import instance_generation.Instance;

import java.io.IOException;

public interface CalFitness {
    public void init(Instance instance);
    public Solution cal(Solution solution) throws IOException, GRBException;
}
