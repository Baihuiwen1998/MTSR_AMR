package algorithm.tote_selection;

import algorithm.Solution;
import gurobi.GRBException;
import instance_generation.Instance;

import java.io.IOException;
import java.util.List;

public interface ToteSelection {
    public void init(Instance instance);
    public void cal(Solution solution) throws IOException, GRBException;
    public double calTotesAndRoute(List<Integer> subSKUComeSeq, int r) throws IOException, GRBException;
    public double calTotesAndRouteIndividually(List<Integer> subSKUComeSeq);
}
