package computational_exp;

import gurobi.GRBException;
import instance_generation.Instance;
import instance_generation.InstanceDeserialize;
import instance_generation.InstanceGeneration;
import model.P0Model;
import support_func.Output;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

public class MainMILPModelCE {
    private static final long serialVersionUID = 1L;
    public static void main(String[] args) throws IOException, GRBException, ClassNotFoundException, InterruptedException {

        for(String instanceName: new String[]{"Set_T_2"}){
            Output output = new Output();
            output.init();
            String output_dir = ".//data/output//";
            String input_dir = ".//data//input//";
            Instance instance;

            // params
            boolean isGenerateInstance = false; // 是否生成新算例
            boolean isOutput = true; // 是否输出到csv
            boolean isPrint = true;
            boolean isSolve = true;
            String out_put_flag = "gurobi";

            for (int i = 0; i < 10; i++) {
                // read instance
                if (isGenerateInstance) {
                    instance = InstanceGeneration.generateInstance(input_dir + instanceName + "//" + (i + 1) + ".txt");
                } else {
                    instance = InstanceDeserialize.deserialize(input_dir + instanceName + "//" + (i + 1) + ".txt");
                }
                instance.toteCapByStation = 3;
                instance.orderBinCap = 3;
                P0Model modelBuilder = null;
                if (isSolve) {
                    // build & solve full model
                    modelBuilder = new P0Model();
                    modelBuilder.buildModel(instance);
                    modelBuilder.solveMIPModel(isPrint);
                }
                System.out.println(modelBuilder.objVal);
                // output
                if (isOutput && isSolve) {
                    output.addInstanceInfo(instance, instanceName + '_' + (i + 1));
                    output.addNFMIPInfo(modelBuilder);
                    output.printMIPToCSV(output_dir + instanceName + "_" + out_put_flag + ".csv");
                }
                if (modelBuilder.runTime > 1000) {
                    TimeUnit.MINUTES.sleep(3);
                }
            }
        }
    }
}