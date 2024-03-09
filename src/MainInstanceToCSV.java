import Entity.Location;
import instance_generation.Instance;
import instance_generation.InstanceDeserialize;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Set;
import java.io.File;
// Press ⇧ twice to open the Search Everywhere dialog and type `show whitespaces`,
// then press Enter. You can now see whitespace characters in your code.
public class MainInstanceToCSV {
    private static final long serialVersionUID = 1L;
    public static void main(String[] args) throws IOException, ClassNotFoundException {
        Instance instance;
        String output_dir = ".//data/output//";
        String input_dir = ".//data//input//";
        for (String instanceName : new String[]{
                "Set_H_1","Set_H_2", "Set_I_1","Set_I_2", "Set_J_1","Set_J_2", "Set_K_1","Set_K_2", "Set_L_1","Set_L_2", "Set_M_1","Set_M_2",
                "Set_N_1","Set_N_2", "Set_O_1","Set_O_2", "Set_P_1","Set_P_2", "Set_Q_1","Set_Q_2", "Set_R_1","Set_R_2", "Set_S_1","Set_S_2",
        }) {
            for (int i = 0; i < 10; i++) {
                instance = InstanceDeserialize.deserialize(input_dir + instanceName + "//" + (i + 1) + ".txt");
                File file = new File(output_dir + instanceName + "//" + (i + 1) + ".csv");

                // 创建所有必需的父目录
                if (!file.getParentFile().exists()) {
                    file.getParentFile().mkdirs();
                }
                try (FileWriter writer = new FileWriter(output_dir + instanceName + "//" + (i + 1) + ".csv")) {
                    // 创建所有必需的父目录
                    // 写入常量的变量名
                    writer.append("orderNum,skuNum,toteNum,toteMaxReqNum,orderBinCap,toteCapByStation,toteCapByRobot,robotMaxRouteNum,aisleNum,blockNum,shelfNum,aisleWidth,crossAisleWidth,toteWidth,toteDepth,pickTime,moveSpeed\n");
                    // 写入常量
                    writer.append(String.format("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f%n",
                            instance.orderNum, instance.skuNum, instance.toteNum, instance.toteMaxReqNum,
                            instance.orderBinCap, instance.toteCapByStation, instance.toteCapByRobot,
                            instance.robotMaxRouteNum, instance.aisleNum, instance.blockNum, instance.shelfNum,
                            instance.aisleWidth, instance.crossAisleWidth, instance.toteWidth, instance.toteDepth,
                            instance.pickTime, instance.moveSpeed));

                    // 写入算例服从分布信息的变量名
                    writer.append("epsilon\n");
                    // 写入算例服从分布信息
                    writer.append(String.valueOf(instance.epsilon)).append("\n");

                    // 写入订单相关参数与集合的变量名
                    writer.append("skuSetByOrder\n");
                    // 写入订单相关参数与集合
                    for (Set<Integer> set : instance.skuSetByOrder) {
                        writer.append(set.toString()).append(",");
                    }
                    writer.append("\n");
                    writer.append("orderWithOneSKU\n");
                    writer.append(instance.orderWithOneSKU.toString()).append("\n");

                    // 写入SKU相关参数与集合的变量名
                    writer.append("toteSetBySKU\n");
                    // 写入SKU相关参数与集合
                    for (Set<Integer> set : instance.toteSetBySKU) {
                        writer.append(set.toString()).append(",");
                    }
                    writer.append("\n");


                    // 写入料箱相关参数与集合的变量名
                    writer.append("blockByTote\n");
                    // 写入料箱相关参数与集合
                    writer.append(Arrays.toString(instance.blockByTote)).append(",");
                    writer.append("\n");
                    writer.append("aisleByTote\n");
                    writer.append(Arrays.toString(instance.aisleByTote)).append(",");
                    writer.append("\n");
                    writer.append("shelfByTote\n");
                    writer.append(Arrays.toString(instance.shelfByTote)).append(",");
                    writer.append("\n");
                    writer.append("xByTote\n");
                    writer.append(Arrays.toString(instance.xByTote)).append(",");
                    writer.append("\n");
                    writer.append("yByTote\n");
                    writer.append(Arrays.toString(instance.yByTote)).append(",");
                    writer.append("\n");
                    writer.append("skuByTote\n");
                    writer.append(Arrays.toString(instance.skuByTote)).append("\n");


                }
            }
        }
    }
}