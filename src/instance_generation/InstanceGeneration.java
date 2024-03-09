package instance_generation;

import Entity.Location;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
public class InstanceGeneration{
//    private static long serialVersionUID = 2906642554793891381L;

    public static Instance instance;

    public static Instance generateInstance(String filename) throws IOException, ClassNotFoundException {
        instance = new Instance();
        setBasicParams();
        generateOrders();
        generateTotes();
        generateLocations();
        getSimilarity();
        getCombination();
        serialize(filename);
        return instance;
    }

    public static void setBasicParams(){
        instance.orderNum = InstanceParams.orderNum;
        instance.skuNum = InstanceParams.skuNum;
        instance.toteNum = InstanceParams.toteNum;
        instance.toteMaxReqNum = InstanceParams.toteMaxReqNum;
        instance.orderBinCap = InstanceParams.orderBinCap;
        instance.toteCapByStation = InstanceParams.toteCapByStation;
        instance.toteCapByRobot = InstanceParams.toteCapByRobot;
        instance.robotMaxRouteNum = InstanceParams.robotMaxRouteNum;
        instance.aisleNum = InstanceParams.aisleNum;
        instance.blockNum = InstanceParams.blockNum;
        instance.shelfNum = InstanceParams.shelfNum;
        instance.aisleWidth = InstanceParams.aisleWidth;
        instance.crossAisleWidth = InstanceParams.crossAisleWidth;
        instance.toteWidth = InstanceParams.toteWidth;
        instance.toteDepth = InstanceParams.toteDepth;
        instance.moveSpeed = InstanceParams.moveSpeed;
        instance.pickTime = InstanceParams.pickTime;
        instance.epsilonProb = InstanceParams.epsilonProb;
        instance.skuHotProb = InstanceParams.skuHotProb;
        instance.toteForHotProb = InstanceParams.toteForHobProb;
    }

    public static void generateOrders(){
        /*
        根据产品热度生成订单内需要的sku信息/
         */
        instance.skuSetByOrder = new Set[instance.orderNum];
        instance.orderWithOneSKU = new ArrayList<>();
        instance.orderSetBySKU = new Set[instance.skuNum];
        for(int i =0;i<instance.skuNum;i++){
            instance.orderSetBySKU[i] = new HashSet<>();
        }
        int hotSKUNum = (int)Math.ceil(instance.skuNum * instance.skuHotProb);
        Set<Integer> skuSet;
        int skuNumInOrder;
        int sumSKUNum = 0;
        int sku;
        for(int i = 0;i<instance.orderNum;i++){
            skuSet = new HashSet<>();
            double rnd = Math.random();
            int idx = 0;
            loop:while(idx < instance.epsilonProb.length && rnd > instance.epsilonProb[idx]){
                idx++;
            }
            skuNumInOrder = idx+1;
            if(skuNumInOrder == 1){
                instance.orderWithOneSKU.add(i);
            }
            sumSKUNum += skuNumInOrder;
            while(skuSet.size()<skuNumInOrder){
                rnd = Math.random();
                if(rnd<=instance.skuHotProb){
                    sku = (int)(hotSKUNum*Math.random());
                }else{
                    sku = (int)(hotSKUNum+ (instance.skuNum-hotSKUNum)*Math.random());
                }
                skuSet.add(sku);
                instance.orderSetBySKU[sku].add(i);
            }
            instance.skuSetByOrder[i] = skuSet;
        }
        instance.epsilon = (double)sumSKUNum/(double)instance.orderNum;
        System.out.println("order epsilon = "+ instance.epsilon);
    }

    public static void generateTotes(){
        /*
        生成料箱内提供的sku信息/
        生成料箱在仓库内的地理位置信息/
        计算料箱之间的距离/
         */
        instance.blockByTote = new int[instance.toteNum];
        instance.aisleByTote = new int[instance.toteNum];
        instance.shelfByTote = new int[instance.toteNum];

        instance.xByTote = new double[instance.toteNum];
        instance.yByTote = new double[instance.toteNum];

        instance.skuByTote = new int[instance.toteNum];
        instance.toteSetBySKU = new Set[instance.skuNum];
        instance.toteNumBySKU = new int[instance.skuNum];

        int toteNumForHotSKU = (int)(instance.toteNum * instance.toteForHotProb);
        int hotSKUNum = (int)Math.ceil(instance.skuNum * instance.skuHotProb);
        toteNumForHotSKU -= hotSKUNum;

        for(int i = 0;i<instance.skuNum;i++){
            Set<Integer> toteSet = new HashSet<>();
            instance.toteSetBySKU[i] = toteSet;
        }
        for(int i = 0;i<instance.toteNum;i++){
            // tote在仓库内的地理位置
            int block = (int)(instance.blockNum * Math.random());
            int aisle = (int)(instance.aisleNum * Math.random());
            int shelf = (int)(instance.shelfNum * Math.random());
            instance.blockByTote[i] = block;
            instance.aisleByTote[i] = aisle;
            instance.shelfByTote[i] = shelf;

            instance.xByTote[i] = (instance.aisleWidth+instance.toteDepth*2)*((double)aisle);
            instance.yByTote[i]= instance.crossAisleWidth * 0.5 + (instance.toteWidth*instance.shelfNum +
                    instance.crossAisleWidth)*((double) block) + (((double) shelf)+0.5) *instance.toteWidth;

            // tote内存的sku种类
            if(i<instance.skuNum){
                instance.skuByTote[i] = i;
                instance.toteSetBySKU[i].add(i);
            }else{
                int sku;
                if(toteNumForHotSKU>0){
                    sku = (int)(hotSKUNum*Math.random());
                    toteNumForHotSKU--;
                }else{
                    sku = (int)(hotSKUNum+(instance.skuNum- hotSKUNum)*Math.random());
                }
                instance.skuByTote[i] = sku;
                instance.toteSetBySKU[sku].add(i);
            }
        }
        for(int i=0;i<instance.skuNum;i++){
            instance.toteNumBySKU[i] = instance.toteSetBySKU[i].size();
        }

        instance.disBetweenTotes = new double[instance.toteNum+1][instance.toteNum+1];
        for(int i = 0;i<instance.toteNum;i++){
            for(int j = 0;j<instance.toteNum;j++){
                if(instance.blockByTote[i]==instance.blockByTote[j]){//相同block
                    if(instance.aisleByTote[i]==instance.aisleByTote[j]){
                        instance.disBetweenTotes[i][j] =  Math.abs(instance.yByTote[i] - instance.yByTote[j]);
                    }else{
                        double distance_1 = (instance.shelfNum*2-2.0-instance.shelfByTote[i]-instance.shelfByTote[j])*instance.toteWidth+
                                Math.abs(instance.xByTote[i]-instance.xByTote[j])+ instance.toteWidth+
                                instance.crossAisleWidth;
                        double distance_2 = ((double)(instance.shelfByTote[i]+instance.shelfByTote[j]))*instance.toteWidth+
                                Math.abs(instance.xByTote[i]-instance.xByTote[j])+ instance.toteWidth+
                                instance.crossAisleWidth;
                        instance.disBetweenTotes[i][j] = Math.min(distance_2, distance_1);
                    }
                }else {//不同block
                    //generate distance
                    instance.disBetweenTotes[i][j] = Math.abs(instance.xByTote[i] - instance.xByTote[j]) + Math.abs(instance.yByTote[i] - instance.yByTote[j]);
                }
                instance.disBetweenTotes[j][i] = instance.disBetweenTotes[i][j];
            }
        }
        for(int i = 0;i<instance.toteNum;i++){
            instance.disBetweenTotes[i][instance.toteNum] = instance.xByTote[i] + instance.yByTote[i];
            instance.disBetweenTotes[instance.toteNum][i] = instance.disBetweenTotes[i][instance.toteNum] ;
        }

    }

    public static void getSimilarity(){
        instance.orderSimilarity = new double[instance.orderNum][instance.orderNum];
        Set<Integer> intersection = new HashSet<>();
        Set<Integer> differenceSet1 = new HashSet<>();
        Set<Integer> differenceSet2 = new HashSet<>();
        double score_for_sku_with_limited_tote;
        for(int i= 0;i<instance.orderNum;i++){
            for(int j = 0;j<instance.orderNum;j++){
                intersection.clear();
                differenceSet1.clear();
                differenceSet2.clear();
                intersection.addAll(instance.skuSetByOrder[i]);
                intersection.retainAll(instance.skuSetByOrder[j]);
                score_for_sku_with_limited_tote = 0.0;
                for(int s:intersection){
                    if(instance.toteSetBySKU[s].size()==1) {
                        score_for_sku_with_limited_tote += 0.5;
                    }
                }
                differenceSet1.addAll(instance.skuSetByOrder[i]);
                differenceSet1.removeAll(instance.skuSetByOrder[j]);
                differenceSet2.addAll(instance.skuSetByOrder[j]);
                differenceSet2.removeAll(instance.skuSetByOrder[i]);
                instance.orderSimilarity[i][j] = intersection.size()/Math.sqrt(intersection.size()*intersection.size()+
                        differenceSet1.size()*differenceSet1.size()+differenceSet2.size()*differenceSet2.size())+
                score_for_sku_with_limited_tote;
                instance.orderSimilarity[j][i] = instance.orderSimilarity[i][j];
            }
        }
    }

    public static void getCombination(){
        instance.orderCombination = new int[instance.orderNum*(instance.orderNum-1)/2][2];
        int countCombination = 0;
        int[] combi;
        for (int i1 = 0;i1<instance.orderNum;i1++) {
            for (int i2 = i1+1;i2<instance.orderNum;i2++) {
                combi = new int[2];
                combi[0]=i1;
                combi[1]=i2;
                instance.orderCombination[countCombination] = combi;
                countCombination +=1;
            }
        }
        instance.combinationNum = countCombination;
    }

    public static void generateLocations(){
        instance.toteLocationsList = new ArrayList<>();
        instance.locationSetBySKU = new Set[instance.skuNum];
        for(int i = 0;i<instance.skuNum;i++){
            instance.locationSetBySKU[i] = new HashSet<>();
        }
        Location l;
        for(int i =0;i<instance.toteNum;i++){
            l = new Location();
            l.initiateToteLocation(instance.blockByTote[i],instance.aisleByTote[i],instance.shelfByTote[i],
                    instance.xByTote[i],instance.yByTote[i],instance.skuByTote[i],i);
            instance.toteLocationsList.add(l);
            instance.locationSetBySKU[instance.skuByTote[i]].add(l);
        }

    }

    public static void serialize(String filename) throws IOException, ClassNotFoundException, IOException {
        ObjectOutputStream objectOutputStream =
                new ObjectOutputStream(new FileOutputStream((new File(filename))));
        objectOutputStream.writeObject(instance);
        objectOutputStream.close();
    }

}
