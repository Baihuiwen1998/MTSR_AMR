package instance_generation;

import Entity.Location;

import java.io.Serializable;
import java.util.List;
import java.util.Set;
public class Instance  implements Serializable {
    /*
    常量/
     */
    public int orderNum, skuNum, toteNum;
    public int toteMaxReqNum; // 最大需要的料箱数量
    public int orderBinCap, toteCapByStation, toteCapByRobot; // 拣选站可容纳的最大订单数量、料箱数量，机器人可以放置的最大料箱数量
    public int robotMaxRouteNum;
    public int aisleNum, blockNum, shelfNum; // 仓库内每个block的aisle数量，每个aisle内的shelf数量
    public double aisleWidth, crossAisleWidth; // 仓库内过道和过道之间的宽度
    public double toteWidth, toteDepth; // 料箱的宽度和深度
    public double pickTime, moveSpeed; // 机器人获取料箱所需时长和机器移动速度
    /*
    算例服从分布信息/
     */
    public double[] epsilonProb;
    public double epsilon;
    public double skuHotProb;
    public double toteForHotProb;
    /*
    订单相关参数与集合/
     */
    public Set<Integer>[] skuSetByOrder; // 订单所需sku集合
    public List<Integer> orderWithOneSKU;

    /*
    SKU相关参数与集合
     */
    public Set<Integer>[] toteSetBySKU; // 存储sku的料箱集合
    public int[] toteNumBySKU; // sku的料箱数量
    public Set<Integer>[] orderSetBySKU; //含有该sku需求的订单集合

    /*
    料箱相关参数与集合/
     */
    public int[] blockByTote, aisleByTote, shelfByTote;//
    public double[] xByTote, yByTote;
    public double[][] disBetweenTotes;
    public int[] skuByTote;

    /*
    启发式用：订单相似度与订单组合/
     */
    public double[][] orderSimilarity;
    public int[][] orderCombination;
    public int combinationNum;

    /*
    启发式用：地图点/
     */
    public List<Location> toteLocationsList;
    public Set<Location>[] locationSetBySKU;













}
