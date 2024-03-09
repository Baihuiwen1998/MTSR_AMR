package instance_generation;

public class InstanceParams {
    public static int orderNum = 50;
    public static int skuNum = 100;
    public static int toteNum = 500;
    public static int toteMaxReqNum = skuNum * 2;
    public static int orderBinCap = 3;
    public static int toteCapByStation = 3;
    public static int toteCapByRobot = 6;
    public static int robotMaxRouteNum = (int)Math.ceil((double)toteMaxReqNum/(double)toteCapByRobot);
    public static int aisleNum = 10; //10;
    public static int blockNum = 2;
    public static int shelfNum = 10;
    public static double aisleWidth = 3;
    public static double crossAisleWidth = 4; // 仓库内过道和过道之间的宽度
    public static double toteWidth = 0.6;
    public static double toteDepth = 0.5; // 料箱的宽度和深度, 实际料箱的长宽高大约是 400*300*120 (mm)，但是考虑到货架需要有安全间隔，两边加了10 cm
    public static double pickTime = 5;
    public static double moveSpeed = 1.8; // 机器人获取料箱所需时长和机器移动速度m/s
    public static double[] epsilonProb = new double[]{0.45, 0.65, 0.85, 0.95}; // 订单内有几个SKU需求的概率
    public static double skuHotProb = 0.3; // 热门sku数量占比
    public static double toteForHobProb = 0.5; // 存储了热门sku的料箱数量占比


}
