package support_func;

import instance_generation.Instance;

import java.util.*;

/*
Simulate the picking process and calculate the minimum robot num that provides no picker waiting time/
 */
public class MinRobotNoWait {
	Instance instance;

	double pickT, regTForOrder, regTForTote, shiftTForBinTote;
	public void init(Instance instance, double pickT, double regTForOrder, double regTForTote, double shiftTForBinTote){
		this.instance = instance;
		this.pickT = pickT;
		this.regTForOrder = regTForOrder;
		this.regTForTote = regTForTote;
		this.shiftTForBinTote = shiftTForBinTote;
	}
	public List<Double> calMinimalRobotNoWait(int[] orderSeq, List<Integer> skuComeSeq,List<Integer> skuLeaveSeq, List<List<Integer>> skuComeRoutes, List<Double> routeLenList){
		List<Double> pickerWaitTByRobotNum = new ArrayList<>();
		int robotNum = 1;
		List<Double>  skuArrivalTimeList;
		Double prePickerWaitingTime = -1.0;
		Double pickerWaitingTime;
		while(true){
			skuArrivalTimeList = calSKUArrivalTime(robotNum, skuComeRoutes, routeLenList);
			pickerWaitingTime= calPickerWaitingTime(orderSeq, skuComeSeq, skuLeaveSeq, skuArrivalTimeList, skuComeRoutes.get(0).size());
			if(pickerWaitingTime.equals(prePickerWaitingTime)){
				break;
			}
			if(pickerWaitingTime == 0.0){
				pickerWaitTByRobotNum.add(pickerWaitingTime);
				break;
			}
			pickerWaitTByRobotNum.add(pickerWaitingTime);
			prePickerWaitingTime = pickerWaitingTime;
			robotNum++;
		}
		return pickerWaitTByRobotNum;
	}

	public List<Double> calSKUArrivalTime(int robotNum, List<List<Integer>> skuComeRoutes, List<Double> routeLenList){
		List<Double> skuArrivalTimeList = new ArrayList<>();
		Double[] nextIdleTimeOfRobot = new Double[robotNum];
		for(int r = 0;r<robotNum;r++){
			nextIdleTimeOfRobot[r] = 0.0;
		}
		int earlistIdleRobot = 0;
		double earlistIdleTime = 0.0;
		for(int route =0;route<routeLenList.size();route++){
			List<Integer> skusByRoute = skuComeRoutes.get(route);
			nextIdleTimeOfRobot[earlistIdleRobot] = nextIdleTimeOfRobot[earlistIdleRobot]+ routeLenList.get(route)+skusByRoute.size()*instance.pickTime;
			for(int cnt = 0;cnt<skusByRoute.size();cnt++){
				skuArrivalTimeList.add(nextIdleTimeOfRobot[earlistIdleRobot]);
			}
			earlistIdleTime = nextIdleTimeOfRobot[earlistIdleRobot];
			// find the next Idle robot
			for(int robot = 0;robot<robotNum;robot++){
				if(nextIdleTimeOfRobot[robot] < earlistIdleTime){
					earlistIdleTime = nextIdleTimeOfRobot[robot];
					earlistIdleRobot = robot;
				}
			}
		}
		return skuArrivalTimeList;
	}

	public double calPickerWaitingTime(int[] orderSeq, List<Integer> skuComeSeq,List<Integer> skuLeaveSeq, List<Double> skuArrivalTimeList, int firstRouteSize){
		double pickerWaitingT = 0.0;
		double currT = 0.0;
		int nextOrderIdx = 0;
		boolean orderNotAdded;
		int nextOrder;
		ArrayList<Integer> keyList = new ArrayList<>();
		// ----------------- Order picking process -----------------
		// initiate picking orders
		HashMap<Integer, Set<Integer>> pickingOrders = new HashMap<>();
		while(pickingOrders.size() < instance.orderBinCap){
			Set<Integer> skusByOrder = new HashSet<>(instance.skuSetByOrder[orderSeq[nextOrderIdx]]);
			pickingOrders.put(orderSeq[nextOrderIdx], skusByOrder);
			nextOrderIdx++;
		}
		// initiate active SKUs
		HashSet<Integer> activeSKUs = new HashSet<>();

		int nextInboundSKUIdx = 0;
		int nextOutboundSKUIdx = 0;
		// pick orders
		while(!pickingOrders.isEmpty()){
			if (activeSKUs.size() == instance.toteCapByStation) {
				// remove sku
				activeSKUs.remove(skuLeaveSeq.get(nextOutboundSKUIdx));
				nextOutboundSKUIdx++;
			}
			// inbound sku
			if(nextInboundSKUIdx == skuComeSeq.size()){
				System.out.println(activeSKUs.toString());
				System.out.println(pickingOrders.toString());
			}
			activeSKUs.add(skuComeSeq.get(nextInboundSKUIdx));
			if(skuArrivalTimeList.get(nextInboundSKUIdx) > currT){
				// record waiting time
				if(nextInboundSKUIdx>=firstRouteSize) {
					pickerWaitingT += skuArrivalTimeList.get(nextInboundSKUIdx) - currT;
				}
				currT = skuArrivalTimeList.get(nextInboundSKUIdx);
			}
			currT += shiftTForBinTote;
			nextInboundSKUIdx++;
			keyList.clear();
			keyList.addAll(pickingOrders.keySet());
			for (int key : keyList) {
				Set<Integer> skuSetByOrder = pickingOrders.get(key);
				for(int s:skuSetByOrder){
					if(activeSKUs.contains(s)){
						currT += pickT + instance.orderBinCap * regTForOrder + instance.toteCapByStation * regTForTote;
					}
				}
				skuSetByOrder.removeAll(activeSKUs);
				pickingOrders.put(key, skuSetByOrder);
				if (pickingOrders.get(key).isEmpty()) {//判断是否拣选完毕
					pickingOrders.remove(key);//删除该订单
					orderNotAdded = true;
					while (orderNotAdded && (nextOrderIdx!=orderSeq.length)){//判断是否需要添加新的订单，或者是否仍有新订单可以添加
						nextOrder = orderSeq[nextOrderIdx];
						skuSetByOrder = new HashSet<>(instance.skuSetByOrder[nextOrder]);
						for(int s:skuSetByOrder){
							if(activeSKUs.contains(s)){
								currT += pickT + instance.orderBinCap * regTForOrder + instance.toteCapByStation * regTForTote;
							}
						}
						skuSetByOrder.removeAll(activeSKUs);
						if (!skuSetByOrder.isEmpty()) {
							// 如果该新订单可以直接被当前服务的SKU完成，则不需要进行进入pickingOrders的操作
							pickingOrders.put(nextOrder, skuSetByOrder);
							orderNotAdded = false;
						}
						currT += shiftTForBinTote;
						nextOrderIdx++;
					}
				}
			}
		}
		return pickerWaitingT;
	}

}
