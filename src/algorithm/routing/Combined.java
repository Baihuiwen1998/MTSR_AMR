package algorithm.routing;
import Entity.Location;
import instance_generation.Instance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.TreeSet;

public class Combined implements Routing{

    List<Location> locationsToVisitList;
    boolean[] ifLocationLeftInBlock;
    int[] subAisleLeft;
    int[] subAisleRight;
    TreeSet<Integer>[] aislesInBlock;

    int[][] furthestSubShelf;
    int[][] nearestSubShelf;
    int curBlock;
    int farthestBlock;
    int nearestBlock;
    int leftMostAisle;
    double routeLength;

    Instance instance;
    @Override
    public void init(Instance instance){
        this.instance = instance;
    }
    @Override
    public double getRouteLength(List<Location> locationsList){
        routeLength = 0.0;
        locationsToVisitList = new ArrayList<>();
        locationsToVisitList.addAll(locationsList);
        //Step1:
        //Determine the left most pick aisle that contains at least one pick location (called left pickaisle) and
        leftMostAisle = getLeftMostAisle();
        // determine the block farthest from the depot that contains at least one pick location (called farthest block).

        int[] farthestAndNearestB = getFarthestAndNearestBlock(this.instance.blockNum-1);
        farthestBlock =  farthestAndNearestB[0];
        nearestBlock = farthestAndNearestB[1];

        //Step2:  The route starts by going from the depot to the front of the left pick aisle (a).

        routeLength+= (double)leftMostAisle*(this.instance.aisleWidth+this.instance.toteDepth*2);


        //Step3:  Traverse the left pick aisle up to the front cross aisle of the farthest block (b)

        routeLength += (this.instance.toteWidth*(double)this.instance.shelfNum+ this.instance.crossAisleWidth)*(double)farthestBlock;

        List<Location> visitedLocationsList = new ArrayList<>();
        for(Location l:locationsToVisitList){
            if((l.getAisle()==leftMostAisle)&& (l.getBlock()<farthestBlock)){
                visitedLocationsList.add(l);
            }
        }
        locationsToVisitList.removeAll(visitedLocationsList);

        //Step4: set i = imin
        curBlock = farthestBlock;
        int curAisle = leftMostAisle;

        getIfLocationLeftInBlock(this.instance.blockNum,this.instance.aisleNum,this.instance.shelfNum);

        while(curBlock>=nearestBlock){
            //Step5: Determine whether or not block i contains items that have not been picked in step 3.
            if(ifLocationLeftInBlock[curBlock]){
                if(Math.abs(curAisle-subAisleLeft[curBlock])<=Math.abs(curAisle-subAisleRight[curBlock])){
                    //左侧更近，到达左侧
                    routeLength += Math.abs(curAisle-subAisleLeft[curBlock])*(this.instance.aisleWidth+this.instance.toteDepth*2);
                    dynamicRouting(true);
                    curAisle = subAisleRight[curBlock];
                }else{
                    //右侧更近，到达右侧
                    routeLength += Math.abs(curAisle-subAisleRight[curBlock])*(this.instance.aisleWidth+this.instance.toteDepth*2);
                    dynamicRouting(false);
                    curAisle=subAisleLeft[curBlock];
                }
            }else{
                //If no items have to be picked in block i: traverse the nearest sub aisle of block i to reach
                //the next block. Continue with step 7.
                routeLength += this.instance.crossAisleWidth+ this.instance.shelfNum*this.instance.toteWidth;
            }
            curBlock--;
        }
        //回到block 0
        routeLength += (double)nearestBlock*(instance.crossAisleWidth+ instance.shelfNum* instance.toteWidth);
        routeLength+= curAisle*(instance.aisleWidth+2* instance.toteDepth);
        return routeLength;
    }
    public void dynamicRouting(boolean fromLeftToRight){
        TreeSet<Integer> aisles;
        if(!fromLeftToRight) {
            //需要对aisle进行倒序操作
            aisles = aislesInBlock[curBlock];
            aisles  = (TreeSet<Integer>) aisles.descendingSet();
        }else {
            aisles =aislesInBlock[curBlock];
        }
        double[][] L = new double[2][instance.aisleNum];
        Arrays.fill(L[0],0);//La
        Arrays.fill(L[1],0);//Lb
        int formerAisle = aisles.first();
        if(curBlock==this.farthestBlock){
            L[0][formerAisle] = instance.crossAisleWidth+ instance.shelfNum* instance.toteWidth;
            L[1][formerAisle] = instance.crossAisleWidth+ (2*furthestSubShelf[curBlock][formerAisle]+1)* instance.toteWidth;
        }else{
            L[0][formerAisle] = instance.crossAisleWidth+ (2*(instance.shelfNum-1-nearestSubShelf[curBlock][formerAisle])+1)* instance.toteWidth;
            L[1][formerAisle] = instance.crossAisleWidth+ instance.shelfNum* instance.toteWidth;
        }
        aisles.pollFirst();
        for(int a:aisles){
            L[0][a] = Math.min(
                    L[0][formerAisle]+Math.abs(a-formerAisle)*(instance.toteDepth*2+ instance.aisleWidth)+ instance.crossAisleWidth+ (2*(instance.shelfNum-1-nearestSubShelf[curBlock][a])+1)* instance.toteWidth,
                    L[1][formerAisle]+Math.abs(a-formerAisle)*(instance.toteDepth*2+ instance.aisleWidth)+ instance.crossAisleWidth+ instance.shelfNum* instance.toteWidth
            );
            L[1][a] = Math.min(
                    L[0][formerAisle]+Math.abs(a-formerAisle)*(instance.toteDepth*2+ instance.aisleWidth)+ instance.crossAisleWidth+ instance.shelfNum* instance.toteWidth,
                    L[1][formerAisle]+Math.abs(a-formerAisle)*(instance.toteDepth*2+ instance.aisleWidth)+ instance.crossAisleWidth+ (2*furthestSubShelf[curBlock][a]+1)* instance.toteWidth
            );
            formerAisle =a;
        }
        if(aisles.size()>0){
            routeLength += L[1][aisles.last()];
        }else{
            routeLength += L[1][formerAisle];
        }

    }
    public int getLeftMostAisle(){
        int leftMostAisle = 10000;
        for(Location l:locationsToVisitList){
            leftMostAisle = Math.min(leftMostAisle,l.getAisle());
        }
        return leftMostAisle;
    }
    public int[] getFarthestAndNearestBlock(int maxBlockIndex){
        int farthestBlock = -1;
        int nearestBlock = maxBlockIndex;
        for(Location l:locationsToVisitList){
            farthestBlock = Math.max(farthestBlock,l.getBlock());
            nearestBlock = Math.min(nearestBlock,l.getBlock());
        }
        return new int[]{farthestBlock,nearestBlock};
    }

    public void getIfLocationLeftInBlock(int blockNum,int aisleNum,int shelfNum){
        ifLocationLeftInBlock = new boolean[blockNum];
        Arrays.fill(ifLocationLeftInBlock,false);
        subAisleLeft = new int[blockNum];
        Arrays.fill(subAisleLeft,aisleNum);
        subAisleRight = new int[blockNum];
        Arrays.fill(subAisleRight,-1);
        aislesInBlock = new TreeSet[blockNum];
        Arrays.fill(aislesInBlock,null);
        furthestSubShelf = new int[blockNum][aisleNum];
        nearestSubShelf= new int[blockNum][aisleNum];
        for(int i = 0;i<blockNum;i++){
            Arrays.fill(furthestSubShelf[i],-1);
            Arrays.fill(nearestSubShelf[i],shelfNum);
        }
        int block;
        int aisle;
        int shelf;
        TreeSet<Integer>  aisleInB;

        for(Location l:locationsToVisitList){
            block = l.getBlock();
            aisle = l.getAisle();
            shelf = l.getShelf();
            ifLocationLeftInBlock[block] = true;
            subAisleLeft[block] = Math.min(subAisleLeft[l.getBlock()],aisle);
            subAisleRight[block] = Math.max(subAisleRight[l.getBlock()],aisle);
            if(aislesInBlock[block] != null){
                aisleInB = aislesInBlock[block];
            }else{
                aisleInB = new TreeSet<>(((o1, o2)->o1.compareTo(o2)));
            }
            aisleInB.add(aisle);
            aislesInBlock[block] = aisleInB;
            furthestSubShelf[block][aisle] = Math.max(furthestSubShelf[block][aisle],shelf);
            nearestSubShelf[block][aisle] = Math.min(furthestSubShelf[block][aisle],shelf);
        }

    }
}
