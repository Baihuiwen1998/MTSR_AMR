package algorithm;

import instance_generation.Instance;

import java.util.*;

public class Operators {

    int combinationNum;

    int[][] combinations;
    int N;


    public void init(Instance instance){
        this.combinationNum = instance.combinationNum;
        this.combinations = instance.orderCombination;
        this.N = instance.orderNum;
    }

    public int[] shaking(int[] seq, int operator) {
        int[] new_seq = new int[seq.length];
        Random random = new Random();
        int index;
        switch(operator) {
            case 1:
                index= random.nextInt(this.combinationNum);
                new_seq = this.twoOpt(this.combinations[index][0],this.combinations[index][1], seq);
            case 2:
                index= random.nextInt( this.combinationNum);
                new_seq = this.reversePart(this.combinations[index][0],this.combinations[index][1], seq);
            case 3:
                index= random.nextInt( this.combinationNum);
                new_seq = this.twoForward(this.combinations[index][0],this.combinations[index][1], seq);
            case 4:
                index= random.nextInt(this.N-1);
                new_seq = this.partForward(index, seq);
        }
        return new_seq;
    }


    public int[] twoOpt(int s1, int s2, int[] seq){
        //调换两个位置
        int[] new_seq = seq.clone();
        new_seq[s1] = seq[s2];
        new_seq[s2] = seq[s1];
        return new_seq;
    }

    public int[] reversePart(int s1, int s2,  int[] seq){
        //对两个位置内部进行倒置
        int[] new_seq = seq.clone();
        int idx = s1;
        for (int i=s2;i>=s1;i--) {
            new_seq[idx] = seq[i];
            idx++;
        }
        return new_seq;
    }
    public int[] twoForward(int s1, int s2, int[] seq){
        //任意选取两个位置前置
        int[] new_seq = new int[N];
        new_seq[0] = seq[s1];
        new_seq[1] = seq[s2];
        System.arraycopy(seq, 0, new_seq, 2, s1);
        System.arraycopy(seq, s1+1, new_seq, s1+2, s2-s1-1);
        System.arraycopy(seq, s2+1, new_seq, s2+1, N-s2-1);
        return new_seq;
    }
    public int[] partForward(int s1, int[] seq){
        //s1位置之后前移
        int[] new_seq = new int[N];
        System.arraycopy(seq, s1, new_seq, 0, N-s1);
        System.arraycopy(seq, 0, new_seq, N-s1, s1);
        return new_seq;
    }

    public List<Integer> shakingList(List<Integer> seq, int operator) {
        List<Integer> new_seq = new ArrayList<>();
        Random random = new Random();
        int index;
        switch(operator) {
            case 1:
                index= random.nextInt(this.combinationNum);
                new_seq = this.twoOptList(this.combinations[index][0],this.combinations[index][1], seq);
            case 2:
                index= random.nextInt( this.combinationNum);
                new_seq = this.reversePartList(this.combinations[index][0],this.combinations[index][1], seq);
            case 3:
                index= random.nextInt( this.combinationNum);
                new_seq = this.twoForwardList(this.combinations[index][0],this.combinations[index][1], seq);
            case 4:
                index= random.nextInt(this.N-1);
                new_seq = this.partForwardList(index, seq);
        }
        return new_seq;
    }


    public List<Integer> twoOptList(int s1, int s2, List<Integer>  seq){
        //调换两个位置
        List<Integer> new_seq = new ArrayList<>();
        new_seq.addAll(seq.subList(0, s1));
        new_seq.add(seq.get(s2));
        new_seq.addAll(seq.subList(s1+1, s2));
        new_seq.add(seq.get(s1));
        new_seq.addAll(seq.subList(s2+1, seq.size()));
        return new_seq;
    }

    public List<Integer> reversePartList(int s1, int s2,  List<Integer> seq){
        //对两个位置内部进行倒置
        List<Integer> new_seq = new ArrayList<>();
        new_seq.addAll(seq.subList(0, s1));
        int idx = s1;
        for (int i=s2;i>=s1;i--) {
            new_seq.add(seq.get(i));
            idx++;
        }
        new_seq.addAll(seq.subList(s2+1, seq.size()));
        return new_seq;
    }
    public List<Integer> twoForwardList(int s1, int s2, List<Integer> seq){
        //任意选取两个位置前置
        List<Integer> new_seq = new ArrayList<>();
        new_seq.add(seq.get(s1));
        new_seq.add(seq.get(s2));
        new_seq.addAll(seq.subList(0, s1));
        new_seq.addAll(seq.subList(s1+1, s2));
        new_seq.addAll(seq.subList(s2+1, seq.size()));
        return new_seq;
    }
    public List<Integer> partForwardList(int s1, List<Integer> seq){
        //s1位置之后前移
        List<Integer> new_seq = new ArrayList<>();
        new_seq.addAll(seq.subList(s1, seq.size()));
        new_seq.addAll(seq.subList(0, s1));
        return new_seq;
    }

    public Set<Integer> randomSetGeneration(int num, int range) {
        //产生多个ran范围内的不重复随机数
        Set<Integer> randomSet = new HashSet<>();
        Random random = new Random();
        int rand = 0;
        while(randomSet.size()<num){
            rand = random.nextInt(range);
            randomSet.add(rand);
        }
        return randomSet;
    }


}
