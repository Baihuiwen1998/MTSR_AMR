package Entity;

import java.io.Serializable;

public class Location implements Serializable {
    int tote;

    int sku;
    int block;

    int aisle;
    int shelf;
    double x;
    double y;

    public void initiateLocation(int block, int aisle, int shelf){
        this.block =block;
        this.aisle = aisle;
        this.shelf = shelf;
    }
    public void initiateToteLocation(int block, int aisle, int shelf, double x, double y,int SKU,int tote){
        this.block =block;
        this.aisle = aisle;
        this.shelf = shelf;
        this.x = x;
        this.y = y;
        this.sku = SKU;
        this.tote =tote;
    }

    public void cal_X_Y(double aisleWidth,double crossAisleWidth,double toteDepth,double toteWidth,int shelfNumInAisle){
        this.x = aisleWidth*(double)this.aisle+toteDepth*2.0*(double)this.aisle;
        this.y = crossAisleWidth*0.5 + crossAisleWidth*(double)this.block + (double)this.shelf *toteWidth +toteWidth*0.5+
                toteWidth*shelfNumInAisle*(double)this.block;
    }
    public int getBlock() {
        return block;
    }

    public void setBlock(int block) {
        this.block = block;
    }

    public int getAisle() {
        return aisle;
    }

    public void setAisle(int aisle) {
        this.aisle = aisle;
    }

    public int getShelf() {
        return shelf;
    }

    public void setShelf(int shelf) {
        this.shelf = shelf;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }
    public int getTote() {
        return tote;
    }

    public void setTote(int tote) {
        this.tote = tote;
    }

    public int getSku() {
        return sku;
    }

    public void setSku(int sku) {
        this.sku = sku;
    }
}