package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

class Mineral {

    private String label;
    private boolean isGold;
    private double leftSide;
    private double rightSide;
    private double topSide;
    private double bottomSide;
    private double center_x;
    private double center_y;
    private double angleToTheMineral;
    private double confidence;

    Mineral(Recognition r) {
        label = r.getLabel();
        leftSide = r.getLeft();
        rightSide = r.getRight();
        topSide = r.getTop();
        bottomSide = r.getBottom();
        center_x = (leftSide + rightSide) / 2;
        center_y = (topSide + bottomSide) / 2;
        angleToTheMineral = r.estimateAngleToObject(AngleUnit.DEGREES);
        confidence = r.getConfidence();
        if (r.getLabel().equals("Gold Mineral")) {
            isGold = true;
        } else {
            isGold = false;
        }
    }

    public void setLeft(double l) {
        this.leftSide = l;
    }

    public void setRight(double r) {
        this.rightSide = r;
    }

    public void setTop(double t) {
        this.topSide = t;
    }

    public void setBottom(double b) {
        this.bottomSide = b;
    }

    public void setCenterX(double cx) {this.center_x = cx;}

    public void setCenterY(double cy) {this.center_y = cy;}

    public void setAngle(double a) {
        this.angleToTheMineral = a;
    }

    public void setConfidence(double confidence) { this.confidence = confidence; }

    public double getLeft() {
        return leftSide;
    }

    public double getRight() {
        return rightSide;
    }

    public double getTop() {
        return topSide;
    }

    public double getBottom() {
        return bottomSide;
    }

    public double getCenterX() {return center_x ; }

    public double getCenterY() {return center_y ; }

    public double getAngle() {
        return angleToTheMineral;
    }

    public double getConfidence() { return confidence; }

    public void markGold() { isGold = true;}

    public void demarkGold() {isGold = false;}

    public boolean isGold() {return isGold;}

    public double calSlopeTo(Mineral mineral) {
        double x1 = this.center_x;
        double y1 = this.center_y;
        double x2 = mineral.center_x;
        double y2 = mineral.center_y;
        if (x2 != x1 ) {
            return (y2 - y1) / (x2 - x1);
        } else {
            return Double.MAX_VALUE;
        }
    }

}
