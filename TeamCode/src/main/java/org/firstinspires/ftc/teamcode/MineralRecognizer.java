package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class MineralRecognizer {

//    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD = "Gold Mineral";
    private static final String LABEL_SILVER = "Silver Mineral";

    /* the TFOD used for the recognization */
    private TFObjectDetector    td = null;

    /** the List to keep the recognized minerals, sorted by the center_x of minierals */
    private List<Mineral>       rMList = new ArrayList<Mineral>();

    /** if there is gold recognized, goldFound is set true, otherwise, is false */
    private boolean             goldFound = false;

    /** the number of recognized Minerals, Golds, and Silvers */
    private int                 numM = 0;
    private int                 numG = 0;
    private int                 numS = 0;

    /** If no gold (even no mineral) is detected, it will be 0;
     * otherwise, it is the order of the gold in the recognized minerals List when first occurs
     */
    private int                 firstGoldOrderFromLeft = 0;

    /** Goal for giving out the Gold's position when 3 minerals are detected
     * when there is ONLY ONE gold in the EXACT THREE minerals' case, it indicates
     * the actual position of the gold, i.e., LEFT / MIDDLE / RIGHT
     * otherwise it will be UNKNOWN
     */
    private GoldPosition        goldPosition = GoldPosition.UNKNOWN;

    /**
     * if no gold (or even no mineral) detected, the angle is set to 90.0 (Degree)
     * otherwise, the angle is the one from the camera to the first gold
     * the value is 0, when the gold is just right in front of the camera,
     *              negative, when the gold is on the right
     *              positive, when the gold is on the left
     * */
    private double              firstGoldAngle = 90.0; //in Degrees, range: -90.0 ~ +90.0

    /**
     *  the slope is 0, when the alignment line of the centers of the recognized minerals is vertical to the camera direction
     *               positive, when the line is from top-left to bottom-right
     *               negative, when the line is from bottom-left to top-right
     *  when no mineral or only one mineral is detected, the value is set to Double.MAX_VALUE;
     */
    private double              hAlignSlope = Double.MAX_VALUE;

    public void initialize(TFObjectDetector tfod) {
        this.td = tfod;
        //td.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        update();
    }

    public void update(){
        if (td != null) {
            List<Recognition> recognitionList = td.getUpdatedRecognitions();
            if (recognitionList == null) {
                numM = 0;
                //TelemetryWrapper.setLine(1,);
            } else {
                numM = recognitionList.size();
            }
            if (numM == 0) {
                numG = 0;
                numS = 0;
                rMList.clear();
            } else {
                rMList.clear();
                for (Recognition r : recognitionList) {
                    //Mineral mineral = new Mineral(r);
                    rMList.add(new Mineral(r));
                    if (r.getLabel().equals(LABEL_GOLD)) {
                        numG++;
                    } else {
                        numS++;
                    }
                }
                Collections.sort(rMList, new Comparator<Mineral>() {
                    @Override
                    public int compare(Mineral lhs, Mineral rhs) {
                        double a = lhs.getLeft();
                        double b = rhs.getLeft();
                        if (a > b) {
                            return 1;
                        } else if (a == b) {
                            return 0;
                        } else {
                            return -1;
                        }
                    }
                });
            }
        }
        goldFound = (numG > 0)? true : false;
        setGoldOrderAndAngle();
        setGoldPosition();
        setHAlignSlope();
    }

    // Sets
    public void setGoldOrderAndAngle() {
        if (numG == 0) {
            firstGoldOrderFromLeft = 0;
            firstGoldAngle = 90.0;
        }
        else {
            int gOrder = 0;
            for (Mineral m: rMList) {
                gOrder ++;
                if (m.isGold()) {
                    firstGoldOrderFromLeft = gOrder;
                    firstGoldAngle = m.getAngle();
                    break;
                }
            }
        }
    }
    public void setGoldPosition() {
        if ((numM == 3) && (numG == 1)){
            if (firstGoldOrderFromLeft == 1)      { goldPosition = GoldPosition.LEFT; }
            else if (firstGoldOrderFromLeft == 2) { goldPosition = GoldPosition.MIDDLE; }
            else if (firstGoldOrderFromLeft == 3) { goldPosition = GoldPosition.RIGHT; }
        } else {goldPosition = GoldPosition.UNKNOWN;}
    }
    public void setHAlignSlope() {
        if (numM < 2) { hAlignSlope = Double.MAX_VALUE; }
        else {
            Mineral m1 = rMList.get(0);
            Mineral m2 = rMList.get(1);
            hAlignSlope = m1.calSlopeTo(m2);
        }
    }
    // Gets
    public int getNumM() { return numM; }
    public int getNumG() { return numG; }
    public int getNumS() { return numS; }

    public GoldPosition getGoldPosition() { return goldPosition; }
    public int getFirstGoldOrderFromLeft() { return firstGoldOrderFromLeft; }
    public double getFirstGoldAngle() { return firstGoldAngle; }
    public List<Mineral> getRecognizedMineralList() { return rMList;}
    public double getHAlignSlope() { return hAlignSlope; }
    public boolean goldIsFound() { return goldFound; }
    public TFObjectDetector getTFOD() { return td; }
    //the parameter idx below starts from 0, goes to numM-1
    public Mineral get(int idx ) {
        if (numM == 0) {
            return null;
        }
        else if ((idx < 0)|| (idx > (numM-1))) {
            return null;
        } else {
            return rMList.get(idx);
        }
    }

    public void activate() {
        if (td != null) {
            td.activate();
        }
    }
    public void deactivate() {
        if(td != null) {
            td.deactivate();
        }
    }
}
