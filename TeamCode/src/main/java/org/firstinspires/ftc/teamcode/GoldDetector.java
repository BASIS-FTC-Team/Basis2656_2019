package org.firstinspires.ftc.teamcode;



import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Parameters.LABEL_GOLD_MINERAL;

public class GoldDetector {

    /** the List to keep the recognized minerals, sorted by the center_x of minierals */
    List<Mineral>               mineralList = new ArrayList<>();
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

    public void GoldDetector() {
    }

    public void update(List<Recognition> recognitionList) {

        numM = 0;
        numG = 0;
        numS = 0;
        mineralList.clear();

        if (recognitionList == null) {
            numM = 0;
        } else {
            numM = recognitionList.size();
        }

        if (numM == 0) {
            numG = 0;
            numS = 0;
        } else {
            mineralList.clear();
            for (Recognition r : recognitionList) {
                mineralList.add(new Mineral(r));
                if (r.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    numG++;
                } else {
                    numS++;
                }
            }
            Collections.sort(mineralList, new Comparator<Mineral>() {
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

    goldFound = (numG > 0)? true : false;

    setGoldOrderAndAngle();

//    setGoldPosition();

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
            for (Mineral m: mineralList) {
                gOrder ++;
                if (m.isGold()) {
                    firstGoldOrderFromLeft = gOrder;
                    firstGoldAngle = m.getAngle();
                    break;
                }
            }
        }
    }


    public void setHAlignSlope() {
        if (numM < 2) { hAlignSlope = Double.MAX_VALUE; }
        else {
            Mineral m1 = mineralList.get(0);
            Mineral m2 = mineralList.get(1);
            hAlignSlope = m1.calSlopeTo(m2);
        }
    }

    // Gets
    public int getNumM() { return numM; }

    public int getNumG() { return numG; }

    public int getNumS() { return numS; }

    public int getFirstGoldOrderFromLeft() { return firstGoldOrderFromLeft; }

    public double getFirstGoldAngle() { return firstGoldAngle; }

    public List<Mineral> getMineralList() { return mineralList;}

    public double getHAlignSlope() { return hAlignSlope; }

    public boolean goldIsFound() { return goldFound; }


    //the parameter idx below starts from 0, goes to numM-1
    public Mineral get(int idx ) {
        if (numM == 0) {
            return null;
        }
        else if ((idx < 0)|| (idx > (numM-1))) {
            return null;
        } else {
            return mineralList.get(idx);
        }
    }

    //The following estimation is based on that the robot starts from the right (move right after landing)
    // so there are two minerals within the camera with sure
    public GoldPosition estimateGoldPostion() {
        if ( numG > 0) { // gold detected
            if (firstGoldAngle >= 0 ) {
                return GoldPosition.RIGHT;
            } else {
                return GoldPosition.MIDDLE;
            }
        } else { // no gold detected
            if (numM >= 2){  // all detected minerals are silvers
                return GoldPosition.LEFT;  // so the gold is on the right
            } else {
                return GoldPosition.UNKNOWN;
            }
        }
    }

    // Hope to check the Gold/Silver mineral in close distance with only one mineral in the picture
    public boolean isTheOnlyGold() {
        return (numM == 1) && (numG == 1) ;
    }

    public boolean isTheOnlySilver() {
        return (numM == 1) && (numS == 1) ;
    }
}
