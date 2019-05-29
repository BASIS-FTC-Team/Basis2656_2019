package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import static org.firstinspires.ftc.teamcode.Parameters.*;
import static org.firstinspires.ftc.teamcode.Hardware2019.*;


public class LiftArm {

    boolean isUping = false;
    boolean isDowning = false;

    public void init() {
        verticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalMotor.setDirection(DcMotor.Direction.REVERSE);
        verticalMotor.setPower(0);
    }

    public void initEnc() {

        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        verticalMotor.setPower(0);

    }

    public void graspOn() {
        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition() - 50);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(GRASP_ON_POWER);
    }

    public void testGraspOn(double power) {
        //verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition());
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(power);
    }


    //This if for Robot going up automatically, whiel the lift itself will be going down
    public void autoGoingUp(int counts) {
        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition() - counts);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(0.9);
        ElapsedTime runtime = new ElapsedTime();
        while (verticalMotor.isBusy() && runtime.milliseconds() < 3000) {
            //wait
            if (isTouched()) {
                break;
            }
        }
        verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition());
        verticalMotor.setPower(GRASP_ON_POWER);
    }

    public int getLiftPosition() {
        return verticalMotor.getCurrentPosition();
    }

    public void setLiftZeroPosition() {
        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveUpOrDown(double power) {
        verticalMotor.setDirection(DcMotor.Direction.FORWARD);
        verticalMotor.setPower(power);
    }

    public void moveUp() {

        verticalMotor.setDirection(DcMotor.Direction.FORWARD);
        verticalMotor.setPower(-1.0 * LIFT_POWER);
    }

    public void moveDown() {
        verticalMotor.setDirection(DcMotor.Direction.FORWARD);
        verticalMotor.setPower(LIFT_POWER);
    }

    public void stop() {
        verticalMotor.setDirection(DcMotor.Direction.FORWARD);
        verticalMotor.setPower(0);
    }

    public void moveUpEnc() {
        moveUpDownEnc(LIFT_POWER,LIFT_COUNTS_PER_UPDOWN_EFFORT,1000);
    }

    public void moveDownEnc() {
        moveUpDownEnc(LIFT_POWER,-LIFT_COUNTS_PER_UPDOWN_EFFORT,1000);
    }

    /**
     * Moving the lift up / down with specified counts at specified speed(power for motor)
     *
     * @param power positive, between 0.0 and 1.0;
     *              if negative, the code turn it into its Math.abs(power)
     *              if more than 1.0 or less than -1.0, Range.clip will turn it into between -1.0 and 1.0
     * @param counts integer, indicate the counts the motor will move.
     *               for most brands of motors:
     *                  - positive, for CW
     *                  - negative, for CCW
     *               but maybe for some other brands, it will be reversed.
     */
    public void moveUpDownEnc(double power, int counts, int timeoutMS) {

        int newTarget;
        ElapsedTime runtime = new ElapsedTime();
        newTarget = verticalMotor.getCurrentPosition() + counts;
        verticalMotor.setPower(Math.abs(Range.clip(power,-1.0,1.0)));
        verticalMotor.setTargetPosition(newTarget);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while (verticalMotor.isBusy() && runtime.milliseconds() < timeoutMS) {
            // wait
        }
    }

    ////////////////////////////////////////////////////

    public void keepUpingEnc() {
        double power = verticalMotor.getPower();
        if (Math.abs(verticalMotor.getTargetPosition() - verticalMotor.getCurrentPosition()) < 1000) {
            verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition() + 10000);
        }
//        if (power < LIFT_POWER) {
//            power += 0.1;
//            if (power > LIFT_POWER) {
//                power = LIFT_POWER;
//            }
//        }
        power = LIFT_POWER;

        isUping = true;
        isDowning = false;

        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(power);

        TelemetryWrapper.setLine(7, String.format("Lift power is %.3f, power is %.3f",LIFT_POWER, verticalMotor.getPower()));
    }

    public void keepDowningEnc() {
        double power = verticalMotor.getPower();
        if (Math.abs(verticalMotor.getTargetPosition() - verticalMotor.getCurrentPosition()) < 1000) {
            verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition() - 10000);
        }
//        if (power < LIFT_POWER) {
//            power += 0.1;
//            if (power > LIFT_POWER) {
//                power = LIFT_POWER;
//            }
//        }
        power = LIFT_POWER;
        isUping = false;
        isDowning = true;

        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(power);
    }

    public void latchOnEnc(int timeoutMS) {
        ElapsedTime runtime = new ElapsedTime();
        //double upDist = LIFT_AUTO_MOVE_DIST; // Measured mannually
        int moveCounts = LIFT_AUTO_LATCHING_COUNTS; // (int) 125 [upDist] / 97.5 [15 segments of chain * 6.5 mm/seg] * 56/24 [Reduction rate] * 288 (counts_per_rev) + 125
        double power = LIFT_POWER;
        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newTarget = verticalMotor.getCurrentPosition() - moveCounts;
        verticalMotor.setTargetPosition(newTarget);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        verticalMotor.setPower(power);
        while ((verticalMotor.isBusy()) && (runtime.milliseconds() < timeoutMS )) {
//            if ((Math.abs(verticalMotor.getTargetPosition() - verticalMotor.getCurrentPosition())) < 20) {
//                power = 0.3;
//            }
            isUping   = false;
            isDowning = true;
            if (isTouched()) {
                break;
            }
            verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            verticalMotor.setPower(power);
        }
        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition());
        verticalMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        verticalMotor.setPower(0);
        isUping = false;
        isDowning = false;
    }

    public void landOffEnc(int timeoutMS) {
        ElapsedTime runtime = new ElapsedTime();
        //double downDist = LIFT_AUTO_MOVE_DIST;
        int moveCounts = LIFT_AUTO_LANDING_COUNTS; // (int) 125 [downDist] / 97.5 [15 segments of chain * 6.5 mm/seg] * 56/24 [Reduction rate] * 288 (counts_per_rev), plus 5 mm
        double power = LIFT_POWER;
        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition() + moveCounts);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        verticalMotor.setPower(power);
        while ((verticalMotor.isBusy()) && (runtime.milliseconds() < timeoutMS )) {
//            if ((Math.abs(verticalMotor.getTargetPosition() - verticalMotor.getCurrentPosition())) < 20) {
//                power = 0.3;
//            }
            isUping = true;
            isDowning = false;
//            if (isTouched()) {
//                break;
//            }
            verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            verticalMotor.setPower(power);
        }
        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition());
        verticalMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        verticalMotor.setPower(0);
        isUping = false;
        isDowning = false;
    }

    public void stopEnc(){
//        double power = verticalMotor.getPower();
//        if ((power > 0.1) && (verticalMotor.isBusy())) {
//            if (isUping) {
//                verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition() + 20);
//            } else if (isDowning) {
//                verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition() - 20);
//            }
//            runtime.reset();
//            verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            verticalMotor.setPower(0.1);
//        }
//        while ((verticalMotor.isBusy() ) && (runtime.milliseconds() < 100) ) {
//            // Waiting
//        }
        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition());
        verticalMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        verticalMotor.setPower(0.0);

        isUping = false;
        isDowning = false;

    }

    public void stopAtOnceEnc(){

        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setTargetPosition(verticalMotor.getCurrentPosition());
        verticalMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        verticalMotor.setPower(0.0);

        isUping = false;
        isDowning = false;
    }

    public double getRunningPower()  { return verticalMotor.getPower();}

    ////////////////////////////////////////////////////


    public boolean isTouched() {
        return !touchSensor.getState();
    }

    public boolean isUping() { return isUping;}

    public boolean isDowning() { return isDowning;}

    public boolean isStopped() { return ( (!isUping) && (!isDowning) );}
}
