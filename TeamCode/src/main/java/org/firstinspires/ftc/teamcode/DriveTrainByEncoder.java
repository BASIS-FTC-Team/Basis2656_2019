package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Config;

import static org.firstinspires.ftc.teamcode.Hardware2019.*;
import static org.firstinspires.ftc.teamcode.Parameters.*;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import static java.lang.Math.*;

public class DriveTrainByEncoder {

    private ElapsedTime runtime = new ElapsedTime();

    /////////// All configuration parameters originally here are already moved to Parameters.java //////////////

    public void init() {

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        setPowerToAllDriveMotors(0);

    }


    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double xMillimeters, double yMillimeters, double rAnglesInDegree,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        int deltaLF,deltaRF,deltaLB,deltaRB;
        int maxDelta;

        // Determine new target position, and pass to motor controller
        deltaLF =  (int)((-yMillimeters-xMillimeters*XY_CORRECTION)* COUNTS_PER_MM  +rAnglesInDegree * COUNTS_PER_DEGREE);
        deltaRF =  (int)((-yMillimeters+xMillimeters*XY_CORRECTION)* COUNTS_PER_MM  -rAnglesInDegree * COUNTS_PER_DEGREE);
        deltaLB =  (int)((-yMillimeters+xMillimeters*XY_CORRECTION)* COUNTS_PER_MM  +rAnglesInDegree * COUNTS_PER_DEGREE);
        deltaRB =  (int)((-yMillimeters-xMillimeters*XY_CORRECTION)* COUNTS_PER_MM  -rAnglesInDegree * COUNTS_PER_DEGREE);
        maxDelta = max(max(abs(deltaLB), abs(deltaLF)), max(abs(deltaRB), abs(deltaRF)));

        newLeftFrontTarget = leftFront.getCurrentPosition() + deltaLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + deltaRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + deltaLB;
        newRightBackTarget = rightBack.getCurrentPosition() + deltaRB;

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();

        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(Range.clip(abs(speed*deltaLF/maxDelta),0,1));
        rightFront.setPower(Range.clip(abs(speed*deltaRF/maxDelta),0,1));
        leftBack.setPower(Range.clip(abs(speed*deltaLB/maxDelta),0,1));
        rightBack.setPower(Range.clip(abs(speed*deltaRB/maxDelta),0,1));

        TelemetryWrapper.setLine(0,  "Running to (x,y,r)=("+xMillimeters+":"+yMillimeters +":"+rAnglesInDegree+")");
        TelemetryWrapper.setLine(1,  "Wheels to (lf,rf,lr,rr) ("+newLeftFrontTarget+":"+newRightFrontTarget +":"+newLeftBackTarget+":"+newRightBackTarget+")");
        while (
                (runtime.seconds()< timeoutS) ||
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

            // Display it for the driver.
            TelemetryWrapper.setLine(2,  "Running at ("+leftFront.getCurrentPosition()+":"+rightFront.getCurrentPosition()
                    +":"+leftBack.getCurrentPosition()+":"+rightBack.getCurrentPosition()+")");
        }

        // Stop all motion;
        //stop();
        setPowerToAllDriveMotors(0);

        // Turn off RUN_TO_POSITION
        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void move(double powerx, double powery, double turn){
        double speedx = powerx;
        double speedy = powery;
        double offset = turn;
        leftFront.setPower(Range.clip(speedy-speedx+offset,-1,1));
        rightFront.setPower(Range.clip(speedy+speedx-offset,-1,1));
        leftBack.setPower(Range.clip(speedy+speedx+offset,-1,1));
        rightBack.setPower(Range.clip(speedy-speedx-offset,-1,1));
    }

    public void stop(){ setPowerToAllDriveMotors(0); }


    /** The following are totally new methods for encoder-driving added by J.TU 17 April, 2019 */
    public void initEnc() {

//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.FORWARD);
//        leftBack.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.setDirection(DcMotor.Direction.FORWARD);

        setModeToAllDriveMotors(DcMotor.RunMode.RUN_USING_ENCODER);
        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(leftFront.getCurrentPosition());
        rightFront.setTargetPosition(rightFront.getCurrentPosition());
        leftBack.setTargetPosition(leftBack.getCurrentPosition());
        rightBack.setTargetPosition(rightBack.getCurrentPosition());
        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);
    }

    /**
     *      speed: the power used for the driving moters
     *      dist: distance to move forward or backward in millimeters
     *              - positive for forwards
     *              - negative for backwards
     *      timeoutMS: timeout setting for the move (in milliseconds)
     */
    public void moveForthBackEnc(double dist, int timeoutMS ) {

        /** The slowSpeed is set for slow start-up and slow-down when stopping to avoid vibration */
        double slowSpeed = Range.clip(MIN_DRIVE_SPEED,-1,1);
        double fastSpeed = Range.clip(MAX_DRIVE_SPEED,-1,1);

        /** Sets the time frame for the move to time out */
        int timeout = timeoutMS;
        ElapsedTime runtime = new ElapsedTime();

        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int deltaLF, deltaRF, deltaLB, deltaRB;
        //int maxDelta;

        int countsToMove = (int) ( dist * COUNTS_PER_MM );
        deltaLF = countsToMove;
        deltaRF = countsToMove;
        deltaLB = countsToMove;
        deltaRB = countsToMove;
        newLeftFrontTarget = leftFront.getCurrentPosition() + deltaLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + deltaRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + deltaLB;
        newRightBackTarget = rightBack.getCurrentPosition() + deltaRB;

        /** Sets the new targets */
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        double power = fastSpeed;  // start from fastSpeed

        //double power = Range.clip(speed, -1,1);
        runtime.reset();
        setPowerToAllDriveMotors(power);


        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                ( runtime.milliseconds() < timeout ) ) {
            // waiting to finish
            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)      ||
                    (Math.abs(rightFront.getTargetPosition()-rightFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN) ||
                    (Math.abs(leftBack.getTargetPosition()-leftBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)     ||
                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN ) ) {
                power = slowSpeed;
            }
            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerToAllDriveMotors(power);
        }

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);

    }

    // Dist: positive, go forwards  (robot)
    //       negative, go backwards (robot)
    public void moveForthBackEnc2(double dist, int timeoutMS ) {

        TelemetryWrapper.setLine(0,String.format("moveForthBackEnc2(dist,timeoutMs)=(%.1f,%d)",dist,timeoutMS));
        /** The slowSpeed is set for slow start-up and slow-down when stopping to avoid vibration */
        // 短距离移动（COUNTS_THRESHOLD_FOR_SLOWDOWN*2个counts以内），前半段从startupSpeed升到slowSpeed，后半段从slowSpeed降到0；否则，power从startupSpeed开始按正弦曲线加速到fastSpeed，
        // 然后，匀速运动至剩下的counts小于等于COUNTS_THRESHOLD_FOR_SLOWDOWN个时，开始按正弦曲线减速到0。
        double startupSpeed = 0.05;
        double slowSpeed = Range.clip(MIN_DRIVE_SPEED,-1,1);
        double fastSpeed = Range.clip(MAX_DRIVE_SPEED,-1,1);
        TelemetryWrapper.setLine(1,String.format("(sS,fS,cS)=(%.1f,%.1f,%.1f)",slowSpeed,fastSpeed,leftFront.getPower()));

        double power = slowSpeed; // start from speed 0

        TelemetryWrapper.setLine(2,String.format("(calPower,curPower)=(%.1f,%.1f)",power,leftFront.getPower()));
        int startCountUp, endCountUp, startCountDown, endCountDown;
        double startPowerUp, endPowerUp, startPowerDown, endPowerDown;


        /** Sets the time frame for the move to time out */
        int timeout = timeoutMS;
        ElapsedTime runtime = new ElapsedTime();

        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int deltaLF, deltaRF, deltaLB, deltaRB;
        //int maxDelta;

        int countsToMove = (int) ( dist * COUNTS_PER_MM );

        TelemetryWrapper.setLine(3,String.format("(dist,COUNTS_PER_MM,counsToMove) = (%.1f,%d,%d)",dist,COUNTS_PER_MM,countsToMove));
        // Calculate the starting points and ending points for speed-up and speed-down
        if (Math.abs(countsToMove)> (2*COUNTS_THRESHOLD_FOR_SLOWDOWN) ) {
            startCountUp = leftFront.getCurrentPosition();
            endCountUp = startCountUp - COUNTS_THRESHOLD_FOR_SLOWDOWN;
            startPowerUp = startupSpeed;
            endPowerUp = fastSpeed;

            startCountDown = startCountUp + countsToMove + COUNTS_THRESHOLD_FOR_SLOWDOWN;
            endCountDown =startCountDown - COUNTS_THRESHOLD_FOR_SLOWDOWN;
            startPowerDown = fastSpeed;
            endPowerDown = startupSpeed;
            TelemetryWrapper.setLine(4,String.format("Case1:(sCup,eCup,sPup,ePup)=(%d,%d,%.1f,%.1f),(sCd,eCd,sPd,ePd)=(%d,%d,%.1f,%.1f)",
                    startCountUp,endCountUp,startPowerUp,endPowerUp,
                    startCountDown,endCountDown,startPowerDown,endPowerDown));

        } else {

            startCountUp = leftFront.getCurrentPosition();
            endCountUp = startCountUp + (int) (countsToMove/2);
            startPowerUp = startupSpeed;
            endPowerUp = slowSpeed;

            startCountDown = endCountUp;
            endCountDown =startCountDown + (int) (countsToMove/2);
            startPowerDown = slowSpeed;
            endPowerDown = startupSpeed;
            TelemetryWrapper.setLine(4,String.format("Case2:(sCup,eCup,sPup,ePup)=(%d,%d,%.1f,%.1f),(sCd,eCd,sPd,ePd)=(%d,%d,%.1f,%.1f)",
                    startCountUp,endCountUp,startPowerUp,endPowerUp,
                    startCountDown,endCountDown,startPowerDown,endPowerDown));
        }

        deltaLF = countsToMove;
        deltaRF = countsToMove;
        deltaLB = countsToMove;
        deltaRB = countsToMove;
        newLeftFrontTarget = leftFront.getCurrentPosition() + deltaLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + deltaRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + deltaLB;
        newRightBackTarget = rightBack.getCurrentPosition() + deltaRB;

        /** Sets the new targets */
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        TelemetryWrapper.setLine(5,String.format("(c1,c2,c3,c4)=(%d,%d,%d,%d),(t1,t2,t3,t4)=(%d,%d,%d,%d)",
                leftFront.getCurrentPosition(),rightFront.getCurrentPosition(),leftBack.getCurrentPosition(),rightBack.getCurrentPosition(),
                leftFront.getTargetPosition(),rightFront.getTargetPosition(),leftBack.getTargetPosition(),rightBack.getTargetPosition()));

        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        //power = leftBack.getPower(); // start from current Speed

        power = slowSpeed;
        //double power = Range.clip(speed, -1,1);
        runtime.reset();
        setPowerToAllDriveMotors(power);
        TelemetryWrapper.setLine(7,String.format("Before Loop: Mode=%s,Power=%.1f",leftFront.getMode().toString(),leftFront.getPower()));

        int curCount;
        int loops=0;
        int loops0 = 0;
        int loops1 = 0;
        int loops2 = 0;
        int loops3 = 0;
        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                ( runtime.milliseconds() < timeout ) ) {
            // waiting to finish
//            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)      ||
//                    (Math.abs(rightFront.getTargetPosition()-rightFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN) ||
//                    (Math.abs(leftBack.getTargetPosition()-leftBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)     ||
//                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN ) ) {
//                //power = slowSpeed;
//                power = calPower()
//            }
            loops ++;
            curCount = leftBack.getCurrentPosition();
            if (whereIsTheNumber(curCount, startCountUp, endCountUp) == -1 ) {
                loops0 ++;
                power = startupSpeed;
            }
            if ( whereIsTheNumber(curCount, startCountUp, endCountUp) == 0) {
                loops1 ++;
                power = calPower(curCount, startCountUp, endCountUp, startPowerUp, endPowerUp);
            } else if ( whereIsTheNumber(curCount,startCountDown, endCountDown) == 0 ) {
                loops2 ++;
                power = calPower(curCount, startCountDown, endCountDown, startPowerDown, endPowerDown);
            } else {
                loops3 ++;
                power = leftBack.getPower();
            }
//            TelemetryWrapper.setLine(10,String.format("Power is %.3f .",power));
//            TelemetryWrapper.setLine(1,String.format("%b, %b, %b, %b",leftFront.isBusy(),rightFront.isBusy(),leftBack.isBusy(),rightBack.isBusy()));
//            TelemetryWrapper.setLine(2,String.format("(Loops,0,1,2,3):(%d, %d,%d,%d,%d)",loops, loops0,loops1,loops2,loops3));
//            TelemetryWrapper.setLine(3,String.format("Left Front count: %d . Target: %d", leftFront.getCurrentPosition(), leftFront.getTargetPosition()));
//            TelemetryWrapper.setLine(4,String.format("Right Front count: %d . Target: %d", rightFront.getCurrentPosition(),rightFront.getTargetPosition()));
//            TelemetryWrapper.setLine(5,String.format("Left Back count: %d . Target: %d", leftBack.getCurrentPosition(),leftBack.getTargetPosition()));
//            TelemetryWrapper.setLine(6,String.format("Right Back count: %d . Target: %d", rightBack.getCurrentPosition(),rightBack.getTargetPosition()));

            TelemetryWrapper.setLine(2,String.format("(calPower,curPower)=(%.1f,%.1f)",power,leftFront.getPower()));
            TelemetryWrapper.setLine(5,String.format("(c1,c2,c3,c4)=(%d,%d,%d,%d),(t1,t2,t3,t4)=(%d,%d,%d,%d)",
                    leftFront.getCurrentPosition(),rightFront.getCurrentPosition(),leftBack.getCurrentPosition(),rightBack.getCurrentPosition(),
                    leftFront.getTargetPosition(),rightFront.getTargetPosition(),leftBack.getTargetPosition(),rightBack.getTargetPosition()));
             TelemetryWrapper.setLine(6,String.format("(%b, %b, %b, %b) and (Loops,0,1,2,3):(%d,%d,%d,%d,%d)",
                    leftFront.isBusy(),rightFront.isBusy(),leftBack.isBusy(),rightBack.isBusy(),
                    loops, loops0,loops1,loops2,loops3 ));

            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerToAllDriveMotors(power);


            TelemetryWrapper.setLine(7,String.format("Within Loop: Mode=%s,Power=%.1f",leftFront.getMode().toString(),leftFront.getPower()));
        }

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);

        TelemetryWrapper.setLine(7,String.format("Exit Loop: Mode=%s,Power=%.1f",leftFront.getMode().toString(),leftFront.getPower()));

    }


    /**
     *      speed: the power used for the driving motors
     *      dist: distance to move left or right in millimeters
     *              - positive for right
     *              - negative for left
     *      timeoutMS: timeout setting for the move (in milliseconds)
     */
    public void moveLeftRightEnc(double dist, int timeoutMS ) {

        /** The slowSpeed is set for slow start-up and slow-down when stopping to avoid vibration */
        double slowSpeed = Range.clip(MIN_DRIVE_SPEED,-1,1);
        double fastSpeed = Range.clip(MAX_DRIVE_SPEED,-1,1);

        /** Sets the time frame for the move to time out */
        int timeout = timeoutMS;
        ElapsedTime runtime = new ElapsedTime();

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /** Calculate the targets */
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int deltaLF, deltaRF, deltaLB, deltaRB;
        int countsToMove = (int) (  dist * COUNTS_PER_MM * XY_CORRECTION );
        TelemetryWrapper.setLine(8,String.format("Counts to move: %d", countsToMove));
//        deltaLF = - countsToMove;
//        deltaRF =   countsToMove;
//        deltaLB =   countsToMove;
//        deltaRB = - countsToMove;
        deltaLF =   countsToMove;
        deltaRF =  - countsToMove;
        deltaLB = - countsToMove;
        deltaRB =  countsToMove;
        newLeftFrontTarget = leftFront.getCurrentPosition() + deltaLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + deltaRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + deltaLB;
        newRightBackTarget = rightBack.getCurrentPosition() + deltaRB;

        /** Sets the new targets */
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        double power = fastSpeed;  // start from fastSpeed
        runtime.reset();
        setPowerToAllDriveMotors(power);

        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                (runtime.milliseconds() < timeout)) {
            // waiting to finish

            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)      ||
                    (Math.abs(rightFront.getTargetPosition()-rightFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN) ||
                    (Math.abs(leftBack.getTargetPosition()-leftBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)     ||
                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN ) ) {
                power = slowSpeed;
            }
            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerToAllDriveMotors(power);
        }
        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);

    }

    /** Spinning clock-wise or counter-clock-wise
     *      speed: power used for motors
     *      angle_in_degree: the size of the angle the robot will turn
     *                          - positve for turning clock-wise
     *                          - negative for turning counter-clock-wise
     *      timeoutMS: timeout setting for the move (in millisecond)
     */
    public void spinEnc(double angle_in_degree, int timeoutMS ) {

        /** The slowSpeed is set for slow start-up and slow-down when stopping to avoid vibration */
        double slowSpeed = Range.clip(MIN_DRIVE_SPEED,-1,1);
        double fastSpeed = Range.clip(MAX_DRIVE_SPEED,-1,1);

        /** Sets the time frame for the move to time out */
        int timeout = timeoutMS;
        ElapsedTime runtime = new ElapsedTime();


        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int deltaLF, deltaRF, deltaLB, deltaRB;
        //int maxDelta;

        int countsToMove = (int) (angle_in_degree * COUNTS_PER_DEGREE);
        deltaLF =  countsToMove;
        deltaRF = -  countsToMove;
        deltaLB =  countsToMove;
        deltaRB = -  countsToMove;

        newLeftFrontTarget = leftFront.getCurrentPosition() + deltaLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + deltaRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + deltaLB;
        newRightBackTarget = rightBack.getCurrentPosition() + deltaRB;

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);

        double power = fastSpeed;  // start from fastSpeed
        runtime.reset();
        setPowerToAllDriveMotors(power);

        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                (runtime.milliseconds() < timeout)) {
            // waiting to finish
            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)      ||
                    (Math.abs(rightFront.getTargetPosition()-rightFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN) ||
                    (Math.abs(leftBack.getTargetPosition()-leftBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)     ||
                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN ) ) {
                power = slowSpeed;
            }
            setModeToAllDriveMotors(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerToAllDriveMotors(power);

        }

        setModeToAllDriveMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAllDriveMotors(0);
    }

    private void setPowerToAllDriveMotors(double powerForAll) {
        leftFront.setPower(powerForAll);
        rightFront.setPower(powerForAll);
        leftBack.setPower(powerForAll);
        rightBack.setPower(powerForAll);
    }
    private void setModeToAllDriveMotors(DcMotor.RunMode runModeForAll) {
        leftFront.setMode(runModeForAll);
        rightFront.setMode(runModeForAll);
        leftBack.setMode(runModeForAll);
        rightBack.setMode(runModeForAll);
    }
    private void setZeroPowerBehaviorToAllDriveMotors(DcMotor.ZeroPowerBehavior zeroPowerBehaviorForAll) {
        leftFront.setZeroPowerBehavior(zeroPowerBehaviorForAll);
        rightFront.setZeroPowerBehavior(zeroPowerBehaviorForAll);
        leftBack.setZeroPowerBehavior(zeroPowerBehaviorForAll);
        rightBack.setZeroPowerBehavior(zeroPowerBehaviorForAll);
    }

    public double calPower(int currCount, int startCount, int endCount, double startPower, double endPower) {
        double power = 0;
        if (startPower == endPower) {
            power = startPower;
            TelemetryWrapper.setLine(9,String.format("calPower: startPower == endPower, power=%.1f",power));
        } else if (startPower < endPower) {  // 加速阶段
            if (((startCount <= endCount) /*正转时加速*/ && (currCount >= endCount) /*当前位置已经超过目标位置*/) ||
                    ((startCount >= endCount) /*反转时加速*/ && (currCount <= endCount) /*当前位置已经超过目标位置*/)) {
                power = endPower;
                TelemetryWrapper.setLine(9,String.format("calPower: startPower < endPower 加速，已超过目标位置, power=%.1f",power));
            } else if (((startCount <= endCount) /*正转时加速*/ && (currCount <= startCount) /*当前位置尚不到起始位置*/)
                    || ((startCount >= endCount) /*反转时加速*/ && (currCount >= startCount) /*当前位置尚不到起始位置*/)) {
                power = startPower;
                TelemetryWrapper.setLine(9,String.format("calPower: startPower < endPower 加速，尚不到起始位置, power=%.1f",power));
            } else { /* 正、反转时加速，当前位置在起始位置和目标位置之间 */
                power =  (startPower + (Math.sin(((currCount - startCount) / (endCount - startCount)) * Math.PI / 2)) * (endPower - startPower));
                TelemetryWrapper.setLine(9,String.format("calPower: startPower < endPower 加速，处起始和目标位置中间, power=%.1f",power));
            }
        } else if (endPower < startPower) {  // 减速阶段
            if (((startCount <= endCount) /*正转时减速*/ && (currCount >= endCount) /*当前位置已经超过目标位置*/) ||
                    ((startCount >= endCount) /*反转时减速*/ && (currCount <= endCount) /*当前位置已经超过目标位置*/)) {
                power = endPower;
                TelemetryWrapper.setLine(9,String.format("calPower: endPower < startPower 减速，已超过目标位置, power=%.1f",power));

            } else if (((startCount <= endCount) /*正转时减速*/ && (currCount <= startCount) /*当前位置尚不到起始位置*/)
                    || ((startCount >= endCount) /*反转时减速*/ && (currCount >= startCount) /*当前位置尚不到起始位置*/)) {
                power = startPower;
                TelemetryWrapper.setLine(9,String.format("calPower: endPower < startPower 减速，尚不到起始位置, power=%.1f",power));
            } else { /* 正、反转时减速，当前位置在起始位置和目标位置之间 */
                power = (startPower + (Math.sin(((currCount - startCount) / (endCount - startCount)) * Math.PI / 2)) * (endPower - startPower));
                TelemetryWrapper.setLine(9,String.format("calPower: endPower < startPower 减速，处起始和目标位置中间, power=%.1f",power));
            }
        }
        return power;
    }

    public int whereIsTheNumber(int i, int startInt, int endInt) {
        int result;
        if (startInt <= endInt) { // 正转
            if ( i < startInt) {
                result = -1;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: 正转 (startIn <= endInt, i < startInt, result = -1)");
            } else if ( i > endInt) {
                result = 1;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: 正转 (startIn <= endInt, i > endInt, result = 1)");
            } else {
                result = 0;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: 正转 (startIn <= endInt, i in between, result = 0)");
            }
        } else { // startInt > endInt 反转
            if ( i > startInt) {
                result = -1;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: 反转 (startIn > endInt, i > startInt, result = -1)");
            } else if ( i < endInt) {
                result = 1;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: 反转 (startIn > endInt, i < endInt, result = 1)");
            } else {
                result = 0;
                TelemetryWrapper.setLine(8,"whereIsTheNumber: 反转 (startIn > endInt, i in between, result = 0)");
            }
        }
        return result;
    }
}
