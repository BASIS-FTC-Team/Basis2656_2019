package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Map;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Config;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

import static java.lang.Math.*;

public class DriveTrainByEncoder {

    DcMotor leftFront   = null;
    DcMotor rightFront   = null;
    DcMotor leftBack  = null;
    DcMotor rightBack  = null;
    HardwareMap hwMap = null;

    private ElapsedTime runtime = new ElapsedTime();

    double     COUNTS_PER_MOTOR_REV     = 1440 ;    // eg: TETRIX Motor Encoder
    double     DRIVE_GEAR_REDUCTION     = 1.0 ;     // This is < 1.0 if geared UP
    double     WHEEL_DIAMETER_MM        = 100.0 ;     // For figuring circumference
    /** for the  distance between two diagonal wheel */
    //double     WHEEL_DIAGONAL_DISTANCE  = 490.0 ;     // the prototype robot
    double     WHEEL_DIAGONAL_DISTANCE  = 464.2 ;     // the competition robot ( wheel distances: sqrt( 350mm ^ 2 * 305mm ^ 2) = 464.0

    double     DEGREE_CORRECTION        = 1.543;
    double     LINE_CORRECTION          = 1.0;
    double     XY_CORRECTION            = 1.2;

    double     COUNTS_PER_MM            = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);
    double     COUNTS_PER_DEGREE        = WHEEL_DIAGONAL_DISTANCE / WHEEL_DIAMETER_MM  * COUNTS_PER_MOTOR_REV / 360. * DEGREE_CORRECTION;

    // MIN_DRIVE_SPEED should be less than or equal to MAX_DRIVE_SPEED
    double MIN_DRIVE_SPEED              = 0.2;
    double MAX_DRIVE_SPEED              = 0.9;

    int    COUNTS_THRESHOLD_FOR_SLOWDOWN = 50;


//    static final double     DRIVE_SPEED             = 0.6;
//    static final double     TURN_SPEED              = 0.5;

    public void init(HardwareMap Map, Config config) {
        hwMap = Map;

        leftFront = hwMap.get(DcMotor.class, "fl_drive");
        rightFront = hwMap.get(DcMotor.class, "fr_drive");
        leftBack = hwMap.get(DcMotor.class, "rl_drive");
        rightBack = hwMap.get(DcMotor.class, "rr_drive");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        setPowerToAll(0);
        setConfigurations(config);
        setModeToAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeToAll(DcMotor.RunMode.RUN_USING_ENCODER);

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
        setModeToAll(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();

        setZeroPowerBehaviorToAll(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(Range.clip(abs(speed*deltaLF/maxDelta),0,1));
        rightFront.setPower(Range.clip(abs(speed*deltaRF/maxDelta),0,1));
        leftBack.setPower(Range.clip(abs(speed*deltaLB/maxDelta),0,1));
        rightBack.setPower(Range.clip(abs(speed*deltaRB/maxDelta),0,1));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
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
        setPowerToAll(0);

        // Turn off RUN_TO_POSITION
        setModeToAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    public void stop(){ setPowerToAll(0); }


    /** The following are totally new methods for encoder-driving added by J.TU 17 April, 2019 */
    public void initEnc(HardwareMap Map, Config config) {
        hwMap = Map;

        leftFront = hwMap.get(DcMotor.class, "fl_drive");
        rightFront = hwMap.get(DcMotor.class, "fr_drive");
        leftBack = hwMap.get(DcMotor.class, "rl_drive");
        rightBack = hwMap.get(DcMotor.class, "rr_drive");

        setConfigurations(config);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        setModeToAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAll(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAll(0);
    }

    /**
     *      speed: the power used for the driving moters
     *      dist: distance to move forward or backward in millimeters
     *              - positive for forwards
     *              - negative for backwards
     *      timeoutMS: timeout setting for the move (in milliseconds)
     */
    public void moveForthBackEnc(double speed, double dist, int timeoutMS ) {

        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int deltaLF, deltaRF, deltaLB, deltaRB;
        //int maxDelta;
        int timeout = timeoutMS;
        ElapsedTime runtime = new ElapsedTime();

        int countsToMove = (int) ( dist * COUNTS_PER_MM );
        deltaLF = countsToMove;
        deltaRF = countsToMove;
        deltaLB = countsToMove;
        deltaRB = countsToMove;

        newLeftFrontTarget = leftFront.getCurrentPosition() + deltaLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + deltaRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + deltaLB;
        newRightBackTarget = rightBack.getCurrentPosition() + deltaRB;

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        setModeToAll(DcMotor.RunMode.RUN_TO_POSITION);

        double power = Range.clip(speed, -1,1);
        runtime.reset();
        setPowerToAll(power);


        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                ( runtime.milliseconds() < timeout ) ) {
            // waiting to finish
            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<30)      ||
                    (Math.abs(rightFront.getTargetPosition()-rightFront.getCurrentPosition())<30) ||
                    (Math.abs(leftBack.getTargetPosition()-leftBack.getCurrentPosition())<30)     ||
                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<30 ) ) {
                power = 0.2;
            }
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);
            //TelemetryWrapper.setLine(3, "" + leftFront.getCurrentPosition());
        }

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);


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

        /** Calculate the targets */
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int deltaLF, deltaRF, deltaLB, deltaRB;
        int countsToMove = (int) (  dist * COUNTS_PER_MM * XY_CORRECTION );
        deltaLF = - countsToMove;
        deltaRF =   countsToMove;
        deltaLB =   countsToMove;
        deltaRB = - countsToMove;
        newLeftFrontTarget = leftFront.getCurrentPosition() + deltaLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + deltaRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + deltaLB;
        newRightBackTarget = rightBack.getCurrentPosition() + deltaRB;

        /** Sets the new targets */
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        setModeToAll(DcMotor.RunMode.RUN_TO_POSITION);

        double power = fastSpeed;  // start from fastSpeed
        runtime.reset();
        setPowerToAll(power);

        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                (runtime.milliseconds() < timeout)) {
            // waiting to finish

            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)      ||
                    (Math.abs(rightFront.getTargetPosition()-rightFront.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN) ||
                    (Math.abs(leftBack.getTargetPosition()-leftBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN)     ||
                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<COUNTS_THRESHOLD_FOR_SLOWDOWN ) ) {
                power = slowSpeed;
            }
            setModeToAll(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerToAll(power);
        }
        setModeToAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAll(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAll(0);

    }

    /** Spinning clock-wise or counter-clock-wise
     *      speed: power used for motors
     *      angle_in_degree: the size of the angle the robot will turn
     *                          - positve for turning clock-wise
     *                          - negative for turning counter-clock-wise
     *      timeoutMS: timeout setting for the move (in millisecond)
     */
    public void spinEnc(double speed, double angle_in_degree, int timeoutMS ) {

        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        int deltaLF, deltaRF, deltaLB, deltaRB;
        //int maxDelta;
        int timeout = timeoutMS;

        ElapsedTime runtime = new ElapsedTime();

        int countsToMove = (int) (angle_in_degree * COUNTS_PER_DEGREE);
        deltaLF = - countsToMove;
        deltaRF =   countsToMove;
        deltaLB = - countsToMove;
        deltaRB =   countsToMove;

        newLeftFrontTarget = leftFront.getCurrentPosition() + deltaLF;
        newRightFrontTarget = rightFront.getCurrentPosition() + deltaRF;
        newLeftBackTarget = leftBack.getCurrentPosition() + deltaLB;
        newRightBackTarget = rightBack.getCurrentPosition() + deltaRB;

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        setModeToAll(DcMotor.RunMode.RUN_TO_POSITION);

        double power = Range.clip(speed, -1,1);
        runtime.reset();
        setPowerToAll(power);

        while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) &&
                (runtime.milliseconds() < timeout)) {
            // waiting to finish
            if ( (Math.abs(leftFront.getTargetPosition()-leftFront.getCurrentPosition())<50)      ||
                    (Math.abs(rightFront.getTargetPosition()-rightFront.getCurrentPosition())<50) ||
                    (Math.abs(leftBack.getTargetPosition()-leftBack.getCurrentPosition())<50)     ||
                    (Math.abs(rightBack.getTargetPosition()-rightBack.getCurrentPosition())<50 ) ) {
                power = 0.2;
            }
            setModeToAll(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerToAll(power);

        }

        setModeToAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setZeroPowerBehaviorToAll(DcMotor.ZeroPowerBehavior.BRAKE);
        setPowerToAll(0);
    }

    private void setPowerToAll(double powerForAll) {
        leftFront.setPower(powerForAll);
        rightFront.setPower(powerForAll);
        leftBack.setPower(powerForAll);
        rightBack.setPower(powerForAll);
    }
    private void setModeToAll(DcMotor.RunMode runModeForAll) {
        leftFront.setMode(runModeForAll);
        rightFront.setMode(runModeForAll);
        leftBack.setMode(runModeForAll);
        rightBack.setMode(runModeForAll);
    }
    private void setZeroPowerBehaviorToAll(DcMotor.ZeroPowerBehavior zeroPowerBehaviorForAll) {
        leftFront.setZeroPowerBehavior(zeroPowerBehaviorForAll);
        rightFront.setZeroPowerBehavior(zeroPowerBehaviorForAll);
        leftBack.setZeroPowerBehavior(zeroPowerBehaviorForAll);
        rightBack.setZeroPowerBehavior(zeroPowerBehaviorForAll);
    }
    private void setConfigurations(Config config) {

        COUNTS_PER_MOTOR_REV = config.getDouble("counts_per_motor_rev", 1440);
        DRIVE_GEAR_REDUCTION = config.getDouble("drive_gear_reduction",1.0);
        WHEEL_DIAGONAL_DISTANCE = config.getDouble("wheel_diagonal_distance", 450.);
        WHEEL_DIAMETER_MM = config.getDouble("wheel_diameter", 100.);
        DEGREE_CORRECTION = config.getDouble("degree_correction", 1.543);
        LINE_CORRECTION = config.getDouble("line_correction", 1.0);
        XY_CORRECTION = config.getDouble("xy_correction", 1.2);
        COUNTS_PER_MM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415) * LINE_CORRECTION;
        COUNTS_PER_DEGREE       = WHEEL_DIAGONAL_DISTANCE / WHEEL_DIAMETER_MM  * COUNTS_PER_MOTOR_REV / 360. * DEGREE_CORRECTION;

        MIN_DRIVE_SPEED = config.getDouble("min_drive_speed",0.2);
        MAX_DRIVE_SPEED = config.getDouble("max_dirve_speed",0.9);

    }
}
