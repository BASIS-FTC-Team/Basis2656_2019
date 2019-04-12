package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Map;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Config;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

import static java.lang.Math.*;

public class DriveTrainByEncoder
{

    DcMotor leftFront   = null;
    DcMotor rightFront   = null;
    DcMotor leftBack  = null;
    DcMotor rightBack  = null;
    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();


    double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    double     WHEEL_DIAMETER_MM       = 100.0 ;     // For figuring circumference
    double     WHEEL_DIAGONAL_DISTANCE  = 450.0 ;     // for the  distance between two diagonal wheel
    double     COUNTS_PER_MM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);
    double     DEGREE_CORRECTION       = 1.543;
    double     LINE_CORRECTION         = 1.0;
    double     COUNTS_PER_DEGREE       = WHEEL_DIAGONAL_DISTANCE / WHEEL_DIAMETER_MM  * COUNTS_PER_MOTOR_REV / 360. * DEGREE_CORRECTION;
    double     XY_CORRECTION           = 1.2;

//    static final double     DRIVE_SPEED             = 0.6;
//    static final double     TURN_SPEED              = 0.5;

    public void init(HardwareMap Map, Config config) {
        hwMap = Map;
// not use the front drive temporary
        leftFront = hwMap.get(DcMotor.class, "fl_drive");
        rightFront = hwMap.get(DcMotor.class, "fr_drive");
        leftBack = hwMap.get(DcMotor.class, "rl_drive");
        rightBack = hwMap.get(DcMotor.class, "rr_drive");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        //read properties from file.....
        //counts_per_motor_rev = 1440
        //drive_gear_reduction = 1.0
        //wheel_diagonal_distance = 450.
        //wheel_diameter = 100.
        COUNTS_PER_MOTOR_REV = config.getDouble("counts_per_motor_rev", 1440);
        DRIVE_GEAR_REDUCTION = config.getDouble("drive_gear_reduction",1.0);
        WHEEL_DIAGONAL_DISTANCE = config.getDouble("wheel_diagonal_distance", 450.);
        WHEEL_DIAMETER_MM = config.getDouble("wheel_diameter", 100.);
        DEGREE_CORRECTION = config.getDouble("degree_correction", 1.543);
        LINE_CORRECTION = config.getDouble("line_correction", 1.);
        XY_CORRECTION = config.getDouble("xy_correction", 1.2);
        COUNTS_PER_MM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415) * LINE_CORRECTION;
        COUNTS_PER_DEGREE       = WHEEL_DIAGONAL_DISTANCE / WHEEL_DIAMETER_MM  * COUNTS_PER_MOTOR_REV / 360. * DEGREE_CORRECTION;


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void stop(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    /*
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
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();

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
        stop();

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    //
    //  Using the encoder mode, in PID control to get accurate speed controlling.
    //
    //
    public void move(double powerx, double powery, double turn){
        double speedx = powerx;
        double speedy = powery;
        double offset = turn;
        leftFront.setPower(Range.clip(speedy-speedx+offset,-1,1));
        rightFront.setPower(Range.clip(speedy+speedx-offset,-1,1));
        leftBack.setPower(Range.clip(speedy+speedx+offset,-1,1));
        rightBack.setPower(Range.clip(speedy-speedx-offset,-1,1));
    }
}
