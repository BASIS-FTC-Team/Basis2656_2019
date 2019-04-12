package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;


public class ForeArmTest {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;

    double FOREARM_UPDOWN_POWER = 0.5;
    double FOREARM_FORTHBACK_POWER = 0.5;
    int FOREARM_COUNTS_PER_UPDOWN_EFFORT = 10;
    int FOREARM_COUNTS_PER_FORTHBACK_EFFORT =10;
    int timeoutS = 3000;


    HardwareMap hwMap = null;
    public ElapsedTime time = new ElapsedTime();
    public ElapsedTime runtimeFor1MovingStep = new ElapsedTime();

    public void init(HardwareMap Map, Config config) {
        hwMap = Map;
        motor1 = hwMap.get(DcMotor.class, "forearm1");
        motor2 = hwMap.get(DcMotor.class, "forearm2");
        motor3 = hwMap.get(DcMotor.class, "forearm3");



        FOREARM_UPDOWN_POWER = config.getDouble("forearm_updown_power", 0.5);
        FOREARM_FORTHBACK_POWER = config.getDouble("forearm_forthback_power",0.5);

        FOREARM_COUNTS_PER_UPDOWN_EFFORT = config.getInt("forearm_counts_per_updown_effort", 10);
        FOREARM_COUNTS_PER_FORTHBACK_EFFORT = config.getInt("forearm_counts_per_forthback_effort",10);

    }

    public void prepEncoderMode() {

        motor1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);

    }


    public void moveUpDown(double power) {
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor1.setPower(power);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor2.setPower(power);
    }

    public void moveUp() {

        int newTarget1,newTarget2;
        newTarget1  = motor1.getCurrentPosition() + FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        newTarget2  = motor2.getCurrentPosition() - FOREARM_COUNTS_PER_UPDOWN_EFFORT;

        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(FOREARM_UPDOWN_POWER);
        motor2.setPower(FOREARM_UPDOWN_POWER);

//        motor1.setDirection(DcMotor.Direction.FORWARD);
//        motor1.setPower(-1.0 * FOREARM_UPDOWN_POWER);
//        motor2.setDirection(DcMotor.Direction.REVERSE);
//        motor2.setPower(-1.0 * FOREARM_UPDOWN_POWER);

    }

    public void moveDown(int timeoutS) {

        int newTarget1,newTarget2;
        newTarget1  = motor1.getCurrentPosition() - FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        newTarget2  = motor2.getCurrentPosition() + FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtimeFor1MovingStep.reset();
        motor1.setPower(-FOREARM_UPDOWN_POWER);
        motor2.setPower(FOREARM_UPDOWN_POWER);

        while (motor1.isBusy() && motor2.isBusy() && runtimeFor1MovingStep.milliseconds() <= timeoutS) {
            //looping
            TelemetryWrapper.setLine(8,"runtimeFor1MovingStep: " + runtimeFor1MovingStep.toString());
        }
        motor1.setPower(0);
        motor2.setPower(0);
//        motor1.setDirection(DcMotor.Direction.FORWARD);
//        motor1.setPower(1.0 * FOREARM_UPDOWN_POWER);
//        motor2.setDirection(DcMotor.Direction.REVERSE);
//        motor2.setPower(1.0 * FOREARM_UPDOWN_POWER);

    }

    public void stopUpDown() {

        motor1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setPower(0);
        motor2.setPower(0);

    }

    public void moveForward() {

        int newTarget3;
        newTarget3  = motor3.getCurrentPosition() + FOREARM_COUNTS_PER_FORTHBACK_EFFORT;
        motor3.setPower(FOREARM_FORTHBACK_POWER);
        motor3.setTargetPosition(newTarget3);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        motor3.setDirection(DcMotor.Direction.FORWARD);
//        motor3.setPower(1.0 * FOREARM_FORTHBACK_POWER);
    }

    public void moveBackward() {

        int newTarget3;
        newTarget3  = motor3.getCurrentPosition() - FOREARM_COUNTS_PER_FORTHBACK_EFFORT;
        motor3.setPower(FOREARM_FORTHBACK_POWER);
        motor3.setTargetPosition(newTarget3);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        motor3.setDirection(DcMotor.Direction.REVERSE);
//        motor3.setPower(1.0 * FOREARM_FORTHBACK_POWER);
    }

    public void stopForthBack() {

        motor3.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setPower(0);
    }
//    public void moveForthBack(double power) {
//        motor3.setDirection(DcMotor.Direction.FORWARD);
//        motor3.setPower(power);
//
//    }

}
