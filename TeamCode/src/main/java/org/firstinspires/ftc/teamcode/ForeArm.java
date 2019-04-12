package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Config;
import java.util.Map;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Config;
import com.qualcomm.robotcore.util.Range;


public class ForeArm {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;

    double FOREARM_UPDOWN_POWER = 1.0;
    double FOREARM_FORTHBACK_POWER = 1.0;
    int FOREARM_COUNTS_PER_UPDOWN_EFFORT = 50;
    int FOREARM_COUNTS_PER_FORTHBACK_EFFORT =50;




    HardwareMap hwMap = null;
    public ElapsedTime time = new ElapsedTime();

    public void init(HardwareMap Map, Config config) {
        hwMap = Map;
        motor1 = hwMap.get(DcMotor.class, "forearm1");
        motor2 = hwMap.get(DcMotor.class, "forearm2");
        motor3 = hwMap.get(DcMotor.class, "forearm3");

//        motor1.setDirection(DcMotor.Direction.FORWARD);
//        motor2.setDirection(DcMotor.Direction.REVERSE);
//        motor1.setPower(0);
//        motor2.setPower(0);
//
//        motor3.setDirection(DcMotor.Direction.FORWARD);
//        motor3.setPower(0);

        motor1.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FOREARM_UPDOWN_POWER = config.getDouble("forearm_updown_power", 1.0);
        FOREARM_FORTHBACK_POWER = config.getDouble("forearm_forthback_power",1.0);

        FOREARM_COUNTS_PER_UPDOWN_EFFORT = config.getInt("forearm_counts_per_updown_effort", 50);
        FOREARM_COUNTS_PER_FORTHBACK_EFFORT = config.getInt("forearm_counts_per_forthback_effort",50);


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
        motor1.setPower(FOREARM_UPDOWN_POWER);
        motor2.setPower(FOREARM_UPDOWN_POWER);
        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        motor1.setDirection(DcMotor.Direction.FORWARD);
//        motor1.setPower(-1.0 * FOREARM_UPDOWN_POWER);
//        motor2.setDirection(DcMotor.Direction.REVERSE);
//        motor2.setPower(-1.0 * FOREARM_UPDOWN_POWER);

    }

    public void moveDown() {

        int newTarget1,newTarget2;
        newTarget1  = motor1.getCurrentPosition() - FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        newTarget2  = motor2.getCurrentPosition() + FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        motor1.setPower(FOREARM_UPDOWN_POWER);
        motor2.setPower(FOREARM_UPDOWN_POWER);
        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        motor1.setDirection(DcMotor.Direction.FORWARD);
//        motor1.setPower(1.0 * FOREARM_UPDOWN_POWER);
//        motor2.setDirection(DcMotor.Direction.REVERSE);
//        motor2.setPower(1.0 * FOREARM_UPDOWN_POWER);

    }

    public void stopUpDown() {

        motor1.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
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

        motor3.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setPower(0);
    }
//    public void moveForthBack(double power) {
//        motor3.setDirection(DcMotor.Direction.FORWARD);
//        motor3.setPower(power);
//
//    }

}
