package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import java.util.Map;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Config;
import com.qualcomm.robotcore.util.Range;


public class RoverArm {
    private DcMotor verticalMotor;
    //private DcMotor verticalMotor2;

    private DigitalChannel touchSensor;
    double LIFT_POWER = 1.0;

    int LIFT_COUNTS_PER_UPDOWN_EFFORT =50;

    double container_position = 0.20;
    HardwareMap hwMap = null;
    public ElapsedTime time = new ElapsedTime();

    public void init(HardwareMap Map, Config config) {
        hwMap = Map;
        verticalMotor = hwMap.get(DcMotor.class, "rover_elevator");
        touchSensor = hwMap.get(DigitalChannel.class, "elevatortouch");
        //verticalMotor.setDirection(DcMotor.Direction.REVERSE);

        LIFT_POWER = config.getDouble("lift_power", 1.0);
        LIFT_COUNTS_PER_UPDOWN_EFFORT = config.getInt("lift_counts_per_updown_effort", 50);

        //verticalMotor.setDirection(DcMotor.Direction.FORWARD);
        verticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        verticalMotor.setPower(0);

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

        int newTarget;
        newTarget  = verticalMotor.getCurrentPosition() + LIFT_COUNTS_PER_UPDOWN_EFFORT;
        verticalMotor.setPower(LIFT_POWER);
        verticalMotor.setTargetPosition(newTarget);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//        verticalMotor.setDirection(DcMotor.Direction.FORWARD);
//        verticalMotor.setPower(-1.0 * LIFT_POWER);
    }

    public void moveDown() {

        int newTarget;
        newTarget  = verticalMotor.getCurrentPosition() - LIFT_COUNTS_PER_UPDOWN_EFFORT;
        verticalMotor.setPower(LIFT_POWER);
        verticalMotor.setTargetPosition(newTarget);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        verticalMotor.setDirection(DcMotor.Direction.FORWARD);
//        verticalMotor.setPower(LIFT_POWER);
    }

    public void stop() {

        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        verticalMotor.setPower(0);

//        verticalMotor.setDirection(DcMotor.Direction.FORWARD);
//        verticalMotor.setPower(0);
    }

    public boolean isTouched() {
        return !touchSensor.getState();
    }
}
