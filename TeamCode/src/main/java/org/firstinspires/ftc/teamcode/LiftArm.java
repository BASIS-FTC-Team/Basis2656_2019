package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import java.util.EventListener;
import java.util.EventListenerProxy;
import java.util.Map;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Config;
import com.qualcomm.robotcore.util.Range;


public class LiftArm {
    private DcMotor verticalMotor;
    //private DcMotor verticalMotor2;

    private DigitalChannel touchSensor;
    double LIFT_POWER = 1.0;

    int LIFT_COUNTS_PER_UPDOWN_EFFORT =50;

    double container_position = 0.20;
    HardwareMap hwMap = null;
    PID pidLiftUpDown = null;

    public ElapsedTime time = new ElapsedTime();

    public void init(HardwareMap Map, Config config, PID pidLiftUpDown) {
        hwMap = Map;
        this.pidLiftUpDown = pidLiftUpDown;
        verticalMotor = hwMap.get(DcMotor.class, "rover_elevator");
        touchSensor = hwMap.get(DigitalChannel.class, "elevatortouch");

        verticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalMotor.setDirection(DcMotor.Direction.REVERSE);

        LIFT_POWER = config.getDouble("lift_power", 1.0);
        LIFT_COUNTS_PER_UPDOWN_EFFORT = config.getInt("lift_counts_per_updown_effort", 50);

//        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        verticalMotor.setPower(0);

    }

    public void initEnc(HardwareMap Map, Config config, PID pidLiftUpDown) {
        hwMap = Map;
        this.pidLiftUpDown = pidLiftUpDown;
        verticalMotor = hwMap.get(DcMotor.class, "rover_elevator");
        touchSensor = hwMap.get(DigitalChannel.class, "elevatortouch");
//
//        verticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalMotor.setDirection(DcMotor.Direction.REVERSE);

        LIFT_POWER = config.getDouble("lift_power", 1.0);
        LIFT_COUNTS_PER_UPDOWN_EFFORT = config.getInt("lift_counts_per_updown_effort", 50);

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
//        int newTarget;
//        ElapsedTime runtime = new ElapsedTime();
//        newTarget = verticalMotor.getCurrentPosition() + LIFT_COUNTS_PER_UPDOWN_EFFORT;
//        verticalMotor.setPower(LIFT_POWER);
//        verticalMotor.setTargetPosition(newTarget);
//        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        runtime.reset();
//        while (verticalMotor.isBusy() && runtime.milliseconds() < 1000) {
//             //wait
//        }
    }

    public void moveDownEnc() {

        moveUpDownEnc(LIFT_POWER,-LIFT_COUNTS_PER_UPDOWN_EFFORT,1000);
//        int newTarget;
//        ElapsedTime runtime = new ElapsedTime();
//        newTarget = verticalMotor.getCurrentPosition() - LIFT_COUNTS_PER_UPDOWN_EFFORT;
//        verticalMotor.setPower(LIFT_POWER);
//        verticalMotor.setTargetPosition(newTarget);
//        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        runtime.reset();
//        while (verticalMotor.isBusy() && runtime.milliseconds() < 1000) {
//            // wait
//        }
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


    public void stopEnc() {

        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        verticalMotor.setPower(0);
    }


    public boolean isTouched() {
        return !touchSensor.getState();
    }
}
