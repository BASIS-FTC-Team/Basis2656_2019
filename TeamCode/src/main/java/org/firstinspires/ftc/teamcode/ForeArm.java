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

    private PID pid = new PID(0.5,0.05,0.05,0.3,0.5,-0.8,0.8);

    double FOREARM_UPDOWN_POWER = 1.0;
    double FOREARM_FORTHBACK_POWER = 1.0;
    int FOREARM_COUNTS_PER_UPDOWN_EFFORT = 50;
    int FOREARM_COUNTS_PER_FORTHBACK_EFFORT =50;
    int FOREARM_COUNTS_AUTODOWN = 300;
    int FOREARM_COUNTS_AUTOUP = 300;

    int COUNTS_PER_REV_FOR_UPDOWN = 1120;
    int GEAR_REDUCTION_FOR_UPDOWN_MOTOR = 6; // 20:40 * 30:90 = 1:6

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

        motor1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FOREARM_UPDOWN_POWER = config.getDouble("forearm_updown_power", 0.8);
        FOREARM_FORTHBACK_POWER = config.getDouble("forearm_forthback_power",0.8);

        FOREARM_COUNTS_PER_UPDOWN_EFFORT = config.getInt("forearm_counts_per_updown_effort", 50);
        FOREARM_COUNTS_PER_FORTHBACK_EFFORT = config.getInt("forearm_counts_per_forthback_effort",50);


    }

    public void moveUp() {

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor1.setPower(-1.0 * FOREARM_UPDOWN_POWER);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor2.setPower(-1.0 * FOREARM_UPDOWN_POWER);

    }
    public void moveDown() {
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor1.setPower(1.0 * FOREARM_UPDOWN_POWER);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor2.setPower(1.0 * FOREARM_UPDOWN_POWER);
    }
    public void moveUpDown(double power) {
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor1.setPower(power);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor2.setPower(power);
    }

    public void moveForward() {
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor3.setPower(1.0 * FOREARM_FORTHBACK_POWER);
    }
    public void moveBackward() {

        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor3.setPower(1.0 * FOREARM_FORTHBACK_POWER);
    }
    public void stopForthBack() {
        motor3.setPower(0);
    }

    public void moveForwardEnc() {

        int newTarget3;
        newTarget3  = motor3.getCurrentPosition() + FOREARM_COUNTS_PER_FORTHBACK_EFFORT;
        motor3.setPower(FOREARM_FORTHBACK_POWER);
        motor3.setTargetPosition(newTarget3);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void moveBackwardEnc() {

        int newTarget3;
        newTarget3  = motor3.getCurrentPosition() - FOREARM_COUNTS_PER_FORTHBACK_EFFORT;
        motor3.setPower(FOREARM_FORTHBACK_POWER);
        motor3.setTargetPosition(newTarget3);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void stopForthBackEnc() {

        motor3.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setPower(0);
    }


    public void moveUpEnc() {

        int newTarget1,newTarget2;
        ElapsedTime runtime = new ElapsedTime();

        newTarget1  = motor1.getCurrentPosition() + FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        newTarget2  = motor2.getCurrentPosition() - FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        motor1.setPower(FOREARM_UPDOWN_POWER);
        motor2.setPower(FOREARM_UPDOWN_POWER);
        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        while ((motor1.isBusy() || motor2.isBusy())  ) {
            //wait
        }


    }
    public void moveDownEnc() {

        int newTarget1,newTarget2;
        ElapsedTime runtime = new ElapsedTime();

        newTarget1  = motor1.getCurrentPosition() - FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        newTarget2  = motor2.getCurrentPosition() + FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        motor1.setPower(FOREARM_UPDOWN_POWER);
        motor2.setPower(FOREARM_UPDOWN_POWER);
        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        while (motor1.isBusy() || motor2.isBusy())   {
            //wait
        }
    }
    public void stopUpDownEnc() {
        motor1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setPower(0);
        motor2.setPower(0);
    }
    public void automoveDownEnc(int timeoutMS, boolean opModeIsActive) {
        int newTarget1,newTarget2;
        ElapsedTime runtime = new ElapsedTime();

        newTarget1  = motor1.getCurrentPosition() - FOREARM_COUNTS_AUTODOWN;
        newTarget2  = motor2.getCurrentPosition() + FOREARM_COUNTS_AUTODOWN;
        motor1.setPower(FOREARM_UPDOWN_POWER);
        motor2.setPower(FOREARM_UPDOWN_POWER);
        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        while ((motor1.isBusy() && motor2.isBusy()) && (runtime.milliseconds()< timeoutMS) && (opModeIsActive) ) {
            //wait
        }
    }
    public void automoveUpEnc(double targetSpeed, double moveAngle) {
        /**
         * targetSpeed: is target ANGULAR speed in DEGREEs
         * moveAngle: is the angle the ForeArm is going to move up/down automatically
         *              - positive for moving down
         *              - negative for moving up
         * opModeIsActive: is
         */
        int newTarget1,newTarget2;
        ElapsedTime runtime = new ElapsedTime();

        double t1 = 0;
        double t2 = 0;
        double dt = 0;
        int pos1 = 0;
        int pos2 = 0;
        double actualSpeed = 0;
        double power = 0;

        int MOVE_COUNTS = (int)( (moveAngle / 360) * COUNTS_PER_REV_FOR_UPDOWN * GEAR_REDUCTION_FOR_UPDOWN_MOTOR);
        newTarget1  = motor1.getCurrentPosition() - MOVE_COUNTS;
        newTarget2  = motor2.getCurrentPosition() + MOVE_COUNTS;

        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();

        t1 = runtime.milliseconds();
        pos1 = motor1.getCurrentPosition();

        power = FOREARM_UPDOWN_POWER;
        motor1.setPower(power);
        motor2.setPower(power);

        while ((motor1.isBusy() && motor2.isBusy())) {
            //wait
            t2 = runtime.seconds();
            pos2 = motor1.getCurrentPosition();
            dt = t2 - t1;
            actualSpeed = ((((pos2 - pos1) / COUNTS_PER_REV_FOR_UPDOWN) * 360 ) /
                            GEAR_REDUCTION_FOR_UPDOWN_MOTOR ) / (t2 - t1);
            power += pid.update(targetSpeed, actualSpeed, dt);
            motor1.setPower(power);
            motor2.setPower(power);
            pos1 = pos2;
            t1 = t2;
        }
    }



//    public void moveForthBack(double power) {
//        motor3.setDirection(DcMotor.Direction.FORWARD);
//        motor3.setPower(power);
//
//    }

}
