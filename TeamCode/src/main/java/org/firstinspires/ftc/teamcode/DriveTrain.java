package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Map;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Config;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;


public class DriveTrain {
    DcMotor leftFront   = null;
    DcMotor rightFront   = null;
    DcMotor leftBack  = null;
    DcMotor rightBack  = null;
    double speedx;
    double speedy;
    double offset;
    HardwareMap hwMap = null;

    public void init(HardwareMap Map, Config config) {
        hwMap = Map;
// not use the front drive temporary
        leftFront = hwMap.get(DcMotor.class, "fl_drive");
        rightFront = hwMap.get(DcMotor.class, "fr_drive");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setPower(0);
        rightFront.setPower(0);

        leftBack = hwMap.get(DcMotor.class, "rl_drive");
        rightBack = hwMap.get(DcMotor.class, "rr_drive");
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }
    public void move(double powerx, double powery, double turn){
        speedx = powerx;
        speedy = powery;
        offset = turn;
        leftFront.setPower(Range.clip(speedy-speedx+offset,-1,1));
        rightFront.setPower(Range.clip(speedy+speedx-offset,-1,1));
        leftBack.setPower(Range.clip(speedy+speedx+offset,-1,1));
        rightBack.setPower(Range.clip(speedy-speedx-offset,-1,1));
    }


    public void stop(){
        move(0,0,0);
    }

    /**
     * Robot motion through 4 Mecanum wheels controlled by gamepad.left_stick_x, left_stick_y, right_stick_x
     * The left joystick is used to translate the robot, while the right joystick controls the rotation of the robot.
     * @param drive_x = - left_stick_x
     * @param drive_y = left_stick_y
     * @param turn    = - right_stick_x
     */
    public void moveFree(double drive_x, double drive_y, double turn) {

        double r = Math.hypot(drive_x,drive_y);
        double robotAngle = Math.atan2(drive_y,drive_x) - Math.PI / 4;
        double v1 = r * Math.cos(robotAngle) + turn;
        double v2 = r * Math.sin(robotAngle) - turn;
        double v3 = r * Math.sin(robotAngle) + turn;
        double v4 = r * Math.cos(robotAngle) - turn;
        double maxV = Math.max(Math.max(Math.abs(v1),Math.abs(v2)),Math.max(Math.abs(v3),Math.abs(v4)));
        if (maxV > 1) {
            v1 /= maxV;
            v2 /= maxV;
            v3 /= maxV;
            v4 /= maxV;
        }


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);
    }

}

