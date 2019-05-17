package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Hardware2019.*;

public class DriveTrain {

    double speedx;
    double speedy;
    double offset;

    public void init() {

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
//
//        leftFront.setPower(0);
//        rightFront.setPower(0);
//        leftBack.setPower(0);
//        rightBack.setPower(0);

    }
    public void move(double powerx, double powery, double turn){
        speedx =  powerx;
        speedy =  - powery;
        offset =  turn;
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

