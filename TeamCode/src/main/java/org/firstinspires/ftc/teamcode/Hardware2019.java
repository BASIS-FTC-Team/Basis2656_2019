package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware2019 {

    //////// Robot Drive  //////////////////////////////
    /** Motors for robot drive, controlling the wheels' rotation */
    public static DcMotor leftFront;
    public static DcMotor rightFront;
    public static DcMotor leftBack;
    public static DcMotor rightBack;

    /////// ForeArm ////////////////////////////////////
    /** Motors for forearm up/down */
    public static DcMotor motor1;
    public static DcMotor motor2;
    /** Motor for forearm forth/back */
    public static DcMotor motor3;
    /** Sensors for foreArm */
    public static DigitalChannel touchSensor1;
    //public static DigitalChannel touchSensor2;
    /** Servos for mineral collectors */
    public static CRServo servo1;  // Rotate to wipe in
    public static Servo servo2;  // Open/close the mineral holder


    //////// Lift //////////////////////////////////////
    /** Motor for lift */
    public static DcMotor verticalMotor;
    /** Sensor for lift */
    public static DigitalChannel touchSensor;


    public static void init(HardwareMap hwMap) {

        //////// For robot drive:
        leftFront = hwMap.get(DcMotor.class, "fl_drive");
        rightFront = hwMap.get(DcMotor.class, "fr_drive");
        leftBack = hwMap.get(DcMotor.class, "rl_drive");
        rightBack = hwMap.get(DcMotor.class, "rr_drive");

//        //////// For ForeArm ///////
        motor1 = hwMap.get(DcMotor.class, "forearm1");
        motor2 = hwMap.get(DcMotor.class, "forearm2");
        motor3 = hwMap.get(DcMotor.class, "forearm3");
        touchSensor1 = hwMap.get(DigitalChannel.class, "forearm_touchsensor1");
//        //touchSensor2 = hwMap.get(DigitalChannel.class, "forearm_touchsensor2");
//
//        // For mineral collector
        servo1 = hwMap.crservo.get("wipe_servo");
        servo2 = hwMap.get(Servo.class, "turn_servo");
//
//        //////// For Lift  ///////
        verticalMotor = hwMap.get(DcMotor.class, "rover_elevator");
        touchSensor = hwMap.get(DigitalChannel.class, "elevatortouch");

    }




}
