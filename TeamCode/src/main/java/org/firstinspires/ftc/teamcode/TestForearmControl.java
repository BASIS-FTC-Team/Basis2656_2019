package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

@TeleOp(name="Test Forearm Control", group = "Try out")
public class TestForearmControl extends LinearOpMode {

    public DcMotor motor_l;
    public DcMotor motor_r;
    public DcMotor motor_c;
    public DcMotor motor_b;

    ElapsedTime runtime1 = new ElapsedTime();
    ElapsedTime runtime2 = new ElapsedTime();

    static double MOTOR_TICK_COUNTS = 1120;
    static double MOTOR_POWER = 0.15;
    //static double TARGET_DELTA_COEF = 0.25;
    static int INCREMENT_PER_EFFORT = 10;
    static DcMotor.ZeroPowerBehavior ZEROPOWERBEHAVIORSETTING = ZeroPowerBehavior.BRAKE;


    @Override
    public void runOpMode() {

        motor_l = hardwareMap.get(DcMotor.class, "forearm1");
        motor_r = hardwareMap.get(DcMotor.class, "forearm2");
        motor_c = hardwareMap.get(DcMotor.class, "forearm3");

        int loops0 = 0;
        int loops1 = 0;
        int loops2 = 0;

        int newTarget_l = 0;
        int newTarget_r = 0;

        TelemetryWrapper.init(telemetry,11);

        motor_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            /**
             * move up/down by encoder
             *      gamepad1.back   -- to trigger this encoder mode
             *      gamepad1.start   -- exit from this mode
             *      dpad_up         -- move up
             *      dpad_down       -- move down
             */
            if (gamepad1.back) {

                TelemetryWrapper.clear();
                TelemetryWrapper.render();
                TelemetryWrapper.setLine(0,"Moving up/down by encoder: ");

                motor_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_l.setZeroPowerBehavior(ZEROPOWERBEHAVIORSETTING);
                motor_r.setZeroPowerBehavior(ZEROPOWERBEHAVIORSETTING);
                motor_l.setPower(0);
                motor_r.setPower(0);

                TelemetryWrapper.setLine(1,"runMode:"+ " l-"+ motor_l.getMode().toString()+ " r-"+ motor_r.getMode().toString());
                TelemetryWrapper.setLine(2,"ZPB:"+ " l-"+ motor_l.getZeroPowerBehavior().toString()+ " r-"+ motor_r.getZeroPowerBehavior().toString() );
                TelemetryWrapper.setLine(3,"setPower:"+ " l-"+ motor_l.getPower()+ " r-"+ motor_r.getPower());
                TelemetryWrapper.setLine(4,"l_cPos:"+motor_l.getCurrentPosition()+ " l_tPos:"+motor_l.getTargetPosition());
                TelemetryWrapper.setLine(5,"r_cPos:"+motor_r.getCurrentPosition()+ " r_tPos:"+motor_r.getTargetPosition());

                while(!gamepad1.start) {

                    if (gamepad1.dpad_up) {

                        motor_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motor_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        TelemetryWrapper.setLine(1,"runMode:"+
                                " l-"+ motor_l.getMode().toString()+
                                " r-"+ motor_r.getMode().toString());

                        newTarget_l = motor_l.getCurrentPosition() + INCREMENT_PER_EFFORT;
                        newTarget_r = motor_r.getCurrentPosition() - INCREMENT_PER_EFFORT;

                        motor_l.setTargetPosition(newTarget_l);
                        motor_r.setTargetPosition(newTarget_r);

                        TelemetryWrapper.setLine(4,"l_cPos:"+motor_l.getCurrentPosition()+
                                " l_tPos:"+motor_l.getTargetPosition());
                        TelemetryWrapper.setLine(5,"r_cPos:"+motor_r.getCurrentPosition()+
                                " r_tPos:"+motor_r.getTargetPosition());

                        motor_l.setPower(MOTOR_POWER);
                        motor_r.setPower(MOTOR_POWER);
                        TelemetryWrapper.setLine(3,"setPower:"+
                                " l-"+ motor_l.getPower()+
                                " r-"+ motor_r.getPower());

                        runtime1.reset();
                        TelemetryWrapper.setLine(6,"Runtime1:"+runtime1.toString());

                        motor_l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor_r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        TelemetryWrapper.setLine(1,"runMode:"+
                                " l-"+ motor_l.getMode().toString()+
                                " r-"+ motor_r.getMode().toString());

                        while ((motor_l.isBusy() && motor_r.isBusy()) && runtime1.milliseconds() < 2000) {
                            // waiting
                            TelemetryWrapper.setLine(4,"l_cPos:"+motor_l.getCurrentPosition()+
                                    " l_tPos:"+motor_l.getTargetPosition());
                            TelemetryWrapper.setLine(5,"r_cPos:"+motor_r.getCurrentPosition()+
                                    " r_tPos:"+motor_r.getTargetPosition());
                            TelemetryWrapper.setLine(6,"(WaitingLoop)Runtime1:"+runtime1.toString());

                        }

                        sleep(100);

                        runtime1.reset();
                        TelemetryWrapper.setLine(6,"Runtime1:"+runtime1.toString());

                        motor_l.setPower(0);
                        motor_r.setPower(0);
                        TelemetryWrapper.setLine(3,"setPower:"+
                                " l-"+ motor_l.getPower()+
                                " r-"+ motor_r.getPower());

                        motor_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motor_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        TelemetryWrapper.setLine(1,"runMode:"+
                                " l-"+ motor_l.getMode().toString()+
                                " r-"+ motor_r.getMode().toString());
                    }

                    if (gamepad1.dpad_down) {

                        motor_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motor_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        TelemetryWrapper.setLine(1,"runMode:"+
                                " l-"+ motor_l.getMode().toString()+
                                " r-"+ motor_r.getMode().toString());

                        newTarget_l = motor_l.getCurrentPosition() - INCREMENT_PER_EFFORT;
                        newTarget_r = motor_r.getCurrentPosition() + INCREMENT_PER_EFFORT;
                        motor_l.setTargetPosition(newTarget_l);
                        motor_r.setTargetPosition(newTarget_r);
                        TelemetryWrapper.setLine(4,"l_cPos:"+motor_l.getCurrentPosition()+
                                " l_tPos:"+motor_l.getTargetPosition());
                        TelemetryWrapper.setLine(5,"r_cPos:"+motor_r.getCurrentPosition()+
                                " r_tPos:"+motor_r.getTargetPosition());

                        motor_l.setPower(MOTOR_POWER);
                        motor_r.setPower(+MOTOR_POWER);
                        TelemetryWrapper.setLine(3,"setPower:"+
                                " l-"+ motor_l.getPower()+
                                " r-"+ motor_r.getPower());

                        runtime1.reset();
                        TelemetryWrapper.setLine(6,"Runtime1:"+runtime1.toString());

                        motor_l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor_r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        TelemetryWrapper.setLine(1,"runMode:"+
                                " l-"+ motor_l.getMode().toString()+
                                " r-"+ motor_r.getMode().toString());

                        while ((motor_l.isBusy() || motor_r.isBusy()) && runtime1.milliseconds() < 2000) {
                            // waiting
                            TelemetryWrapper.setLine(4,"l_cPos:"+motor_l.getCurrentPosition()+
                                    " l_tPos:"+motor_l.getTargetPosition());
                            TelemetryWrapper.setLine(5,"r_cPos:"+motor_r.getCurrentPosition()+
                                    " r_tPos:"+motor_r.getTargetPosition());
                            TelemetryWrapper.setLine(6,"(WaitingLoop)Runtime1:"+runtime1.toString());

                        }

                        sleep(100);

                        runtime1.reset();
                        TelemetryWrapper.setLine(6,"Runtime1:"+runtime1.toString());

                        motor_l.setPower(0);
                        motor_r.setPower(0);
                        TelemetryWrapper.setLine(3,"setPower:"+
                                " l-"+ motor_l.getPower()+
                                " r-"+ motor_r.getPower());

                        motor_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motor_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        TelemetryWrapper.setLine(1,"runMode:"+
                                " l-"+ motor_l.getMode().toString()+
                                " r-"+ motor_r.getMode().toString());

                    }

                    TelemetryWrapper.setLine(4,"l_cPos:"+motor_l.getCurrentPosition()+
                            " l_tPos:"+motor_l.getTargetPosition());
                    TelemetryWrapper.setLine(5,"r_cPos:"+motor_r.getCurrentPosition()+
                            " r_tPos:"+motor_r.getTargetPosition());
                }
            }

            /**
             * move up/down without encoder
             *      gamepad1.left_bumper -- to trigger this mode (without encoder
             *
             * */
            if (gamepad1.dpad_left) {

                TelemetryWrapper.clear();
                TelemetryWrapper.render();
                TelemetryWrapper.setLine(0,"Moving up/down WITHOUT encoder: ");

                motor_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_l.setDirection(DcMotorSimple.Direction.FORWARD);
                motor_r.setDirection(DcMotorSimple.Direction.REVERSE);

                TelemetryWrapper.setLine(1,"runMode:"+
                        " l-"+ motor_l.getMode().toString()+
                        " r-"+ motor_r.getMode().toString());
                TelemetryWrapper.setLine(2,"Direction:"+
                        " l-"+ motor_l.getDirection().toString()+
                        " r-"+ motor_r.getDirection().toString() );

                double tempPower = 0.0;

                while (!gamepad1.dpad_right) {
                    tempPower = gamepad1.right_stick_y;
                    motor_l.setPower(tempPower);
                    motor_r.setPower(tempPower);
                    TelemetryWrapper.setLine(2,"getPower():"+
                            " l-"+ motor_l.getPower()+
                            " r-"+ motor_r.getPower() );
                    TelemetryWrapper.setLine(3,"setPower(): " + tempPower );
                    sleep(50);
                }
            }



            /** Press gamepad1.left_bumper to start configuring the parameter MOTOR_POWER:
             *
             *      dpad_up     to increase Motor Power
             *      dpad_down   to decrease Motor Power
             *      back        to cyclicaly set INCREMENT_PER_EFFORT by step of 10
             *      b           for EROPOWERBEHAVIORSETTING
             *      x           for MOTOR_TICK_COUNTS
             *
             *      right_bumper to exit setting process
             *
             */
            if (gamepad1.left_bumper) {

                TelemetryWrapper.clear();
                TelemetryWrapper.render();
                TelemetryWrapper.setLine(0,"Setting the key parameters: ");
                TelemetryWrapper.setLine(7,"MOTOR_POWER: " + MOTOR_POWER);
                //TelemetryWrapper.setLine(8,"TARGET_DELTA_COEF: " + TARGET_DELTA_COEF);
                TelemetryWrapper.setLine(8,"INCREMENT_PER_EFFORT: " + INCREMENT_PER_EFFORT);
                TelemetryWrapper.setLine(9,"ZEROPOWERBEHAVIOR: " + ZEROPOWERBEHAVIORSETTING.toString());
                TelemetryWrapper.setLine(10,"MOTOR_TICK_COUNTS: " + MOTOR_TICK_COUNTS);

                int loops3=0;
                while (!gamepad1.right_bumper) {
                    loops3 ++;
                    TelemetryWrapper.setLine(1,"Loops when setting: " + loops3);

                    /** Set Motor Power -- gamepad1.dpad_up and gamepad1.dpad_down */
                    if (gamepad1.dpad_up) {
                        MOTOR_POWER += 0.01;
                        if (MOTOR_POWER >1.0) MOTOR_POWER = 1;
                        if (MOTOR_POWER < 0.005 && MOTOR_POWER > -0.005) MOTOR_POWER = 0.0;
                        TelemetryWrapper.setLine(7,"MOTOR_POWER: " + MOTOR_POWER);
                        sleep(100);
                    }
                    if (gamepad1.dpad_down) {
                        MOTOR_POWER -= 0.01;
                        if (MOTOR_POWER < -1.0) MOTOR_POWER = -1.0;
                        if (MOTOR_POWER < 0.005 && MOTOR_POWER > -0.005) MOTOR_POWER = 0.0;
                        TelemetryWrapper.setLine(7,"MOTOR_POWER: " + MOTOR_POWER);
                        sleep(100);
                    }

                    /** Set INCREMENT_PER_EFFORT -- gamepad1.back */
                    if (gamepad1.back) {
                        INCREMENT_PER_EFFORT += 10;
                        if (INCREMENT_PER_EFFORT > 200) {
                            INCREMENT_PER_EFFORT = -200;
                        }
                        TelemetryWrapper.setLine(8,"INCREMENT_PER_EFFORT: " + INCREMENT_PER_EFFORT);
                        sleep(200);
                    }

                    /** Set zeroPowerBehavior -- gamepad1.b */
                    if (gamepad1.b) {
                        if (ZEROPOWERBEHAVIORSETTING == ZeroPowerBehavior.BRAKE) {
                            ZEROPOWERBEHAVIORSETTING = ZeroPowerBehavior.FLOAT;
                        } else {
                            ZEROPOWERBEHAVIORSETTING = ZeroPowerBehavior.BRAKE;
                        }
                        TelemetryWrapper.setLine(9,"ZEROPOWERBEHAVIOR: " + ZEROPOWERBEHAVIORSETTING.toString());
                        sleep(200);
                    }

                    /** Set motor tick counts -- gamepad1.x */
                    if (gamepad1.x) {
                        if (MOTOR_TICK_COUNTS == 1120) {
                            MOTOR_TICK_COUNTS = 1440;
                        } else if (MOTOR_TICK_COUNTS == 1440){
                            MOTOR_TICK_COUNTS = 560;
                        } else if (MOTOR_TICK_COUNTS == 560){
                            MOTOR_TICK_COUNTS = 720;
                        }else if (MOTOR_TICK_COUNTS == 720){
                            MOTOR_TICK_COUNTS = 1120;
                        }
                        TelemetryWrapper.setLine(10,"MOTOR_TICK_COUNTS: " + MOTOR_TICK_COUNTS);
                        sleep(200);
                    }
                }
            }

            TelemetryWrapper.setLine(4,"l_cPos:"+motor_l.getCurrentPosition()+
                    " l_tPos:"+motor_l.getTargetPosition());
            TelemetryWrapper.setLine(5,"r_cPos:"+motor_r.getCurrentPosition()+
                    " r_tPos:"+motor_r.getTargetPosition());

        }
    }
}
