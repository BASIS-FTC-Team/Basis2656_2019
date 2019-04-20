package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

@TeleOp(name="Test Motor Encoder",group = "Test")
@Disabled
public class TestMotorEncoder extends LinearOpMode {

    DcMotor motor10 = null;
    DcMotor motor11 = null;


    ElapsedTime runtime1 = new ElapsedTime();
    ElapsedTime runtime2 = new ElapsedTime();

    static double MOTOR_TICK_COUNTS = 1120;
    static double MOTOR_POWER = 1.0;
    static double TARGET_DELTA_COEF = 0.25;
    static DcMotor.ZeroPowerBehavior ZEROPOWERBEHAVIORSETTING = ZeroPowerBehavior.BRAKE;


    int loops0 = 0;
    int loops1 = 0;
    int loops2 = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        runtime1.reset();
        runtime2.reset();

        TelemetryWrapper.init(telemetry,11);

        motor10 = hardwareMap.get(DcMotor.class,"Motor10");
        motor11 = hardwareMap.get(DcMotor.class,"Motor11");
//        motor10.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//        motor11.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        motor10.setZeroPowerBehavior(ZEROPOWERBEHAVIORSETTING);
        motor11.setZeroPowerBehavior(ZEROPOWERBEHAVIORSETTING);
        motor10.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor11.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor10.setPower(0);
        motor11.setPower(0);

        int newTarget1,newTarget2;
        int gp1_a_times = 0;
        int gp1_y_times = 0;

        waitForStart();

        while (opModeIsActive()) {

            loops0++;
            TelemetryWrapper.setLine(0,"Loops in opModeIsActive: " + loops0);

//            TelemetryWrapper.setLine(3,"setPower(): "+MOTOR_POWER);
//            TelemetryWrapper.setLine(4,"runMode():  " + motor11.getMode().toString());
//            TelemetryWrapper.setLine(5,"isPIDMode():" +
//                    (motor11.getMode().isPIDMode()?"PID Mode;":"Not a PID Mode;")+
//                    " cPos:" + (motor11.getCurrentPosition()) +
//                    " tPos:" + (motor11.getTargetPosition()));
//            TelemetryWrapper.setLine(6,"Motors' zeroPowerBehavior: " + motor11.getZeroPowerBehavior().toString());
//            TelemetryWrapper.setLine(7,"MOTOR_POWER: " + MOTOR_POWER);
//            TelemetryWrapper.setLine(8,"TARGET_DELTA_COEF: " + TARGET_DELTA_COEF);
//            TelemetryWrapper.setLine(9,"ZEROPOWERBEHAVIOR: " + ZEROPOWERBEHAVIORSETTING.toString());
//            TelemetryWrapper.setLine(10,"MOTOR_TICK_COUNTS: " + MOTOR_TICK_COUNTS);

            if (gamepad1.y) {

                TelemetryWrapper.clear();
                TelemetryWrapper.render();

                TelemetryWrapper.setLine(0,"Loops in opModeIsActive: " + loops0);

                TelemetryWrapper.setLine(3,"setPower(): "+MOTOR_POWER);
                TelemetryWrapper.setLine(4,"runMode():  " + motor11.getMode().toString());
                TelemetryWrapper.setLine(5,"isPIDMode():" +
                        (motor11.getMode().isPIDMode()?"PID Mode;":"Not a PID Mode;")+
                        " cPos:" + (motor11.getCurrentPosition()) +
                        " tPos:" + (motor11.getTargetPosition()));
                TelemetryWrapper.setLine(6,"Motors' zeroPowerBehavior: " + motor11.getZeroPowerBehavior().toString());
                TelemetryWrapper.setLine(7,"MOTOR_POWER: " + MOTOR_POWER);
                TelemetryWrapper.setLine(8,"TARGET_DELTA_COEF: " + TARGET_DELTA_COEF);
                TelemetryWrapper.setLine(9,"ZEROPOWERBEHAVIOR: " + ZEROPOWERBEHAVIORSETTING.toString());
                TelemetryWrapper.setLine(10,"MOTOR_TICK_COUNTS: " + MOTOR_TICK_COUNTS);

                runtime2.reset();
                gp1_y_times++;
                TelemetryWrapper.setLine(2,"gampad1.y is pressed: " + gp1_y_times);

                motor11.setZeroPowerBehavior(ZEROPOWERBEHAVIORSETTING);
                TelemetryWrapper.setLine(6,"Motors' zeroPowerBehavior: " + motor11.getZeroPowerBehavior().toString());

                motor11.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                TelemetryWrapper.setLine(4,"runMode():  " + motor11.getMode().toString());

                newTarget2 = motor11.getCurrentPosition() + (int) (MOTOR_TICK_COUNTS * TARGET_DELTA_COEF);
                motor11.setTargetPosition(newTarget2);
                TelemetryWrapper.setLine(5,"isPIDMode():" +
                        (motor11.getMode().isPIDMode()?"PID Mode;":"Not a PID Mode;")+
                        " cPos:" + (motor11.getCurrentPosition()) +
                        " tPos:" + (motor11.getTargetPosition()));

                motor11.setPower(MOTOR_POWER);
                TelemetryWrapper.setLine(3,"setPower(): "+MOTOR_POWER);

                motor11.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                TelemetryWrapper.setLine(4,"runMode():  " + motor11.getMode().toString());
                loops2=0;
                while(motor11.isBusy()){
                    loops2++;
                    TelemetryWrapper.setLine(1,"Loops when busy: " + loops2 + "; Runing time: " + runtime2.toString() );
                    TelemetryWrapper.setLine(5,"isPIDMode():" +
                            (motor11.getMode().isPIDMode()?"PID Mode;":"Not a PID Mode;")+
                            " cPos:" + (motor11.getCurrentPosition()) +
                            " tPos:" + (motor11.getTargetPosition()));
                    if (runtime2.seconds() >= 20) {
                        break;
                    }
                }
                motor11.setPower(0);
                TelemetryWrapper.setLine(3,"setPower() was set to: " + 0);
                motor11.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                TelemetryWrapper.setLine(4,"runMode():  " + motor11.getMode().toString());
            }

            if (gamepad1.a) {

                TelemetryWrapper.clear();
                TelemetryWrapper.render();

                TelemetryWrapper.setLine(0,"Loops in opModeIsActive: " + loops0);

                TelemetryWrapper.setLine(3,"setPower(): "+MOTOR_POWER);
                TelemetryWrapper.setLine(4,"runMode():  " + motor10.getMode().toString());
                TelemetryWrapper.setLine(5,"isPIDMode():" +
                        (motor10.getMode().isPIDMode()?"PID Mode;":"Not a PID Mode;")+
                        " cPos:" + (motor10.getCurrentPosition()) +
                        " tPos:" + (motor10.getTargetPosition()));
                TelemetryWrapper.setLine(6,"Motors' zeroPowerBehavior: " + motor10.getZeroPowerBehavior().toString());
                TelemetryWrapper.setLine(7,"MOTOR_POWER: " + MOTOR_POWER);
                TelemetryWrapper.setLine(8,"TARGET_DELTA_COEF: " + TARGET_DELTA_COEF);
                TelemetryWrapper.setLine(9,"ZEROPOWERBEHAVIOR: " + ZEROPOWERBEHAVIORSETTING.toString());
                TelemetryWrapper.setLine(10,"MOTOR_TICK_COUNTS: " + MOTOR_TICK_COUNTS);

                runtime1.reset();
                gp1_a_times++;
                TelemetryWrapper.setLine(2,"gampad1.a is pressed: " + gp1_a_times);

                motor10.setZeroPowerBehavior(ZEROPOWERBEHAVIORSETTING);
                TelemetryWrapper.setLine(6,"Motors' zeroPowerBehavior: " + motor10.getZeroPowerBehavior().toString());

                motor10.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                TelemetryWrapper.setLine(4,"runMode():  " + motor10.getMode().toString());

                newTarget1 = motor10.getCurrentPosition() + (int) (MOTOR_TICK_COUNTS * TARGET_DELTA_COEF);
                motor10.setTargetPosition(newTarget1);
                TelemetryWrapper.setLine(5,"isPIDMode():" +
                        (motor10.getMode().isPIDMode()?"PID Mode;":"Not a PID Mode;")+
                        " cPos:" + (motor10.getCurrentPosition()) +
                        " tPos:" + (motor10.getTargetPosition()));

                motor10.setPower(MOTOR_POWER);
                TelemetryWrapper.setLine(3,"setPower(): "+MOTOR_POWER);

                motor10.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                TelemetryWrapper.setLine(4,"runMode():  " + motor10.getMode().toString());
                loops1=0;
                while(motor10.isBusy()){
                    loops1++;
                    TelemetryWrapper.setLine(1,"Loops when busy: " + loops1 + "; Runing time: " + runtime1.toString() );
                    TelemetryWrapper.setLine(5,"isPIDMode():" +
                            (motor10.getMode().isPIDMode()?"PID Mode;":"Not a PID Mode;")+
                            " cPos:" + (motor10.getCurrentPosition()) +
                            " tPos:" + (motor10.getTargetPosition()));
                    if (runtime1.seconds() >= 20) {
                        break;
                    }
                }
                motor10.setPower(0);
                TelemetryWrapper.setLine(3,"setPower() was set to: " + 0);
                motor10.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                TelemetryWrapper.setLine(4,"runMode():  " + motor10.getMode().toString());
            }


            /** Press gamepad1.left_bumper to start configuring the parameter MOTOR_POWER:
             *
             *      dpad_up to increase Motor Power
             *      dpad_down to decrease Motor Power
             *      right_bumper to exit setting process
             *
             */
            if (gamepad1.left_bumper) {
                TelemetryWrapper.clear();
                TelemetryWrapper.render();
                TelemetryWrapper.setLine(7,"MOTOR_POWER: " + MOTOR_POWER);
                TelemetryWrapper.setLine(8,"TARGET_DELTA_COEF: " + TARGET_DELTA_COEF);
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
                    /** Set motor power */
                    if (gamepad1.dpad_down) {
                        MOTOR_POWER -= 0.01;
                        if (MOTOR_POWER < -1.0) MOTOR_POWER = -1.0;
                        if (MOTOR_POWER < 0.005 && MOTOR_POWER > -0.005) MOTOR_POWER = 0.0;
                        TelemetryWrapper.setLine(7,"MOTOR_POWER: " + MOTOR_POWER);
                        sleep(100);
                    }

                    /** Set Target Delta Counts -- gamepad1.back */
                    if (gamepad1.back) {
                        TARGET_DELTA_COEF += 0.125;
                        if (TARGET_DELTA_COEF > 1.0) {
                            TARGET_DELTA_COEF = -1.0;
                        }
                        TelemetryWrapper.setLine(8,"TARGET_DELTA_COEF: " + TARGET_DELTA_COEF);
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
                            MOTOR_TICK_COUNTS = 1120;
                        }
                        TelemetryWrapper.setLine(10,"MOTOR_TICK_COUNTS: " + MOTOR_TICK_COUNTS);
                        sleep(200);
                    }
                }
            }
        }
//        TelemetryWrapper.setLine(4,"runMode():  " + motor11.getMode().toString());
    }
}
