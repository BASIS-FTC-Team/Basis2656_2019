//
// This program is the version of the TeleOp for Basis 2019 that try to be recovered to the status
// that might be the most close to the runnable version before Apr.20, 2019.
//
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import static org.firstinspires.ftc.teamcode.util.ButtonHelper.a;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.b;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_down;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_left;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_right;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_up;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.left_bumper;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.right_bumper;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.x;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.y;

@TeleOp(name = "TeleOp_Manual_Control",group = "Basis2656_2019")
//@Disabled
public class TeleOp_ManualCtrl extends LinearOpMode {

    private ButtonHelper        bH1;
    private ButtonHelper        bH2;

    private Config              config = new Config(Config.configFile);
    private ElapsedTime         runtime = new ElapsedTime();

    private DriveTrain          driveTrain = new DriveTrain();
    private LiftArm             liftArm = new LiftArm();
    private ForeArm             foreArm = new ForeArm();
    private MineralCollector    mineralCollector = new MineralCollector();

    //VuforiaLocalizer            vuforia;

    @Override
    public void runOpMode() {

        TelemetryWrapper.init(telemetry,11);

        bH1 = new ButtonHelper(gamepad1);
        bH2 = new ButtonHelper(gamepad2);

        Parameters.init(config);
        Hardware2019.init(hardwareMap);

        driveTrain.init();
        liftArm.initEnc();
        foreArm.initEnc();
        mineralCollector.init();

        // Wait for the start button
        waitForStart();

        runtime.reset();
        while(opModeIsActive()){

            bH1.update();
            bH2.update();

            //////////////// Robot Driving /////////////////////////////////////////////////////////
            /** Reduction ratios are applied to the stick readings
             * to reduce the too strong power applied to the motors
             **/
            double drive_x =  gamepad1.left_stick_x ;
            double drive_y = -gamepad1.left_stick_y ;
            double turn =     gamepad1.right_stick_x ;
            if (Math.abs(drive_y) > 0.01 || Math.abs(drive_x) > 0.01 || Math.abs(turn) > 0.01) {
                // Allow minor stick_y value when intending move left/right only
                if ((Math.abs(drive_y) < Math.abs(drive_x)) && (Math.abs(drive_y) < 0.05)) {
                    drive_y = 0;
                }
                drive_x *=  0.8;
                drive_y *=  0.6;
                turn    *=  0.5;
                driveTrain.moveFree(drive_x, drive_y, turn);
            } else {
                drive_x =   gamepad2.left_stick_x * 0.5;
                drive_y = - gamepad2.left_stick_y * 0.5;
                turn =      gamepad2.right_stick_x * 0.5;
                drive_x *=  0.8;
                drive_y *=  0.6;
                turn    *=  0.5;
                driveTrain.moveFree(drive_x, drive_y, turn);
            }

            // Show the elapsed game time and wheel power.
            //TelemetryWrapper.setLine(4,"Motors in drivex: " + drive_x +"drivey: " + drive_y+" turn: "+turn);


            /////////  Controlling foreArm Up/Down  ////////////////////////////////////////////////////
            /** For Forearm to move UP or DOWN (using encoder)
             * b - foreAre up
             * a - foreArm down
             * x - backward
             * y - forward
             *
             * NOTICE: Actually, in the coding, button a and b are corresponding to
             * the physical buttons x and y of the gamepads, and x and y in coding to
             * a and b in physical gamepads when Logitech F310 is connected to
             * Xiaomi smartphones
             *
             * */
            if (bH1.held(b)) {
                foreArm.keepUpingEnc();
                //TelemetryWrapper.setLine(6, "Button B hold.");
            }
            if (bH1.releasing(b)) {
                foreArm.stopUpDownEnc();
                //TelemetryWrapper.setLine(6, "Button B released.");
            }

            if (bH1.held(a)) {
                foreArm.keepDowningEnc();
                //TelemetryWrapper.setLine(6, "Button A hold.");
            }
            if (bH1.releasing(a)) {
                foreArm.stopUpDownEnc();
                //TelemetryWrapper.setLine(6, "Button A released.");
            }

            ///////////////////////////////  Gamepad2  ///////////////////////////
            if (bH2.held(b)) {
                foreArm.keepUpingEnc2();
                //TelemetryWrapper.setLine(6, "Button B2 hold.");
            }
            if (bH2.releasing(b)) {
                foreArm.stopUpDownEnc2();
                //TelemetryWrapper.setLine(6, "Button B2 released.");
            }

            if (bH2.held(a)) {
                foreArm.keepDowningEnc2();
                //TelemetryWrapper.setLine(6, "Button A2 hold.");
            }
            if (bH2.releasing(a)) {
                foreArm.stopUpDownEnc2();
                //TelemetryWrapper.setLine(6, "Button A2 released.");
            }

//            // 一键上收测试
//            if (bH1.pressed(dpad_up)) {
//                foreArm.moveUpDownAngleEnc2(ANGLE_FOR_ONE_STEP);
//                TelemetryWrapper.setLine(6,"Button BACK pressed.");
//            }
//            // 一键下放测试
//            if (bH1.pressed(dpad_down)) {
//                foreArm.moveUpDownAngleEnc2(-ANGLE_FOR_ONE_STEP);
//                TelemetryWrapper.setLine(6,"Button START pressed.");
//            }

            /////////  Controlling foreArm Forth/Back  ///////////////////////////////////////////////
            /** For Forearm to move FORWARD or BACKWARD */
            if (bH1.held(y)) {
                foreArm.keepForwardingEnc();
                //TelemetryWrapper.setLine(6, "Button Y hold.");
            }
            if (bH1.releasing(y)) {
                foreArm.stopForthBackEnc();
                //TelemetryWrapper.setLine(6, "Button Y released.");
            }

            if (bH1.held(x)) {
                foreArm.keepBackwardingEnc();
                //TelemetryWrapper.setLine(6, "Button X hold.");
            }
            if (bH1.releasing(x)) {
                foreArm.stopForthBackEnc();
                //TelemetryWrapper.setLine(6, "Button X released.");
            }

            ////////  Controlling the mineral collector  ////////////////////////////////////////////
            /** For collecting the minerals */
            if (bH1.pressing(left_bumper)) {

                if (mineralCollector.isWipingIn()) {
                    mineralCollector.wipeStop();
                    //TelemetryWrapper.setLine(2, "Wipe IN stoped");
                } else {
                    mineralCollector.wipeIn();
                    //TelemetryWrapper.setLine(2, "Wiping IN");
                }
            }
            if (bH1.pressing(right_bumper)) {

                if (mineralCollector.isWipingOut()) {
                    mineralCollector.wipeStop();
                    //TelemetryWrapper.setLine(2, "Wipe OUT stoped");
                } else {
                    mineralCollector.wipeOut();
                    //TelemetryWrapper.setLine(2, "Wiping OUT");
                }
            }

            /** For opening or closing the mineral collector holder */
            if (bH1.pressing(dpad_right)) {
                mineralCollector.openHolder();
                //TelemetryWrapper.setLine(2,"Open the Holder");
            } else if (bH1.pressing(dpad_left)) {
                mineralCollector.closeHolder();
                //TelemetryWrapper.setLine(2,"Close the Holder");
            }

            /////////// Latching and landing by Lift ///////////////////
            /** Latching up: dpad_up
             *  landing off: dpad_down
             **/
            if(bH1.held(dpad_up) || bH2.held(dpad_up)) {
                    //TelemetryWrapper.setLine(0, "Button DPAD_UP pressed, going UP");
                    liftArm.keepUpingEnc();
                    //TelemetryWrapper.setLine(1,"Lift Move Up at speed: "+ liftArm.getRunningPower());
                    //TelemetryWrapper.setLine(8,"Lift current postion: "+ liftArm.getLiftPosition());
            }
            if(bH1.releasing(dpad_up) || bH2.releasing(dpad_up)) {
                    //TelemetryWrapper.setLine(0, "Releasing DPAD_UP to STOP going UP");
                    liftArm.stopEnc();
                    //TelemetryWrapper.setLine(1, "Lift Move Up at speed: " + liftArm.getRunningPower());
                    //TelemetryWrapper.setLine(8, "Lift current postion: " + liftArm.getLiftPosition());
            }
            if(bH1.held(dpad_down) || bH2.held(dpad_down)) {
                if(!liftArm.isTouched()) {
                    //TelemetryWrapper.setLine(0, "Sensor Not Touched when going DOWN");
                    liftArm.keepDowningEnc();
                    //TelemetryWrapper.setLine(1, "Lift Move Down at speed: " + liftArm.getRunningPower());
                    //TelemetryWrapper.setLine(8, "Lift current postion: " + liftArm.getLiftPosition());
                } else {
                    liftArm.stopAtOnceEnc();
                    //TelemetryWrapper.setLine(1, "Lift Move Down at speed: " + liftArm.getRunningPower());
                    //TelemetryWrapper.setLine(8, "Lift current postion: " + liftArm.getLiftPosition());
                    //TelemetryWrapper.setLine(0, "Sensor Touched when going DOWN, STOP at once!");
                }
            }

            if(bH1.releasing(dpad_down) || bH2.releasing(dpad_down)) {
                if (!liftArm.isTouched()) {
                    //TelemetryWrapper.setLine(0, "Releasing dpad_down to STOP going DOWN");
                    liftArm.stopEnc();
                    //TelemetryWrapper.setLine(1, "Lift Move Up at speed: " + liftArm.getRunningPower());
                    //TelemetryWrapper.setLine(8, "Lift current postion: " + liftArm.getLiftPosition());
                } else {
                    liftArm.stopAtOnceEnc();
                    //TelemetryWrapper.setLine(1, "Lift Move Down at speed: " + liftArm.getRunningPower());
                    //TelemetryWrapper.setLine(8, "Lift current postion: " + liftArm.getLiftPosition());
                    //TelemetryWrapper.setLine(0, "Sensor Touched when going DOWN, STOP at once!");
                }
            }

        }
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

