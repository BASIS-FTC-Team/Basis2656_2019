package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import static org.firstinspires.ftc.teamcode.util.ButtonHelper.a;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.b;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.back;
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
                // Allow minor stick_y value when intending to move left/right only
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
            if (bH1.held(b) || bH2.held(b) ) {
                foreArm.keepUpingEnc();
            }
            if (bH1.releasing(b) || bH2.releasing(b)) {
                foreArm.stopUpDownEnc();
            }

            if (bH1.held(a) || bH2.held(a) ) {
                foreArm.keepDowningEnc();
            }
            if (bH1.releasing(a) || bH2.releasing(a)) {
                foreArm.stopUpDownEnc();
            }

            /////////  Controlling foreArm Forth/Back  ///////////////////////////////////////////////
            /** For Forearm to move FORWARD or BACKWARD */
            if (bH1.held(y) || bH2.held(y)) {
                foreArm.keepForwardingEnc();
            }
            if (bH1.releasing(y) || bH2.releasing(y)) {
                foreArm.stopForthBackEnc();
            }

            if (bH1.held(x) || bH2.held(x)) {
                foreArm.keepBackwardingEnc();
            }
            if (bH1.releasing(x) || bH2.releasing(x)) {
                foreArm.stopForthBackEnc();
            }

            ////////  Controlling the mineral collector  ////////////////////////////////////////////
            /** For collecting the minerals */
            if (bH1.pressing(left_bumper) || bH2.pressing(left_bumper)) {

                if (mineralCollector.isWipingIn()) {
                    mineralCollector.wipeStop();
                } else {
                    mineralCollector.wipeIn();
                }
            }
            if (bH1.pressing(right_bumper) || bH2.pressing(right_bumper)) {

                if (mineralCollector.isWipingOut()) {
                    mineralCollector.wipeStop();
                } else {
                    mineralCollector.wipeOut();
                }
            }

            /** For opening or closing the mineral collector holder */
            if (bH1.pressing(dpad_right) || bH2.pressing(dpad_right)) {
                mineralCollector.openHolder();
            } else if (bH1.pressing(dpad_left) || bH2.pressing(dpad_left)) {
                mineralCollector.closeHolder();
            }
            if (mineralCollector.holderIsClosed()) {
                mineralCollector.closeHolder();
            } else {
                mineralCollector.openHolder();
            }

            /////////// Latching and landing by Lift ///////////////////
            /** Latching up: dpad_up
             *  landing off: dpad_down
             **/
            if(bH1.held(dpad_up) || bH2.held(dpad_up)) {
                    liftArm.keepUpingEnc();
            }
            if(bH1.releasing(dpad_up) || bH2.releasing(dpad_up)) {
                    liftArm.stopEnc();
            }
            if(bH1.held(dpad_down) || bH2.held(dpad_down)) {
                if(!liftArm.isTouched()) {
                    liftArm.keepDowningEnc();
                } else {
                    liftArm.stopAtOnceEnc();
                }
            }

            if(bH1.releasing(dpad_down) || bH2.releasing(dpad_down)) {
                if (!liftArm.isTouched()) {
                    liftArm.stopEnc();
                } else {
                    liftArm.stopAtOnceEnc();
                }
            }

            if((bH1.pressing(back)) || (bH2.pressing(back))) {
                liftArm.autoGoingUp(600);
                //liftArm.stopAtOnceEnc();
            }

        }
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

