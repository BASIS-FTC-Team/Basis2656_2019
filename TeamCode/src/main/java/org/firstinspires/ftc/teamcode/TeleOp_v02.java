//
// This program is the version of the TeleOp for Basis 2019 that try to be recovered to the status
// that might be the most close to the runnable version before Apr.20, 2019.
//
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;

import static org.firstinspires.ftc.teamcode.Parameters.ANGLE_FOR_ONE_STEP;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.*;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

@TeleOp(name = "TeleOp_v02",group = "Basis2656_2019")
//@Disabled
public class TeleOp_v02 extends LinearOpMode {


    LiftArm liftArm = new LiftArm();
    ForeArm foreArm = new ForeArm();
    MineralCollector mineralCollector = new MineralCollector();
    DriveTrain driveTrain = new DriveTrain();
    PID pidForForearmUpDown = null;
    PID pidForForearmForthBack = null;
    PID pidForLiftUpDown = null;


    private Config config = new Config(Config.configFile);

//
//    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
//    static final int    CYCLE_MS    =   50;     // period of each cycle
//


    boolean foreArmIsUping = false;
    boolean foreArmIsDowning = false;
    boolean foreArmIsForwarding = false;
    boolean foreArmIsBacking = false;
    boolean mineralCollectorIsWipingOut = false;
    boolean mineralCollectorIsWipingIn = false;
    boolean liftIsUping = false;
    boolean liftIsDowning = false;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


//////////////////////////////////////////
    private ButtonHelper bH1 = null;
    private ButtonHelper bH2 = null;

    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;

//////////////////////////////////////////

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        bH1 = new ButtonHelper(gamepad1);
        bH2 = new ButtonHelper(gamepad2);

        pidForForearmUpDown = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);
        pidForForearmForthBack = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);
        pidForLiftUpDown = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);

        Parameters.init(config);
        //In TeleOp, encoders are not used for controlling the motors for robot wheels
        driveTrain.init(hardwareMap,config);
        // Encoder will be used for controlling the liftArm
        liftArm.initEnc(hardwareMap,config,pidForLiftUpDown);
        // Encoder will be used for controlling the foreArm
        foreArm.initEnc(hardwareMap,config,pidForForearmUpDown,pidForForearmForthBack);
        mineralCollector.init(hardwareMap,config);
        TelemetryWrapper.init(telemetry,11);

        // Wait for the start button
        waitForStart();

        runtime.reset();
        while(opModeIsActive()){
            bH1.update();
            bH2.update();

            /** move the robot */

            /** Origin version */
//            double drivey =  -gamepad1.left_stick_y;
//            double drivex =  -gamepad1.left_stick_x;
//            double turn  =  gamepad1.right_stick_x;
//            if (Math.abs(drivey) > 0.01 || Math.abs(drivex) > 0.01 || Math.abs(turn) > 0.01) {
//                driveTrain.move(drivex, drivey, turn);
//            }
//            else {
//                drivey = -gamepad2.left_stick_y * 0.25;
//                drivex = -gamepad2.left_stick_x * 0.25;
//                turn = gamepad2.right_stick_x * 0.25;
//                driveTrain.move(drivex, drivey, turn);
//            }
            /** New version */
            // 0.5 is applied to the stick readings to reduce the too strong power applied to the motors

            double drive_x = gamepad1.left_stick_x ;
            double drive_y =  -gamepad1.left_stick_y ;
            double turn =    gamepad1.right_stick_x ;
            // Allow minor stick_y value when intending move left/right only
            if ((Math.abs(drive_y) < Math.abs(drive_x)) && (Math.abs(drive_y) < 0.05)) {
                drive_y = 0;
            }
            drive_x *=  0.8;
            drive_y *=  0.6;
            turn    *=  0.5;

            driveTrain.moveFree(drive_x,drive_y,turn);

            // Show the elapsed game time and wheel power.
            TelemetryWrapper.setLine(4,"Motors in drivex: " + drive_x +"drivey: " + drive_y+" turn: "+turn);
            TelemetryWrapper.setLine(5, "Press Stop to end test." );


            /** For Forearm to move UP or DOWN (using encoder)
             * b - foreAre up
             * a - foreArm down
             * x - backward
             * y - forward
             * */

///////////////////////////////////////////////////
            if (bH1.held(b)) {
                foreArm.keepUpingEnc();
                TelemetryWrapper.setLine(6, "Button B hold.");
            }
            if (bH1.releasing(b)) {
                foreArm.stopUpDownEnc();
                TelemetryWrapper.setLine(6, "Button B released.");
            }

            if (bH1.held(a)) {
                foreArm.keepDowningEnc();
                TelemetryWrapper.setLine(6, "Button A hold.");
            }
            if (bH1.releasing(a)) {
                foreArm.stopUpDownEnc();
                TelemetryWrapper.setLine(6, "Button A released.");
            }

            // 一键上收测试
            if (bH1.pressed(back)) {
                foreArm.moveUpDownAngleEnc(ANGLE_FOR_ONE_STEP,8000);
                TelemetryWrapper.setLine(6,"Button BACK pressed.");
            }

            // 一键下放测试
            if (bH1.pressed(start)) {
                foreArm.moveUpDownAngleEnc(-ANGLE_FOR_ONE_STEP,8000);
                TelemetryWrapper.setLine(6,"Button START pressed.");
            }

//////////////////////////////////////////////////


            /** For Forearm to move FORWARD or BACKWARD */

///////////////////////////////////////////////////
            if (bH1.held(y)) {
                foreArm.keepForwardingEnc();
                TelemetryWrapper.setLine(6, "Button Y hold.");
            }
            if (bH1.releasing(y)) {
                foreArm.stopForthBackEnc();
                TelemetryWrapper.setLine(6, "Button Y released.");
            }

            if (bH1.held(x)) {
                foreArm.keepBackwardingEnc();
                TelemetryWrapper.setLine(6, "Button X hold.");
            }
            if (bH1.releasing(x)) {
                foreArm.stopForthBackEnc();
                TelemetryWrapper.setLine(6, "Button X released.");
            }
///////////////////////////////////////////////////


            /** For collecting the minerals */
            if (bH1.pressing(left_bumper)) {
                if (mineralCollectorIsWipingIn) {
                    mineralCollector.wipeStop();
                    mineralCollectorIsWipingIn = false;
                    TelemetryWrapper.setLine(2, "Wipe IN stoped");
                } else {
                    mineralCollector.wipeIn();
                    mineralCollectorIsWipingIn = true;
                    TelemetryWrapper.setLine(2, "Wiping IN");
                }
                //sleep(200);
            }
            if (bH1.pressing(right_bumper)) {
                if (mineralCollectorIsWipingOut) {
                    mineralCollector.wipeStop();
                    mineralCollectorIsWipingOut = false;
                    TelemetryWrapper.setLine(2, "Wipe OUT stoped");
                } else {
                    mineralCollector.wipeOut();
                    mineralCollectorIsWipingOut = true;
                    TelemetryWrapper.setLine(2, "Wiping OUT");
                }
                //sleep(200);
            }
//            if(gamepad1.start) {
//                mineralCollector.wipePause();
//                TelemetryWrapper.setLine(2,"Pause wiping");
//            }

            /** For opening or closing the mineral collector holder */
            if (gamepad1.dpad_right) {
                mineralCollector.openHolder();
                TelemetryWrapper.setLine(2,"Open the Holder");
            } else if (gamepad1.dpad_left) {
                mineralCollector.closeHolder();
                TelemetryWrapper.setLine(2,"Close the Holder");
            }

            /** Latching and landing */
            ///////////////// Start of Version 3 ///////////////////
            if(bH1.held(dpad_up)){
                    TelemetryWrapper.setLine(0, "Button DPAD_UP pressed, going UP");
                    liftArm.keepUpingEnc();
                    TelemetryWrapper.setLine(1,"Lift Move Up at speed: "+ liftArm.getRunningPower());
                    TelemetryWrapper.setLine(8,"Lift current postion: "+ liftArm.getLiftPosition());
            }
            if(bH1.releasing(dpad_up)) {
                    TelemetryWrapper.setLine(0, "Releasing DPAD_UP to STOP going UP");
                    liftArm.stopEnc();
                    TelemetryWrapper.setLine(1, "Lift Move Up at speed: " + liftArm.getRunningPower());
                    TelemetryWrapper.setLine(8, "Lift current postion: " + liftArm.getLiftPosition());
            }
            if(bH1.held(dpad_down)){
                if(!liftArm.isTouched()) {
                    TelemetryWrapper.setLine(0, "Sensor Not Touched when going DOWN");
                    liftArm.keepDowningEnc();
                    TelemetryWrapper.setLine(1, "Lift Move Down at speed: " + liftArm.getRunningPower());
                    TelemetryWrapper.setLine(8, "Lift current postion: " + liftArm.getLiftPosition());
                } else {
                    liftArm.stopAtOnceEnc();
                    TelemetryWrapper.setLine(1, "Lift Move Down at speed: " + liftArm.getRunningPower());
                    TelemetryWrapper.setLine(8, "Lift current postion: " + liftArm.getLiftPosition());
                    TelemetryWrapper.setLine(0, "Sensor Touched when going DOWN, STOP at once!");
                }
            }
            if(bH1.releasing(dpad_down)) {
                if (!liftArm.isTouched()) {
                    TelemetryWrapper.setLine(0, "Releasing dpad_down to STOP going DOWN");
                    liftArm.stopEnc();
                    TelemetryWrapper.setLine(1, "Lift Move Up at speed: " + liftArm.getRunningPower());
                    TelemetryWrapper.setLine(8, "Lift current postion: " + liftArm.getLiftPosition());
                } else {
                    liftArm.stopAtOnceEnc();
                    TelemetryWrapper.setLine(1, "Lift Move Down at speed: " + liftArm.getRunningPower());
                    TelemetryWrapper.setLine(8, "Lift current postion: " + liftArm.getLiftPosition());
                    TelemetryWrapper.setLine(0, "Sensor Touched when going DOWN, STOP at once!");
                }
            }

//            if(bH1.pressed(back)) {
//                TelemetryWrapper.setLine(6,"Button BACK pressed to AUTO land off");
//                TelemetryWrapper.setLine(7,"lift_auto_landing_counts is "+liftArm.LIFT_AUTO_LANDING_COUNTS);
//                liftArm.landOffEnc(30000);
//                TelemetryWrapper.setLine(6,"AUTO land off Finished.");
//            }
//            if(bH1.pressed(start)) {
//                TelemetryWrapper.setLine(6,"Button START pressed to AUTO latch on");
//                TelemetryWrapper.setLine(7,"lift_auto_latching_counts is "+liftArm.LIFT_AUTO_LATCHING_COUNTS);
//                liftArm.latchOnEnc(30000);
//                TelemetryWrapper.setLine(6,"AUTO latch on Finished.");
//            }

            if(liftArm.isTouched()){
                TelemetryWrapper.setLine(0,"Touch sensor pressed");
            } else {
                TelemetryWrapper.setLine(0,"Touch sensor NOT pressed");
            }

            if(bH1.pressed(back))


///////////////// End of Version 3 ///////////////////



            /*****************************************************************************
             *  The following part is for automove testing
             */

            if (gamepad2.dpad_down) {
                foreArm.moveUpDownAngleEnc(ANGLE_FOR_ONE_STEP,3000);
            }
            if (gamepad2.dpad_up) {
                foreArm.moveUpDownAngleEnc(ANGLE_FOR_ONE_STEP,3000);
            }

            if (gamepad2.x) {
                foreArm.moveUpDownAngleEncPID(10.0,45.0,3000);
            }
            if (gamepad2.b) {
                foreArm.moveUpDownAngleEncPID(10.0,-45.0,3000);
            }

            if (gamepad2.y) {
                foreArm.stopUpDownEnc();
            }

            /** End of automove testing
             * **************************************************************************/

            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
   
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}

