//
// This program is the version of the TeleOp for Basis 2019 that try to be recovered to the status
// that might be the most close to the runnable version before Apr.20, 2019.
//
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.*;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

@TeleOp(name = "TeleOp_v00",group = "Basis2656_2019")
//@Disabled
public class TeleOp_v00 extends LinearOpMode {


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

    private ButtonHelper gamepadButtons1;
    private ButtonHelper gamepadButtons2;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        gamepadButtons1 = new ButtonHelper(gamepad1);
        gamepadButtons2 = new ButtonHelper(gamepad2);

        pidForForearmUpDown = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);
        pidForForearmForthBack = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);
        pidForLiftUpDown = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);

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

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            gamepadButtons1.update();
            gamepadButtons2.update();

            /** move the robot */

//            /** Origin version */
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
            double drivex = -gamepad1.left_stick_x * 0.5;
            double drivey =  gamepad1.left_stick_y * 0.5;
            double turn = -gamepad1.right_stick_x * 0.5;
            driveTrain.moveFree(drivex,drivey,turn);

            // Show the elapsed game time and wheel power.
            TelemetryWrapper.setLine(4,"Motors in drivex: " + drivex +"drivey: " + drivey+" turn: "+turn);
            TelemetryWrapper.setLine(5, "Press Stop to end test." );


            /** For Forearm to move UP or DOWN (using encoder)
             * b - foreAre up
             * a - foreArm down
             * x - backward
             * y - forward
             * */

            if (gamepad1.b) {
                    foreArm.moveUpEnc(1000);
                    TelemetryWrapper.setLine(3, "b pressed. Fore Arm move UP");
            } else if (gamepad1.a) {
                    foreArm.moveDownEnc(1000);
                    TelemetryWrapper.setLine(3, "x pressed. Fore Arm move DOWN");
            }
//            else {
//                foreArm.stopUpDownEnc();
//                TelemetryWrapper.setLine(3,"b or x not pressed. Fore Arm UpDown stopped");
//            }

            /** For Forearm to move FORWARD or BACKWARD */
            if ( gamepad1.y) {
                    foreArm.moveForwardEnc(1000);
                    TelemetryWrapper.setLine(3, "a pressed. Fore Arm move FORWARD");
            } else if (gamepad1.x) {
                    foreArm.moveBackwardEnc(1000);
                    TelemetryWrapper.setLine(3, "y pressed. Fore Arm move BACKWARD");
            }
//            else {
//                foreArm.stopForthBackEnc();
//                TelemetryWrapper.setLine(3,"a or y not pressed. Fore Arm FandB stopped");
//            }

            /** For collecting the minerals */
            if (gamepad1.left_bumper) {
                if (mineralCollectorIsWipingIn) {
                    mineralCollector.wipeStop();
                    mineralCollectorIsWipingIn = false;
                    TelemetryWrapper.setLine(2, "Wipe IN stoped");
                } else {
                    mineralCollector.wipeIn();
                    mineralCollectorIsWipingIn = true;
                    TelemetryWrapper.setLine(2, "Wiping IN");
                }
                sleep(50);
            }
            if (gamepad1.right_bumper) {
                if (mineralCollectorIsWipingOut) {
                    mineralCollector.wipeStop();
                    mineralCollectorIsWipingOut = false;
                    TelemetryWrapper.setLine(2, "Wipe OUT stoped");
                } else {
                    mineralCollector.wipeOut();
                    mineralCollectorIsWipingOut = true;
                    TelemetryWrapper.setLine(2, "Wiping OUT");
                }
                sleep(50);
            }
//            if(gamepad1.start) {
//                mineralCollector.wipePause();
//                TelemetryWrapper.setLine(2,"Pause wiping");
//            }

            /** For opening or closing the mineral collector holder */
            if (gamepad1.dpad_left) {
                mineralCollector.openHolder();
                TelemetryWrapper.setLine(2,"Open the Holder");
            } else if (gamepad1.dpad_right) {
                mineralCollector.closeHolder();
                TelemetryWrapper.setLine(2,"Close the Holder");
            }

            /** Latching and landing */
//            if(gamepad1.dpad_up) {
//                if (liftIsUping) {
//                    liftArm.stop();
//                    liftIsUping = false;
//                    TelemetryWrapper.setLine(2, "Lift stopped moving up");
//                } else {
//                    liftArm.moveUp();
//                    liftIsUping = true;
//                    TelemetryWrapper.setLine(2, "Lift moving up...");
//                }
//                sleep(50);
//            }
//            if(gamepad1.dpad_down) {
//                if (liftIsDowning) {
//                    liftArm.stop();
//                    liftIsDowning = false;
//                    TelemetryWrapper.setLine(2, "Lift stopped moving down");
//                } else {
//                    liftArm.moveDown();
//                    liftIsDowning = true;
//                    TelemetryWrapper.setLine(2, "Lift moving down...");
//                }
//                sleep(50);
//            }
            if(gamepadButtons1.pressed(dpad_up)){
                if(!liftArm.isTouched()) {
                    TelemetryWrapper.setLine(0, "Not Touched when going UP");
                    liftArm.moveUpEnc();
                    TelemetryWrapper.setLine(1,"Container Move Up at speed: "+ (-1)*liftArm.LIFT_POWER);
                    TelemetryWrapper.setLine(8,"Container current postion: "+ liftArm.getLiftPosition());
                }
            } else if(gamepadButtons1.pressed(dpad_down)){
                    TelemetryWrapper.setLine(0, "Not Touched when going DOWN");
                    liftArm.moveDownEnc();
                    TelemetryWrapper.setLine(1, "Container Move Down at speed: " + liftArm.LIFT_POWER);
                    TelemetryWrapper.setLine(8, "Container current postion: " + liftArm.getLiftPosition());
            } else {
                liftArm.stopEnc();
                TelemetryWrapper.setLine(1,"dpad up/down not pressed. Container Stopped!");
            }
            if(liftArm.isTouched()){
                TelemetryWrapper.setLine(0,"Touch sensor pressed");
            } else {
                TelemetryWrapper.setLine(0,"Touch sensor NOT pressed");
            }

            /*****************************************************************************
             *  The following part is for automove testing
             */

            if (gamepad2.dpad_down) {
                foreArm.moveUpDownAngleEnc(0.5,-45.0,3000);
            }
            if (gamepad2.dpad_up) {
                foreArm.moveUpDownAngleEnc(0.5,45.0,3000);
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

