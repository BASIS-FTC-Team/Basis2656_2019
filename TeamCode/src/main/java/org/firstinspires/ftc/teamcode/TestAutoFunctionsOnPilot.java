package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Parameters.CAMERA_TO_CENTER;
import static org.firstinspires.ftc.teamcode.Parameters.DEPOT_TO_CRATER;
import static org.firstinspires.ftc.teamcode.Parameters.DISTANCE_TO_WALL;
import static org.firstinspires.ftc.teamcode.Parameters.DIST_BTWN_MINERALS;
import static org.firstinspires.ftc.teamcode.Parameters.FIRST_MOVE_RIGHT;
import static org.firstinspires.ftc.teamcode.Parameters.INITIAL_DIST_TO_WALL;
import static org.firstinspires.ftc.teamcode.Parameters.INITIAL_MOVE_TO_MINERAL;
import static org.firstinspires.ftc.teamcode.Parameters.LIFT_AUTO_LANDING_COUNTS;
import static org.firstinspires.ftc.teamcode.Parameters.MINERAL_DETECT_TIMELIMIT;
import static org.firstinspires.ftc.teamcode.Parameters.PULL_BACK;
import static org.firstinspires.ftc.teamcode.Parameters.PUSH_GOLD;
import static org.firstinspires.ftc.teamcode.Parameters.RIGHT_GOLD_DISTANCE;
import static org.firstinspires.ftc.teamcode.Parameters.WALL_TO_DEPOT;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.back;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_down;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_left;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_right;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_up;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.left_bumper;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.right_bumper;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.start;

@TeleOp(name = "Test Auto Functions on Pilot", group = "Test")
//@Disabled
public class TestAutoFunctionsOnPilot extends LinearOpMode {

    ButtonHelper bH1 = null;
    ButtonHelper bH2 = null;
    ElapsedTime runtime = new ElapsedTime();
    private Config config = new Config(Config.configFile);
    GoldPosition gP = GoldPosition.UNKNOWN;

    DriveTrain          driveTrain = new DriveTrain();
    DriveTrainByEncoder driveTrainEnc = new DriveTrainByEncoder();
//    LiftArm             liftArm = new LiftArm();

    private static final String VUFORIA_KEY = "AaVQPxH/////AAABmWbgMV3r8kMuucDJZwS+C8IqcKbjimK6x7yZkfsYnCLGA1cHVqGOF+tSmO//7vH+NwYrxmEfltB1UGzWki397Ksrl57wPSMPbGU2y9Cg+iSgHMGpJVx4IDeD6ldnTIRetHFeW0r4OzmfsDc5eI0tChOd2FYv2Q8MuHq/QXlsdOHEOyy43xqj5QF4eRSVznttm6fDzN2egZWEIr8Un9B0hCEv6OmQATKUsEPx7BnqCxjBK00252+n2Na17OxE2hYP8WXUerdZOOU1GyWFPOG2DDeYDWiipgYGXgpIC+a846STiSZcFXLP2S3ENu78EoCFKs7Fw7sm5u58dzZ5PyMg8VUormyNmcHm9RU2Fl5364WO";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private MineralRecognizer mR;
    //private RobotLocator robotLoc;

    boolean[] stepEnabled = new boolean[20];

    /**
     * Define your variables here
     */


    /**
     * End of Definition
     **********/

    @Override
    public void runOpMode() {
        bH1 = new ButtonHelper(gamepad1);
        bH2 = new ButtonHelper(gamepad2);
        TelemetryWrapper.init(telemetry, 11);

        for (int i = 0; i < 20; i++) {
            stepEnabled[i] = false;
        }

        /** Initialize the variables here */
        Parameters.init(config);
        Hardware2019.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        initVuforia(cameraMonitorViewId);
        //initVuforia();
        initTfod();
        tfod.activate();

        mR = new MineralRecognizer();
        mR.initialize(tfod);

        TelemetryWrapper.setLine(0,String.format("mR initialized. gP is: %s", mR.getGoldPosition().toString()));
//        robotLoc = new RobotLocator();
//        robotLoc.initialize(this.vuforia);

//        liftArm.initEnc();
        //////// For robot drive:
//        leftFront = hardwareMap.get(DcMotor.class, "fl_drive");
//        rightFront = hardwareMap.get(DcMotor.class, "fr_drive");
//        leftBack = hardwareMap.get(DcMotor.class, "rl_drive");
//        rightBack = hardwareMap.get(DcMotor.class, "rr_drive");

        //////////


//        TelemetryWrapper.setLine(1, "ForeArm Reduction: " + GEAR_REDUCTION_FOR_UPDOWN_MOTOR);
//        TelemetryWrapper.setLine(2, "Max foreArm power:" + MAX_POPER_FOR_FOREARM_UPDOWN);
//        TelemetryWrapper.setLine(3, "Min foreArm Power: " + MIN_POWER_FOR_FOREARM_UPDOWN);
        /////////

        /** End of Initialization *********/

        waitForStart();
        //mR.activate();

//        robotLoc.activate();

        int mainloop = 0;
        while (opModeIsActive()) {

            mainloop ++;
            bH1.update();
            bH2.update();

            // Quit by button 'back'
            if (bH1.pressing(back)) {
                break;
            }

            //////////// Moving robot bu remote mannual control /////////
            //In TeleOp, encoders are not used for controlling the motors for robot wheels
            driveTrain.init();
            double drive_x = gamepad1.left_stick_x;
            double drive_y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            if (Math.abs(drive_y) > 0.01 || Math.abs(drive_x) > 0.01 || Math.abs(turn) > 0.01) {

                // Allow minor stick_y value when intending move left/right only
                if ((Math.abs(drive_y) < Math.abs(drive_x)) && (Math.abs(drive_y) < 0.05)) {
                    drive_y = 0;
                }

                drive_x *= 0.8;
                drive_y *= 0.6;
                turn *= 0.5;
                driveTrain.moveFree(drive_x, drive_y, turn);

            } else {
                drive_x = gamepad2.left_stick_x * 0.5;
                drive_y = -gamepad2.left_stick_y * 0.5;
                turn = gamepad2.right_stick_x * 0.5;
                drive_x *= 0.8;
                drive_y *= 0.6;
                turn *= 0.5;
                driveTrain.moveFree(drive_x, drive_y, turn);
            }

            //////////////////////// TFOD mineral recognizer ////////////////////////////////////////////////
            mR.update();
            gP = mR.getGoldPosition();
            TelemetryWrapper.setLine(0,String.format("mR updated in MAIN LOOP (%d). (numM, Gold?, gP) = (%d, %b, %s)",
                    mainloop, mR.getNumM(),mR.goldIsFound(), gP.toString()));
//            if (mR.getNumM() > 0) {
//                List<Mineral> l = mR.getRecognizedMineralList();
//                TelemetryWrapper.clearLines(0,2);
//                TelemetryWrapper.setLine(0, "Mineral(s) recognized.");
//                TelemetryWrapper.setLine(1, String.format("(numM,numG,numS): (%d,%d,%d)", mR.getNumM(), mR.getNumG(), mR.getNumS()));
//                if (mR.getNumM() >= 2) {
//                    TelemetryWrapper.setLine(2, String.format("Slope is %.2f . goldIsFound: %b", mR.getHAlignSlope(),mR.goldIsFound()));
//                    TelemetryWrapper.setLine(3,String.format("Mineral1[%.2f,%.2f,%b], Mineral2[%.2f,%.2f,%b]",
//                            l.get(0).getCenterX(),l.get(0).getCenterY(),l.get(0).isGold(),
//                            l.get(1).getCenterX(),l.get(1).getCenterY(),l.get(1).isGold()));
//                } else {
//                    TelemetryWrapper.setLine(2,String.format("Less than 2 mineral, slope: %.2f . goldIsFound: %b", mR.getHAlignSlope(),mR.goldIsFound()));
//                }
//                int i = 2;
//                for (Mineral m : l) {
//                    i++;
//                    TelemetryWrapper.setLine(i, String.format("(G?,X,Y,A): (%b, %4.1f, %4.1f, %4.1f)", m.isGold(), m.getCenterX(), m.getCenterY(), m.getAngle()));
//                }
//            }
            //////////////////////// Robot Locator ////////////////////////////////////////////////
//            robotLoc.update();
//            if (robotLoc.targetIsVisible()) {
//                TelemetryWrapper.clearLines(3,5);
//                TelemetryWrapper.setLine(3, "Target is visible.");
//                TelemetryWrapper.setLine(4, String.format("Location (x,y,z): (%.2f, %.2f, %.2f)", robotLoc.getLocX(), robotLoc.getLocY(), robotLoc.getLocZ()));
//                TelemetryWrapper.setLine(5, String.format("Orientation(A1,A2,A3): (%.2f, %.2f, %.2f)", robotLoc.getAngle1(), robotLoc.getAngle2(), robotLoc.getAngle3()));
//            } else {
//                TelemetryWrapper.clearLines(3,5);
//                TelemetryWrapper.setLine(3, "Target is NOT visible.");
//            }


            ///////// Choose steps to test ////////
            if (bH1.pressing(start)) {
                driveTrainEnc.initEnc();
                if (stepEnabled[0]) {  // Landing off
//                    TelemetryWrapper.clearLines(6,10);
//                    TelemetryWrapper.setLine(7,String.format("LIFT_AUTO_LANDING_COUNTS = %d ", LIFT_AUTO_LANDING_COUNTS));
//                    TelemetryWrapper.setLine(8, String.format("%8.1f Step 1: Landing off ...", runtime.milliseconds()));
//                    liftArm.landOffEnc(10000);
                }
                if (stepEnabled[1]) {
                    ElapsedTime runtime = new ElapsedTime();

                    TelemetryWrapper.setLine(7,String.format("FIRST_MOVE_RIGHT = %.1f ", FIRST_MOVE_RIGHT));
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 2: Moving 40mm right...", runtime.milliseconds()));
                    driveTrainEnc.moveLeftRightEnc(FIRST_MOVE_RIGHT, 5000);

//                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));
//                    driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
//                    driveTrainEnc.moveForthBackEnc(-FIRST_MOVE_RIGHT - CAMERA_TO_CENTER,5000);

                    double angelToTurnAtWall = -135.0;
//                    runtime.reset();
//                    mR.update();
//                    while ((mR.getNumM() < 1) && (runtime.milliseconds() < MINERAL_DETECT_TIMELIMIT )) {
//                        mR.update();
//                    }
                    runtime.reset();
                    int quick_times = 0;
                    while ((gP == GoldPosition.UNKNOWN) && (runtime.milliseconds()<1000)) {
                        quick_times ++;
                        TelemetryWrapper.setLine(0,String.format("Quick updating mR in START branch: %d", quick_times));
                        mR.update();
                        gP = mR.getGoldPosition();
                    }
                    if ( gP != GoldPosition.UNKNOWN ) {
                        TelemetryWrapper.setLine(0,String.format("Gold found. gP: %s", gP.toString()));
                        switch (gP) {
                            case MIDDLE:
                                TelemetryWrapper.setLine(3,String.format("Sure Middle : (numM, Gold?, gP) = (%d, %b, %s)",
                                        mR.getNumM(),mR.goldIsFound(),mR.getGoldPosition().toString()));
                                TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));
                                driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                                driveTrainEnc.moveForthBackEnc(-FIRST_MOVE_RIGHT,5000);
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD,5000);
                                driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                                //driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL,8000);
                                angelToTurnAtWall = -135.0;
                            case RIGHT:
                                TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));
                                driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                                driveTrainEnc.moveForthBackEnc(-FIRST_MOVE_RIGHT ,5000);
                                driveTrainEnc.spinEnc(45.0,5000);
                                TelemetryWrapper.setLine(3,String.format("Sure Right : (numM, Gold?, gP) = (%d, %b, %s)",
                                        mR.getNumM(),mR.goldIsFound(),mR.getGoldPosition().toString()));
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD*1.4,5000);
                                driveTrainEnc.moveForthBackEnc(-PULL_BACK*1.4, 5000);
                                driveTrainEnc.spinEnc(-45.0,5000);
                                //driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL,8000);
                                angelToTurnAtWall = -135.0;
                            case LEFT:
                                TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));
                                driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                                driveTrainEnc.moveForthBackEnc(-FIRST_MOVE_RIGHT ,5000);
                                TelemetryWrapper.setLine(3,String.format("Sure Left : (numM, Gold?,gP) = (%d, %b, %s)",
                                        mR.getNumM(),mR.goldIsFound(),mR.getGoldPosition().toString()));
                                driveTrainEnc.spinEnc(-45.0,5000);
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD*1.4,5000);
                                driveTrainEnc.moveLeftRightEnc(-PUSH_GOLD,5000);
                                //driveTrainEnc.moveForthBackEnc(DISTANCE_TO_WALL - PUSH_GOLD,8000);
                                angelToTurnAtWall = -45.0;
                        }
                    } else {
                        TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));
                        driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                        driveTrainEnc.moveForthBackEnc(-FIRST_MOVE_RIGHT - CAMERA_TO_CENTER,5000);
                        runtime.reset();
                        mR.update();
                        while ((mR.getNumM() < 1) && (runtime.milliseconds() < MINERAL_DETECT_TIMELIMIT )) {
                            mR.update();
                        }
                        if (mR.goldIsFound()) { //gold is at the CENTER
                            TelemetryWrapper.setLine(3,String.format("Single Center : (numM, Gold?) = (%d, %b)",mR.getNumM(),mR.goldIsFound()));
                            driveTrainEnc.moveForthBackEnc(PUSH_GOLD,5000);
                            driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                            //driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL,8000);
                            angelToTurnAtWall = -135.0;

                        } else {
                            driveTrainEnc.spinEnc(45.0,5000);
                            runtime.reset();
                            mR.update();
                            while ((mR.getNumM() < 1) && (runtime.milliseconds() < MINERAL_DETECT_TIMELIMIT )) {
                                mR.update();
                            }
                            if (mR.goldIsFound()) { //gold is on the RIGHT
                                TelemetryWrapper.setLine(3,String.format("Single Right : (numM, Gold?) = (%d, %b)",mR.getNumM(),mR.goldIsFound()));
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD*1.4,5000);
                                driveTrainEnc.moveForthBackEnc(-PULL_BACK*1.4, 5000);
                                driveTrainEnc.spinEnc(-45.0,5000);
                                //driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL,8000);
                                angelToTurnAtWall = -135.0;
                            } else {
                                TelemetryWrapper.setLine(3,String.format("Supposed Right : (numM, Gold?) = (%d, %b)",mR.getNumM(),mR.goldIsFound()));
                                driveTrainEnc.spinEnc(-90.0,5000);
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD*1.4,5000);
                                driveTrainEnc.moveLeftRightEnc(-PUSH_GOLD,5000);
                                driveTrainEnc.spinEnc(-45.0,5000);
                                //driveTrainEnc.moveForthBackEnc(DISTANCE_TO_WALL-PUSH_GOLD,8000);
                                angelToTurnAtWall = -45.0;
                            }
                        }
                    }
//                    driveTrainEnc.spinEnc(angelToTurnAtWall,5000);
//                    driveTrainEnc.moveForthBackEnc(WALL_TO_DEPOT,5000);
                }
                if (stepEnabled[2]) {
//                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));
//                    driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                }
                if (stepEnabled[3]) {
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 4: Moving left/right to search gold...", runtime.milliseconds()));
                    driveTrainEnc.moveLeftRightEnc(RIGHT_GOLD_DISTANCE, 5000);
                    DISTANCE_TO_WALL = INITIAL_DIST_TO_WALL + RIGHT_GOLD_DISTANCE;
                }
                if (stepEnabled[4]) {
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 5: Moving forwards to push gold...", runtime.milliseconds()));
                    driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);
                }
                if (stepEnabled[5]) {
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 6: Moving back after pushing gold...", runtime.milliseconds()));
                    driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                }
                if (stepEnabled[6]) {
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 7: Moving left towards the wall...", runtime.milliseconds()));
                    driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL, 10000);
                }
                if (stepEnabled[7]) {
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 8: Turning - 135 degrees for depot...", runtime.milliseconds()));
                    driveTrainEnc.spinEnc(-135, 5000);
                }
                if (stepEnabled[8]) {
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 9: Moving towards depot...", runtime.milliseconds()));
                    driveTrainEnc.moveForthBackEnc(WALL_TO_DEPOT, 10000);
                }
                if (stepEnabled[9]) {
//                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 10: Putting off the Team Marker...", runtime.milliseconds()));
//                    foreArm.moveUpDownAngleEnc(-30, 5000);
//                    mineralCollector.wipeOut();
//                    foreArm.moveUpDownAngleEnc(+30, 5000);
                }
                if (stepEnabled[10]) {
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 11: Turning - 180 degrees for crater...", runtime.milliseconds()));
                    driveTrainEnc.spinEnc(-180, 10000);
                }
                if (stepEnabled[11]) {
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 12: Moving towards depot...", runtime.milliseconds()));
                    driveTrainEnc.moveForthBackEnc(DEPOT_TO_CRATER, 10000);
                }
                if (stepEnabled[12]) {
//                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 13: Putting off the foreArm to touch the crater...", runtime.milliseconds()));
//                    foreArm.moveUpDownAngleEnc(-30, 5000);
                }
                if (stepEnabled[13]) {

                }
                if (stepEnabled[14]) { // To Right
//                    TelemetryWrapper.setLine(7,String.format("FIRST_MOVE_RIGHT = %.1f ", 600.));
                    TelemetryWrapper.setLine(10, String.format("FIRST_MOVE_RIGHT = %.1f ; %8.1f Step 1: Moving 600mm right...", 600.,runtime.milliseconds()));
                    driveTrainEnc.moveLeftRightEnc(600., 5000);
                }
                if (stepEnabled[15]) { // To Left
                    TelemetryWrapper.setLine(7,String.format("MOVE_LEFT = %.1f ", -600.));
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 2: Moving 600mm left...", runtime.milliseconds()));
                    driveTrainEnc.moveLeftRightEnc(-600., 5000);
                }
                if (stepEnabled[16]) { // Forwards
//                    TelemetryWrapper.setLine(7,String.format("MOVE_FORWARDS = %.1f ", 1300.));
                    TelemetryWrapper.setLine(10, String.format("MOVE_FORWARDS = %.1f ; %8.1f Step 3: Moving forwards...", 1300., runtime.milliseconds()));
                    driveTrainEnc.moveForthBackEnc2(1300., 5000);
                }
                if (stepEnabled[17]) { //Backwards
//                    TelemetryWrapper.setLine(7,String.format("MOVE_BACKWARDS = %.1f ", -1300.));
                    TelemetryWrapper.setLine(10, String.format("MOVE_BACKWARDS = %.1f ; %8.1f Step 3: Moving backwards...", -1300., runtime.milliseconds()));
                    driveTrainEnc.moveForthBackEnc2(-1300., 5000);
                }
                if (stepEnabled[18]) {
                    TelemetryWrapper.setLine(7,String.format("CCW Turning = %.1f Degrees.", -90.));
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 8: Turning - 135 degrees for depot...", runtime.milliseconds()));
                    driveTrainEnc.spinEnc(-90., 5000);
                }
                if (stepEnabled[19]) {
                    TelemetryWrapper.setLine(7,String.format("CCW Turning = %.1f Degrees.", 90.));
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 8: Turning - 135 degrees for depot...", runtime.milliseconds()));
                    driveTrainEnc.spinEnc(90., 5000);
                }
                driveTrain.init();
            }

            if (bH1.pressing(left_bumper)) {

                int index = 0;
                //TelemetryWrapper.clear();
                TelemetryWrapper.setLine(0, "*** Setting Mode ***");

                while (! bH1.pressing(right_bumper)) {
                    bH1.update();
                    bH2.update();

                    if (bH1.pressing(dpad_up)) {
                        index++;
                        if (index > 19) {index = 0;}
                        TelemetryWrapper.setLine(2,String.format("dpad_up is pressed. index = %d ", index));
                        //sleep(100);
                    }
                    if (bH1.pressing(dpad_down)) {
                        index--;
                        if (index < 0) {index = 19;}
                        TelemetryWrapper.setLine(2,String.format("dpad_down is pressed. index = %d ", index));
                        //sleep(100);
                    }
                    if (bH1.pressing(dpad_left) ) {
                        stepEnabled[index] = !stepEnabled[index];
                        TelemetryWrapper.setLine(2,String.format("dpad_left is pressed. index = %d ", index));
                        //sleep(100);
                    }
                    if (bH1.pressing(dpad_right)) {
                        stepEnabled[index] = !stepEnabled[index];
                        TelemetryWrapper.setLine(2,String.format("dpad_right is pressed. index = %d ", index));
                        //sleep(100);
                    }
                    TelemetryWrapper.setLine(4, String.format("stepEnable[%d] = %b ", index, stepEnabled[index]));
                }
                //TelemetryWrapper.clearLines(0,10);
            }

        }
        tfod.deactivate();
        //mR.deactivate();
        //robotLoc.deactivate();
    }

    private void initVuforia(int cameraMonitorViewId) {
        /** For Vuforia engine 1 (for Android Camera) */
        VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters1.vuforiaLicenseKey = VUFORIA_KEY;
        parameters1.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //  Instantiate the Vuforia engine 1
        vuforia = ClassFactory.getInstance().createVuforia(parameters1);
    }

    private void initVuforia() {

        /** For Vuforia engine 1 (for Android Camera) */
        //VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters();
        parameters1.vuforiaLicenseKey = VUFORIA_KEY;
        parameters1.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //  Instantiate the Vuforia engine 1
        vuforia = ClassFactory.getInstance().createVuforia(parameters1);
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.minimumConfidence = 0.4; // Added by J.Tu
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
