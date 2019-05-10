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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Parameters.CAMERA_LEFT_DISPLACEMENT;
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
import static org.firstinspires.ftc.teamcode.Parameters.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.Parameters.WALL_TO_DEPOT;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.back;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_down;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_left;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_right;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_up;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.left_bumper;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.right_bumper;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.start;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.x;

@TeleOp(name = "Test Auto Functions on Pilot", group = "Test")
//@Disabled
public class TestAutoFunctionsOnPilot extends LinearOpMode {

    /**
     * Define your variables here
     ********************************************************/
    private ButtonHelper        bH1 = null;
    private ButtonHelper        bH2 = null;
    private ElapsedTime         runtime = new ElapsedTime();
    private Config              config = new Config(Config.configFile);

    boolean[]                   stepEnabled = new boolean[20];

    ElapsedTime                 timeCheckPoint = new ElapsedTime();
    List<TimeRecorder>          timeRecorderList = new ArrayList<TimeRecorder>();
    TimeRecorder                tR = new TimeRecorder();
    int                         checkPoint = 0;
    double                      oldTime = 0;
    double                      newTime = 0;
    double                      deltaTime = 0;

    private GoldPosition        gP = GoldPosition.UNKNOWN;

    private DriveTrain          driveTrain = new DriveTrain();
    private DriveTrainByEncoder driveTrainEnc = new DriveTrainByEncoder();
    private ForeArm             foreArm = new ForeArm();
    private LiftArm             liftArm = new LiftArm();
    private MineralCollector    mineralCollector = new MineralCollector();

    //private static final String VUFORIA_KEY = "AaVQPxH/////AAABmWbgMV3r8kMuucDJZwS+C8IqcKbjimK6x7yZkfsYnCLGA1cHVqGOF+tSmO//7vH+NwYrxmEfltB1UGzWki397Ksrl57wPSMPbGU2y9Cg+iSgHMGpJVx4IDeD6ldnTIRetHFeW0r4OzmfsDc5eI0tChOd2FYv2Q8MuHq/QXlsdOHEOyy43xqj5QF4eRSVznttm6fDzN2egZWEIr8Un9B0hCEv6OmQATKUsEPx7BnqCxjBK00252+n2Na17OxE2hYP8WXUerdZOOU1GyWFPOG2DDeYDWiipgYGXgpIC+a846STiSZcFXLP2S3ENu78EoCFKs7Fw7sm5u58dzZ5PyMg8VUormyNmcHm9RU2Fl5364WO";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private VuforiaLocalizer    vuforia;
    private TFObjectDetector    tfod;
    private MineralRecognizer   mR;
    private RobotLocator        robotLoc;

    /**
     * End of Definition
     ****************************************************************/

    @Override
    public void runOpMode() {


        /** Initialize the variables here ************************************************/
        for (int i = 0; i < 20; i++) {
            stepEnabled[i] = false;
        }

        bH1 = new ButtonHelper(gamepad1);
        bH2 = new ButtonHelper(gamepad2);

        TelemetryWrapper.init(telemetry, 100);

        Parameters.init(config);
        Hardware2019.init(hardwareMap);

        liftArm.initEnc();
        foreArm.initEnc();
        mineralCollector.init();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        initVuforia(cameraMonitorViewId);
        initTfod();
        tfod.activate();

        mR = new MineralRecognizer();
        mR.initialize(tfod);

        robotLoc = new RobotLocator();
        robotLoc.initialize(vuforia);
        robotLoc.activate();

        double distToMoveToWall = 0;
        double angleToTurnAtWall = 0;
        /** End of Initialization *********************************************************/

        waitForStart();

        //Time Check Point 0
        setTimeRecordCheckPoint("Start...");
        ////////////////////

        int mainloop = 0;
        while (opModeIsActive()) {
            mainloop++;

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
//                drive_x *= 0.8;
//                drive_y *= 0.6;
//                turn *= 0.5;
                driveTrain.moveFree(drive_x, drive_y, turn);
            }

            //////////////////////// TFOD mineral recognizer ////////////////////////////////////////////////
            //Time Check Point 1
            setTimeRecordCheckPoint("mR.update() Starts");
            //////////

            mR.update();

            //Time Check Point 2
            setTimeRecordCheckPoint("mR.update() Finished");
            //////////

            gP = mR.getGoldPosition();
            TelemetryWrapper.setLine(0, String.format("mR updated in MAIN LOOP (%d). \n(numM, Gold?, gP, Angle1) = (%d,  %b,  %s,  %.1f)",
                    mainloop, mR.getNumM(), mR.goldIsFound(), gP.toString(), ((mR.getNumM() == 1) ? mR.get(0).getAngle() : 90.0)));

            //Time Check Point 3
            setTimeRecordCheckPoint("robotLoc.update() starts");
            //////////

            robotLoc.update();

            //Time Check Point 4
            setTimeRecordCheckPoint("robotLoc.update() finished");
            //////////


            TelemetryWrapper.setLine(2, String.format("(Visible) = (%b) \n(x,y,z)=( %.1f,  %.1f,  %.1f ) \n(A1,A2,A3) = ( %.1f,  %.1f,  %.1f)",
                    robotLoc.targetIsVisible(),
                    robotLoc.getLocX(), robotLoc.getLocY(), robotLoc.getLocZ(),
                    robotLoc.getAngle1(), robotLoc.getAngle2(), robotLoc.getAngle3()));


            /////////////////// Choose steps to test ///////////////////////
            if (bH1.pressing(start)) {
                driveTrainEnc.initEnc();
                if (stepEnabled[0]) {
                    //
                }
                if (stepEnabled[1]) { //《========================== Test Case 01 =======================

                    /* Landing off */

                    //Time Check Point 5
                    setTimeRecordCheckPoint("Landing off starts");
                    //////////

                    liftArm.landOffEnc(10000);

                    //Time Check Point 6
                    setTimeRecordCheckPoint("Landing off finished");
                    //////////

                    /* After landing, move a FIRST_MOVE_RIGHT to let the hook get out of the latch */

                    //Time Check Point 7
                    setTimeRecordCheckPoint("FIRST_MOVE_RIGHT starts");
                    //////////

                    driveTrainEnc.moveLeftRightEnc(FIRST_MOVE_RIGHT, 5000);

                    //Time Check Point 8
                    setTimeRecordCheckPoint("FIRST_MOVE_RIGHT finished");
                    //////////

                    angleToTurnAtWall = -135.0;

                    runtime.reset();

                    /* Quick (for 1 second at most) check for the GOLD mineral */


                    //Time Check Point 9
                    setTimeRecordCheckPoint("Gold position Quick check starts");
                    //////////

                    int quick_times = 0;
                    while ((gP == GoldPosition.UNKNOWN) && (runtime.milliseconds() < 1000)) {
                        quick_times++;
                        mR.update();
                        gP = mR.getGoldPosition();
                    }

                    //Time Check Point 10
                    setTimeRecordCheckPoint("Gold position Quick check finished");
                    //////////

                    /* If 3 minerals are detected and the GOLD mineral's position is examined,
                     * the robot will finished the first steps automatically:
                     * 1. move an INITIAL_MOVE_TO_MINERAL
                     * 2. turn to the GOLD mineral if needed
                     * 3. move the GOLD mineral of its place
                     * 4. move back to the initial start place and get ready to move to the wall
                     * */
                    if (gP != GoldPosition.UNKNOWN) {
                        TelemetryWrapper.setLine(0, String.format("Gold found. gP: %s", gP.toString()));
                        switch (gP) {
                            case MIDDLE:
                                TelemetryWrapper.setLine(3, String.format("Sure Middle : (numM, Gold?, gP) = (%d, %b, %s)",
                                        mR.getNumM(), mR.goldIsFound(), mR.getGoldPosition().toString()));
                                TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));

                                //Time Check Point 11
                                setTimeRecordCheckPoint("Quick MIDDLE: starts");
                                //////////
                                driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                                driveTrainEnc.moveLeftRightEnc(-FIRST_MOVE_RIGHT, 5000);
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);
                                driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                                driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL, 8000);
                                //Time Check Point 12
                                setTimeRecordCheckPoint("Quick MIDDLE: finished");
                                //////////

                                angleToTurnAtWall = -135.0;

                            case RIGHT:
                                //Time Check Point 13
                                setTimeRecordCheckPoint("Quick RIGHT: starts");
                                //////////
                                driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                                driveTrainEnc.moveLeftRightEnc(-FIRST_MOVE_RIGHT, 5000);
                                driveTrainEnc.spinEnc(45.0, 5000);
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD * 1.4, 5000);
                                driveTrainEnc.moveForthBackEnc(-PULL_BACK * 1.4, 5000);
                                driveTrainEnc.spinEnc(-45.0, 5000);
                                driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL, 8000);
                                //Time Check Point 14
                                setTimeRecordCheckPoint("Quick RIGHT: finished");
                                //////////

                                angleToTurnAtWall = -135.0;
                            case LEFT:
                                //Time Check Point 15
                                setTimeRecordCheckPoint("Quick LEFT: starts");
                                //////////
                                driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                                driveTrainEnc.moveLeftRightEnc(-FIRST_MOVE_RIGHT, 5000);
                                TelemetryWrapper.setLine(3, String.format("Sure Left : (numM, Gold?,gP) = (%d, %b, %s)",
                                        mR.getNumM(), mR.goldIsFound(), mR.getGoldPosition().toString()));
                                driveTrainEnc.spinEnc(-45.0, 5000);
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD * 1.4, 5000);
                                driveTrainEnc.moveLeftRightEnc(-PUSH_GOLD, 5000);
                                driveTrainEnc.moveForthBackEnc(DISTANCE_TO_WALL - PUSH_GOLD, 8000);
                                //Time Check Point 16
                                setTimeRecordCheckPoint("Quick LEFT: finished");
                                //////////

                                angleToTurnAtWall = -45.0;
                        }
                    }
                    /* Else, to detect the three positions respectively.
                     * First, detect the middle place, if yes, push the gold and pull back
                     * Second, the right place, if yes, push the gold and pull back
                     * Third, push the left mineral directly without detection
                     */
                    else {
                        //TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));
                        /* Move to the initial start-up place
                         * 1. Move forward with a distance of  INITIAL_MOVE_TO_MINERAL
                         * 2. Move left-right to let the cameral be on the center line
                         * */

                        //Time Check Point 17
                        setTimeRecordCheckPoint("non-Quick: [Initial move to CENTER] starts");
                        //////////
                        driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                        driveTrainEnc.moveLeftRightEnc(-FIRST_MOVE_RIGHT + CAMERA_LEFT_DISPLACEMENT, 5000);
                        //Time Check Point 18
                        setTimeRecordCheckPoint("non-Quick: [Initial move to CENTER] finished");
                        //////////

                        /* Detect if the middle one is a gold mineral*/

                        //Time Check Point 19
                        setTimeRecordCheckPoint("non-Quick: [mR.update() for CENTER] starts");
                        //////////
                        runtime.reset();
                        mR.update();
                        while ((mR.getNumM() < 1) && (runtime.milliseconds() < MINERAL_DETECT_TIMELIMIT)) {
                            mR.update();
                        }
                        //Time Check Point 20
                        setTimeRecordCheckPoint("non-Quick: [mR.update() for CENTER] finished");
                        //////////

                        /* If the GOLD is at the CENTER ... */
                        if (mR.goldIsFound()) { //gold is at the CENTER
                            //Time Check Point 21
                            setTimeRecordCheckPoint("non-Quick CENTER: [Frome PUSH_GOLD ... move to wall] starts");
                            //////////
                            driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);
                            driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                            driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL, 8000);
                            //Time Check Point 22
                            setTimeRecordCheckPoint("non-Quick CENTER: [Frome PUSH_GOLD ... move to wall] finished");
                            //////////

                            angleToTurnAtWall = -135.0;
                        } else {
                            /* If not at the center, then try the RIGHT:
                             * turn 45 degrees right, and detect ...
                             * */

                            //Time Check Point 23
                            setTimeRecordCheckPoint("non-Quick: [Turn to right, check for RIGHT] starts");
                            //////////
                            driveTrainEnc.spinEnc(45.0, 5000);
                            //Time Check Point 24
                            setTimeRecordCheckPoint("non-Quick: [Turn to right, check for RIGHT] finished");
                            //////////

                            //Time Check Point 25
                            setTimeRecordCheckPoint("non-Quick: [mR.update() for RIGHT] starts");
                            //////////
                            runtime.reset();
                            mR.update();
                            while ((mR.getNumM() < 1) && (runtime.milliseconds() < MINERAL_DETECT_TIMELIMIT)) {
                                mR.update();
                            }
                            //Time Check Point 26
                            setTimeRecordCheckPoint("non-Quick: [mR.update() for RIGHT] finished");
                            //////////

                            /* If the GOLD on the RIGHT ... */
                            if (mR.goldIsFound()) { //gold is on the RIGHT
                                //Time Check Point 27
                                setTimeRecordCheckPoint("non-Quick RIGHT: [Frome PUSH_GOLD ... move to wall] starts");
                                //////////
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD * 1.4, 5000);
                                driveTrainEnc.moveForthBackEnc(-PULL_BACK * 1.4, 5000);
                                driveTrainEnc.spinEnc(-45.0, 5000);
                                driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL, 8000);
                                //Time Check Point 28
                                setTimeRecordCheckPoint("non-Quick RIGHT: [Frome PUSH_GOLD ... move to wall] finished");
                                //////////

                                angleToTurnAtWall = -135.0;
                            } else {
                                /* If the GOLD in not on the right, then it must be on the LEFT,
                                 * not detection needed
                                 * */

                                //Time Check Point 29
                                setTimeRecordCheckPoint("non-Quick LEFT: [Frome spin 90 degrees left ... move to wall] starts");
                                //////////
                                driveTrainEnc.spinEnc(-90.0, 5000);
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD * 1.4, 5000);
                                driveTrainEnc.moveLeftRightEnc(-PUSH_GOLD, 5000);
                                driveTrainEnc.spinEnc(-45.0, 5000);
                                driveTrainEnc.moveForthBackEnc(DISTANCE_TO_WALL - PUSH_GOLD, 8000);
                                //Time Check Point 30
                                setTimeRecordCheckPoint("non-Quick LEFT: [Frome spin 90 degrees left ... move to wall] starts");
                                //////////

                                angleToTurnAtWall = -45.0;
                            }
                        }
                    }

                    /* Turn to face to depot */
                    //Time Check Point 31
                    setTimeRecordCheckPoint("[Turn to face to depot] starts");
                    //////////
                    driveTrainEnc.spinEnc(angleToTurnAtWall, 5000);

                    /* Drive to depot to set the team marker */
                    //Time Check Point 32
                    setTimeRecordCheckPoint("[Move to depot] starts");
                    //////////
                    driveTrainEnc.moveForthBackEnc(WALL_TO_DEPOT, 5000);

                    /* Put down the forearm so to get it ready to set the team marker */
                    //Time Check Point 33
                    setTimeRecordCheckPoint("[Put down the forearm] starts");
                    //////////
                    foreArm.moveUpDownAngleEnc(-60., 5000);

                    /* Stretch out the forearm to put the team marker into the depot zone */
                    //!!!!!!!!!! Might be unnecessary if the robot moves fast and long enough to crater
                    //Time Check Point 34
                    setTimeRecordCheckPoint("[Stretch out the forearm] starts");
                    //////////
                    foreArm.moveForwardEnc(5000);

                    /* Wipe out to set the team marker on to the depot */
                    //Time Check Point 35
                    setTimeRecordCheckPoint("[Wipe out the team marker (1 second waiting and wipe stopping included)] starts");
                    //////////
                    mineralCollector.wipeOut();
                    sleep(1000); // wait 1 second for the team marker to be wiped out to the depot

                    /* Stop the mineral collector's rotation (wiping out) */
                    mineralCollector.wipeStop();

                    /* Withdraw the forearm after the team marker is set onto the depot */
                    //Time Check Point 36
                    setTimeRecordCheckPoint("[Pull back the forearm] starts");
                    //////////
                    foreArm.moveBackwardEnc(5000);

                    /* Move up the forearm to avoid any obstacle */
                    //Time Check Point 37
                    setTimeRecordCheckPoint("[Move up the forearm] starts");
                    //////////
                    foreArm.moveUpDownAngleEnc(60, 5000);

                    /* Turn back */
                    //Time Check Point 38
                    setTimeRecordCheckPoint("[Turn Back (spin -180)] starts");
                    //////////
                    driveTrainEnc.spinEnc(-180, 10000);

                    /* Move back to crater */
                    //Time Check Point 39
                    setTimeRecordCheckPoint("[Move to crater] starts");
                    //////////
                    driveTrainEnc.moveForthBackEnc(DEPOT_TO_CRATER, 10000);

                    /* Put the foreArm on to the crater, to earn the parking points */
                    //Time Check Point 40
                    setTimeRecordCheckPoint("[Put down the forearm] starts");
                    //////////
                    foreArm.moveUpDownAngleEnc(-60., 5000);
                    //Time Check Point 41
                    setTimeRecordCheckPoint("[Put down the forearm] finished");
                    //////////

                    if (timeRecorderList.size() > 0) {
                        TelemetryWrapper.clear();
                        int line = 0;
                        for (TimeRecorder tr : timeRecorderList) {
                            TelemetryWrapper.setLine(line, tr.toString());
                        }
                    }
                    while (!bH1.pressing(back)) {
                    }
                }
                if (stepEnabled[2]) {  //《========================== Test Case 02 =======================

                    /* Landing off */

                    //Time Check Point 5
                    setTimeRecordCheckPoint("Landing off starts");
                    //////////

                    liftArm.landOffEnc(10000);

                    //Time Check Point 6
                    setTimeRecordCheckPoint("Landing off finished");
                    //////////

                    /* After landing, move a FIRST_MOVE_RIGHT to let the hook get out of the latch */

                    //Time Check Point 7
                    setTimeRecordCheckPoint("FIRST_MOVE_RIGHT starts");
                    //////////

                    driveTrainEnc.moveLeftRightEnc(FIRST_MOVE_RIGHT, 5000);

                    //Time Check Point 8
                    setTimeRecordCheckPoint("FIRST_MOVE_RIGHT finished");
                    //////////

                    angleToTurnAtWall = -135.0;
                    runtime.reset();

                    /* Quick (for 1 second at most) check for the GOLD mineral */

                    //Time Check Point 9
                    setTimeRecordCheckPoint("Gold position Quick check starts");
                    //////////

                    int quick_times = 0;
                    while ((gP == GoldPosition.UNKNOWN) && (runtime.milliseconds() < 1000)) {
                        quick_times++;
                        TelemetryWrapper.setLine(0, String.format("Quick updating mR in START branch: %d", quick_times));
                        mR.update();
                        gP = mR.getGoldPosition();
                    }

                    //Time Check Point 10
                    setTimeRecordCheckPoint("Gold position Quick check finished");
                    //////////

                    /* If 3 minerals are detected and the GOLD mineral's position is examined,
                     * the robot will finished the first steps automatically:
                     * 1. move an INITIAL_MOVE_TO_MINERAL
                     * 2. turn to the GOLD mineral if needed
                     * 3. move the GOLD mineral of its place
                     * 4. move back to the initial start place and get ready to move to the wall
                     * */
                    if (gP != GoldPosition.UNKNOWN) {
                        TelemetryWrapper.setLine(0, String.format("Gold found. gP: %s", gP.toString()));
                        switch (gP) {
                            case MIDDLE:
                                TelemetryWrapper.setLine(3, String.format("Sure Middle : (numM, Gold?, gP) = (%d, %b, %s)",
                                        mR.getNumM(), mR.goldIsFound(), mR.getGoldPosition().toString()));
                                TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));

                                //Time Check Point 11
                                setTimeRecordCheckPoint("Quick MIDDLE: starts");
                                //////////
                                driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                                driveTrainEnc.moveLeftRightEnc(-FIRST_MOVE_RIGHT, 5000);

                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);

                                //Time Check Point 11-1
                                setTimeRecordCheckPoint("Quick MIDDLE: [Set the team marker] starts");
                                //////////
                                //Sets the team marker
                                foreArm.moveUpDownAngleEnc(-60, 5000);
                                foreArm.moveForwardEnc(5000);
                                mineralCollector.wipeOut();
                                sleep(500);
                                foreArm.moveBackwardEnc(5000);
                                foreArm.moveUpDownAngleEnc(60, 5000);
                                //Time Check Point 11-2
                                setTimeRecordCheckPoint("Quick MIDDLE: [Set the team marker] finished");
                                //////////

                                driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                                driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL, 8000);
                                //Time Check Point 12
                                setTimeRecordCheckPoint("Quick MIDDLE: finished");
                                //////////

                                //distToMoveToWall = DISTANCE_TO_WALL;
                                angleToTurnAtWall = -135.0;

                            case RIGHT:
                                TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));

                                //Time Check Point 13
                                setTimeRecordCheckPoint("Quick RIGHT: starts");
                                //////////
                                driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                                driveTrainEnc.moveLeftRightEnc(DIST_BTWN_MINERALS - FIRST_MOVE_RIGHT, 5000);
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);

                                //Time Check Point 13-1
                                setTimeRecordCheckPoint("Quick RIGHT: [Set the team marker] starts");
                                //////////
                                //Sets the team marker
                                foreArm.moveUpDownAngleEnc(-60, 5000);
                                foreArm.moveForwardEnc(5000);
                                mineralCollector.wipeOut();
                                sleep(500);
                                foreArm.moveBackwardEnc(5000);
                                foreArm.moveUpDownAngleEnc(60, 5000);
                                //Time Check Point 13-2
                                setTimeRecordCheckPoint("Quick RIGHT: [Set the team marker] finished");
                                //////////

                                driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                                driveTrainEnc.moveLeftRightEnc(-(DISTANCE_TO_WALL + DIST_BTWN_MINERALS), 8000);
                                //Time Check Point 14
                                setTimeRecordCheckPoint("Quick RIGHT: finished");
                                //////////

                                //distToMoveToWall = DISTANCE_TO_WALL + DIST_BTWN_MINERALS;
                                angleToTurnAtWall = -135.0;
                            case LEFT:
                                TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));

                                //Time Check Point 15
                                setTimeRecordCheckPoint("Quick LEFT: starts");
                                //////////
                                driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                                driveTrainEnc.moveLeftRightEnc(-(FIRST_MOVE_RIGHT + DIST_BTWN_MINERALS), 5000);
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);

                                //Time Check Point 15-1
                                setTimeRecordCheckPoint("Quick RIGHT: [Set the team marker] starts");
                                //////////
                                //Sets the team marker
                                foreArm.moveUpDownAngleEnc(-60, 5000);
                                foreArm.moveForwardEnc(5000);
                                mineralCollector.wipeOut();
                                sleep(500);
                                foreArm.moveBackwardEnc(5000);
                                foreArm.moveUpDownAngleEnc(60, 5000);
                                //Time Check Point 15-2
                                setTimeRecordCheckPoint("Quick RIGHT: [Set the team marker] finished");
                                //////////

                                driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                                driveTrainEnc.moveForthBackEnc(DISTANCE_TO_WALL - PUSH_GOLD, 8000);
                                //Time Check Point 16
                                setTimeRecordCheckPoint("Quick LEFT: finished");
                                //////////

                                angleToTurnAtWall = -45.0;
                        }
                    }
                    /* Else, to detect the three positions respectively.
                     * First, detect the middle place, if yes, push the gold and pull back
                     * Second, the right place, if yes, push the gold and pull back
                     * Third, push the left mineral directly without detection
                     */
                    else {
                        //TelemetryWrapper.setLine(1, String.format("%8.1f Step 3: Moving to initial position...", runtime.milliseconds()));
                        /* Move to the initial start-up place
                         * 1. Move forward with a distance of  INITIAL_MOVE_TO_MINERAL
                         * 2. Move left-right to let the cameral be on the center line
                         * */

                        //Time Check Point 17
                        setTimeRecordCheckPoint("non-Quick: [Initial move to CENTER] starts");
                        //////////
                        driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                        driveTrainEnc.moveLeftRightEnc(-FIRST_MOVE_RIGHT + CAMERA_LEFT_DISPLACEMENT, 5000);
                        //Time Check Point 18
                        setTimeRecordCheckPoint("non-Quick: [Initial move to CENTER] finished");
                        //////////

                        /* Detect if the middle one is a gold mineral*/

                        //Time Check Point 19
                        setTimeRecordCheckPoint("non-Quick: [mR.update() for CENTER] starts");
                        //////////
                        runtime.reset();
                        mR.update();
                        while ((mR.getNumM() < 1) && (runtime.milliseconds() < MINERAL_DETECT_TIMELIMIT)) {
                            mR.update();
                        }
                        //Time Check Point 20
                        setTimeRecordCheckPoint("non-Quick: [mR.update() for CENTER] finished");
                        //////////

                        /* If the GOLD is at the CENTER ... */
                        if (mR.goldIsFound()) { //gold is at the CENTER
                            TelemetryWrapper.setLine(3, String.format("Single Center : (numM, Gold?) = (%d, %b)", mR.getNumM(), mR.goldIsFound()));

                            //Time Check Point 21
                            setTimeRecordCheckPoint("non-Quick CENTER: [Frome PUSH_GOLD ... move to wall] starts");
                            //////////
                            driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);

                            //Time Check Point 21-1
                            setTimeRecordCheckPoint("Quick RIGHT: [Set the team marker] starts");
                            //////////
                            //Sets the team marker
                            foreArm.moveUpDownAngleEnc(-60, 5000);
                            foreArm.moveForwardEnc(5000);
                            mineralCollector.wipeOut();
                            sleep(500);
                            foreArm.moveBackwardEnc(5000);
                            foreArm.moveUpDownAngleEnc(60, 5000);
                            //Time Check Point 21-2
                            setTimeRecordCheckPoint("Quick RIGHT: [Set the team marker] finished");
                            //////////

                            driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                            driveTrainEnc.moveLeftRightEnc(-(DISTANCE_TO_WALL + CAMERA_LEFT_DISPLACEMENT), 8000);
                            //Time Check Point 22
                            setTimeRecordCheckPoint("non-Quick CENTER: [Frome PUSH_GOLD ... move to wall] finished");
                            //////////

                            angleToTurnAtWall = -135.0;
                        } else {
                            /* If not at the center, then try the RIGHT:
                             * turn 45 degrees right, and detect ...
                             * */

                            //Time Check Point 23
                            setTimeRecordCheckPoint("non-Quick: [Move to right to check for RIGHT] starts");
                            //////////
                            //driveTrainEnc.spinEnc(45.0,5000);
                            driveTrainEnc.moveLeftRightEnc(DIST_BTWN_MINERALS, 5000);
                            //Time Check Point 24
                            setTimeRecordCheckPoint("non-Quick: [Move to right to check for RIGHT] finished");
                            //////////

                            //Time Check Point 25
                            setTimeRecordCheckPoint("non-Quick: [mR.update() for RIGHT] starts");
                            //////////
                            runtime.reset();
                            mR.update();
                            while ((mR.getNumM() < 1) && (runtime.milliseconds() < MINERAL_DETECT_TIMELIMIT)) {
                                mR.update();
                            }
                            //Time Check Point 26
                            setTimeRecordCheckPoint("non-Quick: [mR.update() for RIGHT] finished");
                            //////////

                            /* If the GOLD on the RIGHT ... */
                            if (mR.goldIsFound()) { //gold is on the RIGHT
                                TelemetryWrapper.setLine(3, String.format("Single Right : (numM, Gold?) = (%d, %b)", mR.getNumM(), mR.goldIsFound()));

                                //Time Check Point 27
                                setTimeRecordCheckPoint("non-Quick RIGHT: [Frome PUSH_GOLD ... move to wall] starts");
                                //////////
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);

                                //Time Check Point 27-1
                                setTimeRecordCheckPoint("Quick RIGHT: [Set the team marker] starts");
                                //////////
                                //Sets the team marker
                                foreArm.moveUpDownAngleEnc(-60, 5000);
                                foreArm.moveForwardEnc(5000);
                                mineralCollector.wipeOut();
                                sleep(500);
                                foreArm.moveBackwardEnc(5000);
                                foreArm.moveUpDownAngleEnc(60, 5000);
                                //Time Check Point 27-2
                                setTimeRecordCheckPoint("Quick RIGHT: [Set the team marker] finished");
                                //////////

                                driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                                driveTrainEnc.moveLeftRightEnc(-(DISTANCE_TO_WALL + DIST_BTWN_MINERALS + CAMERA_LEFT_DISPLACEMENT), 8000);
                                //Time Check Point 28
                                setTimeRecordCheckPoint("non-Quick RIGHT: [Frome PUSH_GOLD ... move to wall] finished");
                                //////////

                                angleToTurnAtWall = -135.0;
                            } else {
                                /* If the GOLD in not on the right, then it must be on the LEFT,
                                 * not detection needed
                                 * */
                                TelemetryWrapper.setLine(3, String.format("Supposed Right : (numM, Gold?) = (%d, %b)", mR.getNumM(), mR.goldIsFound()));

                                //Time Check Point 29
                                setTimeRecordCheckPoint("non-Quick LEFT: [Frome spin 90 degrees left ... move to wall] starts");
                                //////////
                                //driveTrainEnc.spinEnc(-90.0,5000);
                                driveTrainEnc.moveLeftRightEnc(-DIST_BTWN_MINERALS, 5000);
                                driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);

                                //Time Check Point 29-1
                                setTimeRecordCheckPoint("Quick RIGHT: [Set the team marker] starts");
                                //////////
                                //Sets the team marker
                                foreArm.moveUpDownAngleEnc(-60, 5000);
                                foreArm.moveForwardEnc(5000);
                                mineralCollector.wipeOut();
                                sleep(500);
                                foreArm.moveBackwardEnc(5000);
                                foreArm.moveUpDownAngleEnc(60, 5000);
                                //Time Check Point 29-2
                                setTimeRecordCheckPoint("Quick RIGHT: [Set the team marker] finished");
                                //////////


                                driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                                //driveTrainEnc.spinEnc(-45.0,5000);
                                driveTrainEnc.moveLeftRightEnc(-(DISTANCE_TO_WALL - DIST_BTWN_MINERALS + CAMERA_LEFT_DISPLACEMENT), 8000);
                                //Time Check Point 30
                                setTimeRecordCheckPoint("non-Quick LEFT: [Frome spin 90 degrees left ... move to wall] starts");
                                //////////

                                angleToTurnAtWall = -135.0;
                            }
                        }
                    }

                    /* Turn to face to depot */
                    //Time Check Point 31
                    setTimeRecordCheckPoint("[Turn to face to crater] starts");
                    //////////
                    driveTrainEnc.spinEnc(angleToTurnAtWall, 5000);


                    /* Move back to crater */
                    //Time Check Point 32
                    setTimeRecordCheckPoint("[Move to crater] starts");
                    //////////
                    driveTrainEnc.moveForthBackEnc(DEPOT_TO_CRATER, 10000);

                    /* Put the foreArm on to the crater, to earn the parking points */
                    //Time Check Point 33
                    setTimeRecordCheckPoint("[Put down the forearm] starts");
                    //////////
                    foreArm.moveUpDownAngleEnc(-60., 5000);
                    //Time Check Point 34
                    setTimeRecordCheckPoint("[Put down the forearm] finished");
                    //////////

                    if (timeRecorderList.size() > 0) {
                        TelemetryWrapper.clear();
                        int line = 0;
                        for (TimeRecorder tr : timeRecorderList) {
                            TelemetryWrapper.setLine(line, tr.toString());
                        }
                    }
                    while (!bH1.pressing(back)) {
                    }
                }

                if (stepEnabled[3]) {

                }
                if (stepEnabled[4]) {

                }
                if (stepEnabled[5]) {

                }
                if (stepEnabled[6]) {

                }
                if (stepEnabled[7]) {

                }
                if (stepEnabled[8]) {

                }
                if (stepEnabled[9]) {

                }
                if (stepEnabled[10]) {

                }
                if (stepEnabled[11]) {

                }
                if (stepEnabled[12]) {

                }
                if (stepEnabled[13]) {

                }
                ////////////// Test for justifying movement XXXX_CORRECTION parameters //////////////
                /* Left/Right to test XY_CORRECTION */
                if (stepEnabled[14]) { //To Right
//                    TelemetryWrapper.setLine(7,String.format("FIRST_MOVE_RIGHT = %.1f ", 600.));
                    TelemetryWrapper.setLine(10, String.format("FIRST_MOVE_RIGHT = %.1f ; %8.1f Step 1: Moving 600mm right...", 600., runtime.milliseconds()));
                    driveTrainEnc.moveLeftRightEnc(600., 5000);
                }
                if (stepEnabled[15]) { // To Left
                    TelemetryWrapper.setLine(7, String.format("MOVE_LEFT = %.1f ", -600.));
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 2: Moving 600mm left...", runtime.milliseconds()));
                    driveTrainEnc.moveLeftRightEnc(-600., 5000);
                }
                /* Forwards/Backwards to test LINE_CORRECTION */
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
                /* CW/CCW to test DEGREE_CORRECTION */
                if (stepEnabled[18]) {
                    TelemetryWrapper.setLine(7, String.format("CCW Turning = %.1f Degrees.", -90.));
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 8: Turning - 135 degrees for depot...", runtime.milliseconds()));
                    driveTrainEnc.spinEnc(-90., 5000);
                }
                if (stepEnabled[19]) {
                    TelemetryWrapper.setLine(7, String.format("CCW Turning = %.1f Degrees.", 90.));
                    TelemetryWrapper.setLine(1, String.format("%8.1f Step 8: Turning - 135 degrees for depot...", runtime.milliseconds()));
                    driveTrainEnc.spinEnc(90., 5000);
                }
                driveTrainEnc.stop();
            }

            if (bH1.pressing(left_bumper)) {

                int index = 0;
                //TelemetryWrapper.clear();
                TelemetryWrapper.setLine(0, "*** Setting Mode ***");

                while (!bH1.pressing(right_bumper)) {
                    bH1.update();
                    bH2.update();

                    TelemetryWrapper.setLine(2, String.format("Initial start-up. index = %d ", index));
                    if (bH1.pressing(dpad_up)) {
                        index++;
                        if (index > 19) {
                            index = 0;
                        }
                        TelemetryWrapper.setLine(2, String.format("dpad_up is pressed. index = %d ", index));
                    }
                    if (bH1.pressing(dpad_down)) {
                        index--;
                        if (index < 0) {
                            index = 19;
                        }
                        TelemetryWrapper.setLine(2, String.format("dpad_down is pressed. index = %d ", index));
                    }
                    if (bH1.pressing(dpad_left)) {
                        stepEnabled[index] = !stepEnabled[index];
                        TelemetryWrapper.setLine(2, String.format("dpad_left is pressed. index = %d ", index));
                    }
                    if (bH1.pressing(dpad_right)) {
                        stepEnabled[index] = !stepEnabled[index];
                        TelemetryWrapper.setLine(2, String.format("dpad_right is pressed. index = %d ", index));
                        //sleep(100);
                    }
                    TelemetryWrapper.setLine(4, String.format("stepEnable[%d] = %b ", index, stepEnabled[index]));
                }
                //TelemetryWrapper.clearLines(0,10);
            }
        }
        tfod.deactivate();
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

    // The variables: oldTime, newTime, deltaTime, checkPoint, timeRecorderList should be initialized in
    // the beginning of the OpMode
    private void setTimeRecordCheckPoint(String stepDesc) {
        newTime = timeCheckPoint.milliseconds();
        if ( checkPoint == 0) {
            deltaTime = 0;
        } else {
            deltaTime = (newTime - oldTime) / 1000;
        }
        tR.updateWith(checkPoint,stepDesc,newTime,deltaTime);
        timeRecorderList.add(tR);
        oldTime = newTime;
        checkPoint ++;
    }

    class TimeRecorder {
        private int stepNo;
        private String stepDesc;
        private double timeStampMS;
        private double timeDeltaS;

        public void setStepNo(int stepNo) {
            this.stepNo = stepNo;
        }

        public void setStepDesc(String stepDesc) {
            this.stepDesc = stepDesc;
        }

        public void setTimeStampMS(double timeStampMS) {
            this.timeStampMS = timeStampMS;
        }

        public void setTimeDeltaS(double timeDeltaS) {
            this.timeDeltaS = timeDeltaS;
        }

        public int getStepNo() {
            return stepNo;
        }

        public String getStepDesc() {
            return stepDesc;
        }

        public double getTimeStampMS() {
            return timeStampMS;
        }

        public double getTimeDeltaS() {
            return timeDeltaS;
        }

        public void updateWith(int stepNo, String stepDesc, double timeStampMS, double timeDeltaS) {
            this.stepNo = stepNo;
            this.stepDesc = stepDesc;
            this.timeStampMS = timeStampMS;
            this.timeDeltaS = timeDeltaS;
        }

        public void copyFrom(TimeRecorder timeRecorder) {
            this.stepNo = timeRecorder.stepNo;
            this.stepDesc = timeRecorder.stepDesc;
            this.timeStampMS = timeRecorder.timeStampMS;
            this.timeDeltaS = timeRecorder.timeDeltaS;
        }

        @Override public String toString() {
            return  String.format("Step %02d: [ %15.3f ] : [ %10.3f ] : %s ",stepNo, timeStampMS, timeDeltaS, stepDesc);
        }

    }

}
