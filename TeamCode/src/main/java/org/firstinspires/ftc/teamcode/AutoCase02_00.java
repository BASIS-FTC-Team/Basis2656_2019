package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.camera2.CameraManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.Config;

import static org.firstinspires.ftc.teamcode.Parameters.ANGLE_AUTO_UPDOWN;
import static org.firstinspires.ftc.teamcode.Parameters.CAMERA_LEFT_DISPLACEMENT;
import static org.firstinspires.ftc.teamcode.Parameters.COUNTS_FOREARM_BACKWARD;
import static org.firstinspires.ftc.teamcode.Parameters.DEPOT_TO_CRATER;
import static org.firstinspires.ftc.teamcode.Parameters.DISTANCE_TO_WALL;
import static org.firstinspires.ftc.teamcode.Parameters.DIST_BTWN_MINERALS;
import static org.firstinspires.ftc.teamcode.Parameters.DIST_TO_ADJUST_BY_WALL;
import static org.firstinspires.ftc.teamcode.Parameters.FIRST_MOVE_RIGHT;
import static org.firstinspires.ftc.teamcode.Parameters.INITIAL_DIST_TO_WALL;
import static org.firstinspires.ftc.teamcode.Parameters.INITIAL_MOVE_TO_MINERAL;
import static org.firstinspires.ftc.teamcode.Parameters.MAX_COUNTS_FOREARM_FORWARD;
import static org.firstinspires.ftc.teamcode.Parameters.MINERAL_DETECT_TIMELIMIT;
import static org.firstinspires.ftc.teamcode.Parameters.PULL_BACK;
import static org.firstinspires.ftc.teamcode.Parameters.PUSH_GOLD;
import static org.firstinspires.ftc.teamcode.Parameters.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.Parameters.WALL_TO_DEPOT;


@Autonomous(name="AutoCase02_00",group = "Basis2656_2019")
//@Disabled

public class AutoCase02_00 extends LinearOpMode {


    /*** Define your variables here ********************************************************/
//    private ButtonHelper        bH1 = null;
//    private ButtonHelper        bH2 = null;
    private ElapsedTime         runtime = new ElapsedTime();
    private Config              config = new Config(Config.configFile);

//    boolean[]                   stepEnabled = new boolean[20];

//    ElapsedTime                 timeCheckPoint = new ElapsedTime();
//    List<TimeRecorder>          timeRecorderList = new ArrayList<TimeRecorder>();
//    TimeRecorder                tR = new TimeRecorder();
//    int                         checkPoint = 0;
//    double                      oldTime = 0;
//    double                      newTime = 0;
//    double                      deltaTime = 0;

    private GoldPosition        gP = GoldPosition.UNKNOWN;

//    private DriveTrain          driveTrain = new DriveTrain();
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

    private CameraManager mCameraManager;
    //private boolean flashlightIsOn = false;

    @Override
    public void runOpMode() {

        /***************************** Start of Initializations ****************************/

        Parameters.init(config);
        Hardware2019.init(hardwareMap);

        driveTrainEnc.initEnc();
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

        mCameraManager = (CameraManager) hardwareMap.appContext.getSystemService(Context.CAMERA_SERVICE);

        /** End of Initialization *********************************************************/

        waitForStart();


        //////////////////////////////  End of Initializations ///////////////////////////////


        /////////////////////////////  Start of Autonomous Actions  /////////////////////////

        /** Landing off */
        liftArm.landOffEnc(4000);

        /** Quick (for 1 second at most) check for the GOLD mineral */
        turnLightOn();
        int quick_times = 1;
        mR.update();
        while ((!mR.goldIsFound()) && (runtime.milliseconds() < 500)) {
            mR.update();
            quick_times++;
        }
        turnLightOff();
        if (mR.getFirstGoldOrderFromLeft() == 1) {
            gP = GoldPosition.LEFT;
        } else if (mR.getFirstGoldOrderFromLeft() == 2) {
            gP = GoldPosition.MIDDLE;
        } else {
            gP = GoldPosition.RIGHT;
        }

        /** After landing, move a FIRST_MOVE_RIGHT to let the hook get out of the latch */
        driveTrainEnc.moveLeftRightEnc(FIRST_MOVE_RIGHT, 5000);

        angleToTurnAtWall = -135.0;
        runtime.reset();

        if (gP != GoldPosition.UNKNOWN) {
            switch (gP) {
                case MIDDLE:
                    driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                    driveTrainEnc.moveLeftRightEnc(-FIRST_MOVE_RIGHT, 5000);
                    driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);
                    driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                    DISTANCE_TO_WALL = INITIAL_DIST_TO_WALL;
                    driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL, 8000);
                    angleToTurnAtWall = -135.0;
                case RIGHT:
                    driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                    driveTrainEnc.moveLeftRightEnc(DIST_BTWN_MINERALS-FIRST_MOVE_RIGHT, 5000);
                    //driveTrainEnc.spinEnc(45.0, 5000);
                    driveTrainEnc.moveForthBackEnc(PUSH_GOLD , 5000);
                    driveTrainEnc.moveForthBackEnc(-PULL_BACK , 5000);
                    //driveTrainEnc.spinEnc(-45.0, 5000);
                    DISTANCE_TO_WALL = INITIAL_DIST_TO_WALL + DIST_BTWN_MINERALS;
                    driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL, 8000);
                    angleToTurnAtWall = -135.0;
                case LEFT:
                    driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
                    driveTrainEnc.moveLeftRightEnc(-(DIST_BTWN_MINERALS + FIRST_MOVE_RIGHT), 5000);
                    //driveTrainEnc.spinEnc(-45.0, 5000);
                    driveTrainEnc.moveForthBackEnc(PUSH_GOLD , 5000);
                    driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                    DISTANCE_TO_WALL = INITIAL_DIST_TO_WALL - DIST_BTWN_MINERALS;
                    driveTrainEnc.moveForthBackEnc(DISTANCE_TO_WALL, 8000);
                    angleToTurnAtWall = -135.0;
            }
        }
        else {
            driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
            driveTrainEnc.moveLeftRightEnc(-FIRST_MOVE_RIGHT + CAMERA_LEFT_DISPLACEMENT, 5000);

            /* Detect if the middle one is a gold mineral*/
            runtime.reset();
            turnLightOn();
            mR.update();
            while ((mR.getNumM() < 1) && (runtime.milliseconds() < MINERAL_DETECT_TIMELIMIT)) {
                mR.update();
            }
            turnLightOff();

            /* If the GOLD is at the CENTER ... */
            if (mR.goldIsFound()) { //gold is at the CENTER
                driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);
                driveTrainEnc.moveForthBackEnc(-PULL_BACK, 5000);
                driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL, 8000);

                angleToTurnAtWall = -135.0;
            } else {
                /* If not at the center, then try the RIGHT:
                 * turn 45 degrees right, and detect ...
                 * */

                driveTrainEnc.spinEnc(45.0, 5000);

                runtime.reset();
                turnLightOn();
                mR.update();
                while ((mR.getNumM() < 1) && (runtime.milliseconds() < MINERAL_DETECT_TIMELIMIT)) {
                    mR.update();
                }
                turnLightOff();

                /* If the GOLD on the RIGHT ... */
                if (mR.goldIsFound()) { //gold is on the RIGHT
                    driveTrainEnc.moveForthBackEnc(PUSH_GOLD * 1.4, 5000);
                    driveTrainEnc.moveForthBackEnc(-PULL_BACK * 1.4, 5000);
                    driveTrainEnc.spinEnc(-45.0, 5000);
                    driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL, 8000);
                    angleToTurnAtWall = -135.0;
                } else {
                    /* If the GOLD in not on the right, then it must be on the LEFT,
                     * not detection needed
                     **/
                    driveTrainEnc.spinEnc(-90.0, 5000);
                    driveTrainEnc.moveForthBackEnc(PUSH_GOLD * 1.4, 5000);
                    driveTrainEnc.moveLeftRightEnc(-PUSH_GOLD, 5000);
                    driveTrainEnc.spinEnc(-45.0, 5000);
                    driveTrainEnc.moveForthBackEnc(DISTANCE_TO_WALL - PUSH_GOLD, 8000);
                    angleToTurnAtWall = -45.0;
                }
            }
        }

        driveTrainEnc.spinEnc( angleToTurnAtWall, 5000);
        driveTrainEnc.moveLeftRightEnc(DIST_TO_ADJUST_BY_WALL,3000);
        driveTrainEnc.moveForthBackEnc(WALL_TO_DEPOT, 5000);

        /* Stretch out and put down the forearm to put the team marker into the depot zone */
        foreArm.moveForwardEnc(MAX_COUNTS_FOREARM_FORWARD,5000);
        foreArm.moveBackwardEnc(COUNTS_FOREARM_BACKWARD,5000);
        foreArm.moveUpDownAngleEnc(-ANGLE_AUTO_UPDOWN, 5000);

        mineralCollector.dropTeamMarker();
        /* Wipe out to set the team marker on to the depot */
        mineralCollector.wipeOut();
//        sleep(500); // wait 1 second for the team marker to be wiped out to the depot
//        /* Stop the mineral collector's rotation (wiping out) */
//        mineralCollector.wipeStop();
        foreArm.moveUpDownAngleEnc(ANGLE_AUTO_UPDOWN, 5000);
        mineralCollector.openHolder();
        mineralCollector.wipeOut();
        /* Move back to crater */
        driveTrainEnc.moveForthBackEnc(-DEPOT_TO_CRATER, 6000);

        /* Turn back */
        driveTrainEnc.spinEnc(-180, 5000);

        /* Put the foreArm on to the crater, to earn the parking points */
        foreArm.moveUpDownAngleEnc(-ANGLE_AUTO_UPDOWN,5000);

        /////////////////////////////  End of Autonomous Actions  /////////////////////////

        robotLoc.deactivate();
        tfod.deactivate();
    }

    /////////////////////////// Start of Shared functions ///////////////////////////


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

    private void turnLightOn() {
        try {
            mCameraManager.setTorchMode("0", true);
        } catch (Exception e) {}
    }

    private void turnLightOff() {
        try {
            mCameraManager.setTorchMode("0", false);
        } catch (Exception e) {}
    }

    // The variables: oldTime, newTime, deltaTime, checkPoint, timeRecorderList should be initialized in
    // the beginning of the OpMode
//    private void setTimeRecordCheckPoint(String stepDesc) {
//        newTime = timeCheckPoint.milliseconds();
//        if ( checkPoint == 0) {
//            deltaTime = 0;
//        } else {
//            deltaTime = (newTime - oldTime) / 1000;
//        }
//        tR.updateWith(checkPoint,stepDesc,newTime,deltaTime);
//        timeRecorderList.add(tR);
//        oldTime = newTime;
//        checkPoint ++;
//    }
//
//    class TimeRecorder {
//        private int stepNo;
//        private String stepDesc;
//        private double timeStampMS;
//        private double timeDeltaS;
//
//        public void setStepNo(int stepNo) {
//            this.stepNo = stepNo;
//        }
//
//        public void setStepDesc(String stepDesc) {
//            this.stepDesc = stepDesc;
//        }
//
//        public void setTimeStampMS(double timeStampMS) {
//            this.timeStampMS = timeStampMS;
//        }
//
//        public void setTimeDeltaS(double timeDeltaS) {
//            this.timeDeltaS = timeDeltaS;
//        }
//
//        public int getStepNo() {
//            return stepNo;
//        }
//
//        public String getStepDesc() {
//            return stepDesc;
//        }
//
//        public double getTimeStampMS() {
//            return timeStampMS;
//        }
//
//        public double getTimeDeltaS() {
//            return timeDeltaS;
//        }
//
//        public void updateWith(int stepNo, String stepDesc, double timeStampMS, double timeDeltaS) {
//            this.stepNo = stepNo;
//            this.stepDesc = stepDesc;
//            this.timeStampMS = timeStampMS;
//            this.timeDeltaS = timeDeltaS;
//        }
//
//        public void copyFrom(TimeRecorder timeRecorder) {
//            this.stepNo = timeRecorder.stepNo;
//            this.stepDesc = timeRecorder.stepDesc;
//            this.timeStampMS = timeRecorder.timeStampMS;
//            this.timeDeltaS = timeRecorder.timeDeltaS;
//        }
//
//        @Override public String toString() {
//            return  String.format("Step %02d: [ %15.3f ] : [ %10.3f ] : %s ",stepNo, timeStampMS, timeDeltaS, stepDesc);
//        }

//    }





    /////////////////////////// End of Shared functions ///////////////////////////


    ///////////////////////////////////////////////////////
//    public void landFromLatch() {
//
//
////        TelemetryWrapper.setLine(1,"Landing from the latch...");
////        liftArm.moveUpDownEnc(AUTO_LIFT_DOWN_COUNTS, 2000);
////        driveTrainEnc.moveLeftRightEnc(40,2000); // move 40mm right to come out of the hook
////        liftArm.stop();
////        driveTrainEnc.stop();
//    }
//
//    /**
//     * Adjust the robot position:
//     *   - to turn CW/CCW to make the hAlignSlope (horizontal alignment line slope) close to 0
//     *   - to translate left/right to make the first gold angle close to 0
//     *   - so the robot is right facing the gold mineral
//     */
//    public void adjustInitialPosition() {
//
//        //TelemetryWrapper.setLine(1,"landFromLatch...");
//        runtime.reset();
//        double maxLRMovingDist = 200.0; //millimeters
//        double increamentalDist = 50.0;
//        while ( runtime.milliseconds() < 5000 ) {
//            int loops0 = 0;
//            while ((loops0 < 10) && ( mR.getNumM() == 0 )) {
//                mR.update();
//                loops0 ++;
//            }
//            if (mR.getNumM() <= 1) {
//                int loops = 0;
//                while ( mR.getNumM() <= 1 )
//                driveTrainEnc.moveLeftRightEnc(increamentalDist, 2000);
//                continue;
//            } else if ( mR.getHAlignSlope() > 2.0 ) {
//                driveTrainEnc.spinEnc(Math.atan(mR.getHAlignSlope()),2000);
//                continue;
//            } else if (! mR.goldIsFound()) {
//
//                driveTrainEnc.moveLeftRightEnc(increamentalDist,2000);
//                continue;
//            } else if ( mR.getFirstGoldAngle() > 1.5 ) {
//                driveTrainEnc.spinEnc(mR.getFirstGoldAngle(),2000);
//                continue;
//            }
//        }
//        driveTrainEnc.stop();
//    }
//
//    /**
//     * To detect the Gold mineral sample and  identify the position
//     *
//     * @return
//     * GoldPosition.LEFT - Gold is on the left
//     * GoldPosition.MIDDLE - in the middle
//     * GoldPosition.RIGHT - on the right
//     * GoldPosition.UNKNOWN - no mineral detected
//     *
//     */
//    public GoldPosition identifyGoldPosition()
//    {
//        //TelemetryWrapper.setLine(1,"Identifying the gold position ...");
//        //mr.update();
//        return mR.getGoldPosition();
//    }
//
//    public void placeTeamMarker() {
//        TelemetryWrapper.setLine(1,"Placing the team maker...");
//    }
//
//    /**
//     * Initialize the Vuforia localization engine.
//     */
//    private void initVuforia() {
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = CameraDirection.BACK;
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//    }
//
//    /**
//     * Initialize the Tensor Flow Object Detection engine.
//     */
//    private void initTfod() {
//
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId",
//                "id",
//                hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minimumConfidence = TFOD_CONFIDENCE;  //Added by J.Tu on 2019-04-24 00:23
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
//
//    }

}
