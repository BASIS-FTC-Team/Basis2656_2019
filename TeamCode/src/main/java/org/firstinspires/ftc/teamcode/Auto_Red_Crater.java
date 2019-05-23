package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;
import static org.firstinspires.ftc.teamcode.Parameters.ANGLE_AUTO_UPDOWN;
import static org.firstinspires.ftc.teamcode.Parameters.BLUE_WALL_TARGET_X;
import static org.firstinspires.ftc.teamcode.Parameters.BLUE_WALL_TARGET_Y;
import static org.firstinspires.ftc.teamcode.Parameters.COUNTS_FOREARM_BACKWARD;
import static org.firstinspires.ftc.teamcode.Parameters.DIST_BTWN_MINERALS;
import static org.firstinspires.ftc.teamcode.Parameters.FIRST_MOVE_RIGHT;
import static org.firstinspires.ftc.teamcode.Parameters.INITIAL_DIST_TO_WALL;
import static org.firstinspires.ftc.teamcode.Parameters.INITIAL_MOVE_TO_MINERAL;
import static org.firstinspires.ftc.teamcode.Parameters.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.teamcode.Parameters.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.teamcode.Parameters.MAX_COUNTS_FOREARM_FORWARD;
import static org.firstinspires.ftc.teamcode.Parameters.MINERAL_DETECT_TIMELIMIT;
import static org.firstinspires.ftc.teamcode.Parameters.PUSH_GOLD;
import static org.firstinspires.ftc.teamcode.Parameters.RED_WALL_TARGET_X;
import static org.firstinspires.ftc.teamcode.Parameters.RED_WALL_TARGET_Y;
import static org.firstinspires.ftc.teamcode.Parameters.TFOD_MODEL_ASSET;
import static org.firstinspires.ftc.teamcode.Parameters.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.Parameters.WALL_TO_DEPOT;


@Autonomous(name="Auto_Red_Crater",group = "Basis2656_2019")
//@Disabled

public class Auto_Red_Crater extends LinearOpMode {

    /**
     * Notice for programmer:
     * wallTarget_X and wallTarget_Y are set to be different values for different autoCase
     * Just uncomment the correct case to set the correct values.
     * */

    /* For AutoCase.BLUE_LEFT */
//    private double wallTarget_X = BLUE_WALL_TARGET_X;
//    private double wallTarget_Y = BLUE_WALL_TARGET_Y;

    /* For AutoCase.RED_LEFT */
    private double wallTarget_X = RED_WALL_TARGET_X;
    private double wallTarget_Y = RED_WALL_TARGET_Y;



    /*** Define your variables here ********************************************************/
    private ElapsedTime         runtimeMain = new ElapsedTime();
    private Config              config = new Config(Config.configFile);
    private int                 logCount = 0;
    private int                 loop_times = 0;

    private DriveTrainByEncoder driveTrainEnc = new DriveTrainByEncoder();
    private ForeArm             foreArm = new ForeArm();
    private LiftArm             liftArm = new LiftArm();
    private MineralCollector    mineralCollector = new MineralCollector();
    private TeamMarkerController tmController;

    private VuforiaLocalizer    vuforia;
    private TFObjectDetector    tfod;
    private GoldDetector        goldDetector;
    private GoldPosition        goldPostion = GoldPosition.UNKNOWN;
    private RobotLocator        robotLoc;


    @Override
    public void runOpMode() {

        /***************************** Start of Initializations ****************************/

        //Log initialization
        TelemetryWrapper.init(telemetry,200);
        //

        Parameters.init(config);
        Hardware2019.init(hardwareMap);

        driveTrainEnc.initEnc();

        foreArm.initEnc();
        mineralCollector.init();
        liftArm.initEnc();
        liftArm.graspOn();

        tmController = new TeamMarkerController();
        tmController.stayOn();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        initVuforia(cameraMonitorViewId);
        initTfod();
        tfod.activate();
        goldDetector = new GoldDetector();

        robotLoc = new RobotLocator();
        robotLoc.initialize(vuforia);
        robotLoc.activate();
        robotLoc.update();

        /***************************** End of Initialization ******************************/



        waitForStart();

        /*****************************   Start of Autonomous Actions  ******************************/

        /** Landing off */
        liftArm.landOffEnc(4000);

        /** After landing, move a FIRST_MOVE_RIGHT to let the hook get out of the latch */
        driveTrainEnc.moveLeftRightEnc(FIRST_MOVE_RIGHT, 5000);

        goldDetector.update(tfod.getUpdatedRecognitions());
        goldPostion = goldDetector.estimateGoldPostion();

        loop_times = 0;
        runtimeMain.reset();

        //Log
        TelemetryWrapper.setLine(logCount++,String.format("[B4loop]%f: (numM, numG, numS)=(%d, %d, %d) \n gP = %s",
                runtimeMain.milliseconds(),goldDetector.getNumM(), goldDetector.getNumG(), goldDetector.getNumS(), goldDetector.estimateGoldPostion().toString()));
        //


        while (( goldPostion.equals(GoldPosition.UNKNOWN) ) &&  (runtimeMain.milliseconds() < MINERAL_DETECT_TIMELIMIT) ) {

            goldDetector.update(tfod.getRecognitions());
            goldPostion = goldDetector.estimateGoldPostion();
            //Log
            TelemetryWrapper.setLine(logCount++, String.format("[INloop] gD Loops: %d; %s", ++loop_times, goldPostion.toString()));
            //
        }

        //Log
        TelemetryWrapper.setLine(logCount++, String.format("[AFTloop]%f: (numM, numG, numS)=(%d, %d, %d) \n gP = %s",
                runtimeMain.milliseconds(),goldDetector.getNumM(), goldDetector.getNumG(), goldDetector.getNumS(), goldDetector.estimateGoldPostion().toString()));
        //

        if (goldPostion.equals(GoldPosition.UNKNOWN) ) {
            goldPostion = GoldPosition.LEFT;
        }

        //Log
        TelemetryWrapper.setLine(logCount++,String.format("%f: finally gP = %s",
                runtimeMain.milliseconds(), goldDetector.estimateGoldPostion().toString()));
        //

        driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL, 5000);
        driveTrainEnc.spinEnc(90, 5000);

        loop_times = 0;
        //Log
        TelemetryWrapper.setLine(logCount++, String.format("[B4loop]rLoc Loops: %d; (x, y)=(%.1f, %.1f), angle3=%.1f",
                ++loop_times, robotLoc.getLocX(),robotLoc.getLocY(),robotLoc.getAngle3()));
        //
        runtimeMain.reset();
        while ((!robotLoc.targetIsVisible()) && (runtimeMain.milliseconds()<1000) ) {

            robotLoc.update();
            //Log
            TelemetryWrapper.setLine(logCount++, String.format("[INloop]rLoc Loops: %d; (x, y)=(%.1f, %.1f), angle3=%.1f",
                    ++loop_times, robotLoc.getLocX(),robotLoc.getLocY(),robotLoc.getAngle3()));
            //
        }

        //Log
        TelemetryWrapper.setLine(logCount++, String.format("[INloop]rLoc Loops: %d; (x, y)=(%.1f, %.1f), angle3=%.1f",
                ++loop_times, robotLoc.getLocX(),robotLoc.getLocY(),robotLoc.getAngle3()));
        //

        double angleAtWall = 0;
        if (robotLoc.targetIsVisible()) {

            //Log
            TelemetryWrapper.setLine(logCount++,"RobotLocator targetIsVisible! -- Using calculated path.");
            //
            double x1, y1, a1, x2, y2;

            x1 = robotLoc.getLocX();
            y1 = robotLoc.getLocY();
            a1 = robotLoc.getAngle3();
            x2 = wallTarget_X;
            y2 = wallTarget_Y;

            //Log
            TelemetryWrapper.setLine(logCount++,String.format("(x1, y1)=(%.1f, %.1f), (x2, y2)=(%.1f, %.1f)",x1, y1,x2, y2));
            //

            double distToMove = hypot(y2 - y1, x2 - x1);
            double theAngle = toDegrees(atan2(y2 - y1, x2 - x1));
            double angleToTurn = theAngle - a1 - 180;

            driveTrainEnc.spinEnc(-angleToTurn, 5000);
            driveTrainEnc.moveForthBackEnc(-distToMove, 7000);
            angleAtWall = theAngle - 180;

        } else {
            //Log
            TelemetryWrapper.setLine(logCount++,"RobotLocator NOT targetIsVisible! - Using pre-configured path");
            //

            driveTrainEnc.moveForthBackEnc(-(INITIAL_DIST_TO_WALL + FIRST_MOVE_RIGHT + 20.0), 8000);
            angleAtWall = -45.0;
        }
        driveTrainEnc.spinEnc(angleAtWall, 5000);
        driveTrainEnc.moveForthBackEnc(-WALL_TO_DEPOT, 5000);
        tmController.pushOff();
        driveTrainEnc.moveForthBackEnc( WALL_TO_DEPOT, 5000);
        driveTrainEnc.spinEnc(45,5000);

        switch (goldPostion) {
            case UNKNOWN:
                driveTrainEnc.moveForthBackEnc( INITIAL_DIST_TO_WALL - DIST_BTWN_MINERALS - 100, 7000);
                driveTrainEnc.spinEnc(-70, 5000);
                break;
            case LEFT:
                driveTrainEnc.moveForthBackEnc( INITIAL_DIST_TO_WALL - DIST_BTWN_MINERALS -100, 7000);
                driveTrainEnc.spinEnc(-70, 5000);
                break;
            case MIDDLE:
                driveTrainEnc.moveForthBackEnc( INITIAL_DIST_TO_WALL , 7000);
                driveTrainEnc.spinEnc(-90, 5000);
                break;
            case RIGHT:
                driveTrainEnc.moveForthBackEnc( INITIAL_DIST_TO_WALL + DIST_BTWN_MINERALS + 100  , 7000);
                driveTrainEnc.spinEnc(-110, 5000);
                break;
        }

        driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 5000);

        foreArm.moveUpDownAngleEnc(-ANGLE_AUTO_UPDOWN,5000);

        foreArm.moveForwardEnc(MAX_COUNTS_FOREARM_FORWARD,5000);

        mineralCollector.openHolder();

        foreArm.moveForwardEnc( - COUNTS_FOREARM_BACKWARD,5000);



        robotLoc.deactivate();
        tfod.deactivate();
        driveTrainEnc.stopEnc();
        tmController.stayOn();

    }
    /*******************************   End of Autonomous Actions  **********************************/



    /************************************ Start of Shared functions ********************************/

    private void initVuforia(int cameraMonitorViewId) {
        /** For Vuforia engine 1 (for Android Camera) */
        VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters1.vuforiaLicenseKey = VUFORIA_KEY;
        parameters1.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters1);
    }

    private void initVuforia() {

        /** For Vuforia engine 1 (for Android Camera) */
        //VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters();
        parameters1.vuforiaLicenseKey = VUFORIA_KEY;
        parameters1.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters1);
    }

    private void initVuforiaWithWebcam() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initVuforiaWithWebcam(int cameraMonitorViewId) {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /************************************ End of Shared functions ****************************************/

    public enum AutoCase {
        BLUE_LEFT, BLUE_RIGHT, RED_LEFT, RED_RIGHT;
    }

}
