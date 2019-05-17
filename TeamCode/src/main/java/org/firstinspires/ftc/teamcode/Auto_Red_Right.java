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
import static org.firstinspires.ftc.teamcode.Parameters.COUNTS_FOREARM_BACKWARD;
import static org.firstinspires.ftc.teamcode.Parameters.DIST_BTWN_MINERALS;
import static org.firstinspires.ftc.teamcode.Parameters.FIRST_MOVE_RIGHT;
import static org.firstinspires.ftc.teamcode.Parameters.INITIAL_MOVE_TO_MINERAL;
import static org.firstinspires.ftc.teamcode.Parameters.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.teamcode.Parameters.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.teamcode.Parameters.MAX_COUNTS_FOREARM_FORWARD;
import static org.firstinspires.ftc.teamcode.Parameters.MINERAL_DETECT_TIMELIMIT;
import static org.firstinspires.ftc.teamcode.Parameters.TFOD_MODEL_ASSET;
import static org.firstinspires.ftc.teamcode.Parameters.VUFORIA_KEY;


@Autonomous(name="Auto_Red_Right",group = "Basis2656_2019")
//@Disabled

public class Auto_Red_Right extends LinearOpMode {

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

    //private final double blue_pRx = -319, blue_pRy = 1036, blue_pMx = -600, blue_pMy = 600, blue_pLx = -1036, blue_pLy = 319;
    private final double red_pRx = 319, red_pRy = -1036, red_pMx = 600, red_pMy = -600, red_pLx = 1036, red_pLy = -319;


    @Override
    public void runOpMode() {

        /***************************** Start of Initializations ****************************/

        TelemetryWrapper.init(telemetry,200);
        Parameters.init(config);
        Hardware2019.init(hardwareMap);

        driveTrainEnc.initEnc();

        foreArm.initEnc();
        mineralCollector.init();
        liftArm.initEnc();

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

        /** End of Initialization *********************************************************/

        liftArm.graspOn();

        waitForStart();



        /////////////////////////////  Start of Autonomous Actions  /////////////////////////

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
            //Log
            TelemetryWrapper.setLine(logCount++, String.format("[Gold Not Detected] goldPosition UNKNOWN, set as default to MIDDLE."));
            //
            goldPostion = GoldPosition.MIDDLE;
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


        if (robotLoc.targetIsVisible()) {

            //Log
            TelemetryWrapper.setLine(logCount++,"RobotLocator targetIsVisible! -- Using calculated path.");
            //
            double x1, y1, alpha, x2, y2, beta, distToMove1, angleToTurn1, distToMove2, angleToTurn2, angleAtWall;

            x1 = robotLoc.getLocX();
            y1 = robotLoc.getLocY();
            alpha = robotLoc.getAngle3();
            switch (goldPostion) {
                case RIGHT:
                    x2 = red_pRx;
                    y2 = red_pRy;
                    break;
                case MIDDLE:
                    x2 = red_pMx;
                    y2 = red_pMy;
                    break;
                case LEFT:
                    x2 = red_pLx;
                    y2 = red_pLy;
                    break;
                default:
                    //Log
                    TelemetryWrapper.setLine(logCount++, String.format("[prepare path] goldPosition UNKNOWN, set as default to MIDDLE."));
                    //
                    goldPostion = GoldPosition.MIDDLE;
                    x2 = red_pMx;
                    y2 = red_pMy;
            }

            distToMove1 = hypot(y2 - y1, x2 - x1);
            beta = toDegrees(atan2(y2 - y1, x2 - x1));

            angleToTurn1 = beta - alpha;
            //find the shorter turning direction
            if (angleToTurn1 < -180) {
                angleToTurn1 += 360;
            } else if (angleToTurn1 > 180) {
                angleToTurn1 -= 360;
            }
            //reverse the target direction
            if (angleToTurn1 > 0) {
                angleToTurn1 -= 180;
            } else if (angleToTurn1 < 0) {
                angleToTurn1 += 180;
            }
            //Log
            TelemetryWrapper.setLine(logCount++,String.format("(x1, y1)=(%.1f, %.1f), (x2, y2)=(%.1f, %.1f)",x1, y1,x2, y2));
            //
            driveTrainEnc.spinEnc(-angleToTurn1, 5000);
            driveTrainEnc.moveForthBackEnc(-distToMove1, 7000);

            // Second move
            alpha = alpha + angleToTurn1;
            if (alpha > 180) {
                alpha = alpha - 360;
            } else if ( alpha < -180 ) {
                alpha = alpha + 360;
            }
            x1 = x2;
            y1 = y2;
            x2 = 1480;
            y2 = -1480;
            distToMove2 = hypot(y2 - y1, x2 - x1);
            beta = toDegrees(atan2(y2 - y1, x2 - x1));

            angleToTurn2 = beta - alpha;
            //find the shorter turning direction
            if (angleToTurn2 < -180) {
                angleToTurn2 += 360;
            } else if (angleToTurn2 > 180) {
                angleToTurn2 -= 360;
            }
            //reverse the target direction
            if (angleToTurn2 > 0) {
                angleToTurn2 -= 180;
            } else if (angleToTurn2 < 0) {
                angleToTurn2 += 180;
            }

            driveTrainEnc.spinEnc(-angleToTurn2, 5000);
            driveTrainEnc.moveForthBackEnc(-distToMove2, 7000);
            tmController.pushOff();

            angleAtWall = beta - 90;
            driveTrainEnc.spinEnc(angleAtWall,5000);
            driveTrainEnc.moveForthBackEnc(1780,7000);

        } else {
            //Log
            TelemetryWrapper.setLine(logCount++,"RobotLocator NOT targetIsVisible! - Using pre-configured path");
            //

            switch (goldPostion) {
                case RIGHT:
                    driveTrainEnc.spinEnc(90,5000);
                    driveTrainEnc.moveLeftRightEnc(-(DIST_BTWN_MINERALS - FIRST_MOVE_RIGHT),5000);
                    driveTrainEnc.moveForthBackEnc(-hypot(600,600),6000);
                    driveTrainEnc.spinEnc(-45,5000);
                    driveTrainEnc.moveForthBackEnc(-600,5000);
                    tmController.pushOff();
                    driveTrainEnc.spinEnc(90,5000);
                    driveTrainEnc.moveForthBackEnc(1780,7000);
                    break;
                case MIDDLE:
                    driveTrainEnc.spinEnc(90,5000);
                    driveTrainEnc.moveLeftRightEnc(FIRST_MOVE_RIGHT,5000);
                    driveTrainEnc.moveForthBackEnc(-hypot(900,900),6000);
                    tmController.pushOff();
                    driveTrainEnc.spinEnc(45,5000);
                    driveTrainEnc.moveForthBackEnc(1780,7000);
                    break;
                case LEFT:
                    driveTrainEnc.spinEnc(90,5000);
                    driveTrainEnc.moveLeftRightEnc(DIST_BTWN_MINERALS + FIRST_MOVE_RIGHT,5000);
                    driveTrainEnc.moveForthBackEnc(-hypot(600,600),6000);
                    tmController.pushOff();
                    driveTrainEnc.spinEnc(45,5000);
                    driveTrainEnc.moveForthBackEnc(-600,5000);
                    driveTrainEnc.moveForthBackEnc(1780,7000);
            }

        }

        foreArm.moveForwardEnc(MAX_COUNTS_FOREARM_FORWARD,5000);
        foreArm.moveForwardEnc( - COUNTS_FOREARM_BACKWARD,5000);
        foreArm.moveUpDownAngleEnc(-ANGLE_AUTO_UPDOWN,5000);



        robotLoc.deactivate();
        tfod.deactivate();
        driveTrainEnc.stopEnc();
        tmController.stayOn();

    }

    /////////////////////////// Start of Shared functions ///////////////////////////

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

    /////////////////////////// End of Shared functions ///////////////////////////

}
