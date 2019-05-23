package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.camera2.CameraManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import java.util.ArrayList;
import java.util.List;

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
import static org.firstinspires.ftc.teamcode.Parameters.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.teamcode.Parameters.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.teamcode.Parameters.LIFT_AUTO_LANDING_COUNTS;
import static org.firstinspires.ftc.teamcode.Parameters.MAX_COUNTS_FOREARM_FORWARD;
import static org.firstinspires.ftc.teamcode.Parameters.MINERAL_DETECT_TIMELIMIT;
import static org.firstinspires.ftc.teamcode.Parameters.PULL_BACK;
import static org.firstinspires.ftc.teamcode.Parameters.PUSH_GOLD;
//import static org.firstinspires.ftc.teamcode.Parameters.RIGHT_GOLD_DISTANCE;
import static org.firstinspires.ftc.teamcode.Parameters.TFOD_MODEL_ASSET;
import static org.firstinspires.ftc.teamcode.Parameters.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.Parameters.WALL_TO_DEPOT;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.a;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.back;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_down;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_left;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_right;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_up;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.left_bumper;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.right_bumper;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.start;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.x;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.y;

@TeleOp(name = "Calibration", group = "Test")
//@Disabled
public class Calibration extends LinearOpMode {

    /*** Define your variables here ********************************************************/
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

//    private GoldPosition        gP = GoldPosition.UNKNOWN;

    private DriveTrain          driveTrain = new DriveTrain();
    private DriveTrainByEncoder driveTrainEnc = new DriveTrainByEncoder();
    private ForeArm             foreArm = new ForeArm();
    private LiftArm             liftArm = new LiftArm();
    private MineralCollector    mineralCollector = new MineralCollector();
    private TeamMarkerController tmCtrl = new TeamMarkerController();

    //private static final String VUFORIA_KEY = "AaVQPxH/////AAABmWbgMV3r8kMuucDJZwS+C8IqcKbjimK6x7yZkfsYnCLGA1cHVqGOF+tSmO//7vH+NwYrxmEfltB1UGzWki397Ksrl57wPSMPbGU2y9Cg+iSgHMGpJVx4IDeD6ldnTIRetHFeW0r4OzmfsDc5eI0tChOd2FYv2Q8MuHq/QXlsdOHEOyy43xqj5QF4eRSVznttm6fDzN2egZWEIr8Un9B0hCEv6OmQATKUsEPx7BnqCxjBK00252+n2Na17OxE2hYP8WXUerdZOOU1GyWFPOG2DDeYDWiipgYGXgpIC+a846STiSZcFXLP2S3ENu78EoCFKs7Fw7sm5u58dzZ5PyMg8VUormyNmcHm9RU2Fl5364WO";
//    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
//    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
//    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

//    private VuforiaLocalizer    vuforia;
//    private TFObjectDetector    tfod;
//
//    private GoldDetector gD;
//    private RobotLocator        robotLoc;

    /*** End of Definition ****************************************************************/

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
//
//        liftArm.initEnc();
//        foreArm.initEnc();
//        mineralCollector.init();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        initVuforia(cameraMonitorViewId);
//        initTfod();
//        tfod.activate();

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


//            if (bH1.pressing(x)) {
//                tmCtrl.setPostion(0.0);
//            }
//            if (bH1.pressing(y)) {
//                tmCtrl.setPostion(1.0);
//            }

            if (bH1.pressing(dpad_up)) {

                driveTrainEnc.initEnc();
                driveTrainEnc.moveForthBackEnc(600., 5000);

            }

            if (bH1.pressing(dpad_down)) {
                driveTrainEnc.initEnc();
                driveTrainEnc.moveForthBackEnc(-600, 5000);

            }

            if (bH1.pressing(dpad_left)) {

                driveTrainEnc.initEnc();
                driveTrainEnc.moveLeftRightEnc(-600., 5000);
            }

            if (bH1.pressing(dpad_right)) {

                driveTrainEnc.initEnc();
                driveTrainEnc.moveLeftRightEnc(-600, 5000);
            }

            if (bH1.pressing(left_bumper)) {

                driveTrainEnc.initEnc();
                driveTrainEnc.spinEnc(-90,5000);
            }

            if (bH1.pressing(right_bumper)) {

                driveTrainEnc.initEnc();
                driveTrainEnc.spinEnc(90,5000);
            }




            /////////////////// Choose steps to test ///////////////////////
            if (bH1.pressing(start)) {
                driveTrainEnc.initEnc();
                if (stepEnabled[0]) {

                }
                if (stepEnabled[1]) {

                }
                if (stepEnabled[2]) {

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
                    driveTrainEnc.moveForthBackEnc(600., 5000);
                }
                if (stepEnabled[17]) { //Backwards
//                    TelemetryWrapper.setLine(7,String.format("MOVE_BACKWARDS = %.1f ", -1300.));
                    TelemetryWrapper.setLine(10, String.format("MOVE_BACKWARDS = %.1f ; %8.1f Step 3: Moving backwards...", -1300., runtime.milliseconds()));
                    driveTrainEnc.moveForthBackEnc(-600., 5000);
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
//        tfod.deactivate();
    }

//    private void initVuforia(int cameraMonitorViewId) {
//        /** For Vuforia engine 1 (for Android Camera) */
//        VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters1.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters1.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
//        //  Instantiate the Vuforia engine 1
//        vuforia = ClassFactory.getInstance().createVuforia(parameters1);
//    }
//
//    private void initVuforia() {
//
//        /** For Vuforia engine 1 (for Android Camera) */
//        //VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters();
//        parameters1.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters1.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
//        //  Instantiate the Vuforia engine 1
//        vuforia = ClassFactory.getInstance().createVuforia(parameters1);
//    }
//
//    /**
//     * Initialize the Tensor Flow Object Detection engine.
//     */
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        //tfodParameters.minimumConfidence = 0.4; // Added by J.Tu
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
//    }
//
//    private void turnLightOn() {
//        try {
//            mCameraManager.setTorchMode("0", true);
//        } catch (Exception e) {}
//    }
//    private void turnLightOff() {
//        try {
//            mCameraManager.setTorchMode("0", false);
//        } catch (Exception e) {}
//    }
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
