package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;


@Autonomous(name="AutoCase01_Basis2656_2019_02",group = "Basis2656_2019")

public class AutoCase01_02 extends LinearOpMode {

    private static final String VUFORIA_KEY = "AaVQPxH/////AAABmWbgMV3r8kMuucDJZwS+C8IqcKbjimK6x7yZkfsYnCLGA1cHVqGOF+tSmO//7vH+NwYrxmEfltB1UGzWki397Ksrl57wPSMPbGU2y9Cg+iSgHMGpJVx4IDeD6ldnTIRetHFeW0r4OzmfsDc5eI0tChOd2FYv2Q8MuHq/QXlsdOHEOyy43xqj5QF4eRSVznttm6fDzN2egZWEIr8Un9B0hCEv6OmQATKUsEPx7BnqCxjBK00252+n2Na17OxE2hYP8WXUerdZOOU1GyWFPOG2DDeYDWiipgYGXgpIC+a846STiSZcFXLP2S3ENu78EoCFKs7Fw7sm5u58dzZ5PyMg8VUormyNmcHm9RU2Fl5364WO";

    /** Vuforia localization engine.  */
    private VuforiaLocalizer vuforia;
    /** Tensor Flow Object Detection engine. */
    private TFObjectDetector tfod;
    /** Mineral Recognizer */
    private MineralRecognizer mR;
    private PID pidForForearmUpDown = null;
    private PID pidForForearmForthBack = null;
    private PID pidForLiftUpDown = null;
    private MineralCollector mineralCollector = new MineralCollector();

    private ElapsedTime runtime = new ElapsedTime();

    private Config config = new Config(Config.configFile);
    private DriveTrainByEncoder driveTrainEnc = new DriveTrainByEncoder();
    private ForeArm foreArm = new ForeArm();
    private LiftArm liftArm = new LiftArm();

    /** Auto drive speed ( actually the power applied to motors, from 0.0 to 1.0 ) */
    private double AUTO_DRIVE_SPEED = 0.5;
//    private double AUTO_LIFT_SPEED = 0.3;
//    private int    AUTO_LIFT_DOWN_COUNTS = 200;

    /** Distances in millimeters to move automatically
     *
     * private static final double TILE_SIDE_LENGHT = 597.0; //23.5 inches * 25.4 mm/in
     * private static final double TILE_DIAGONAL = 844.1; // sqrt((TILE_SIDE_LENGHT ** 2) + (TILE_SIDE_LENGHT ** 2))
     * private static final double HALF_TILE_DIAGONAL = 822.1 // TILE_DIAGONAL / 2
     * private static final double DIST_BTWN_MINERALS = 368.3; // 14.5 inches * 25.4 mm/in
     * private static final double FRONT_WHEEL_CENTER_TO_FRONT_RIM = 80.0;
     * private static final double REAR_WHEEL_CENTER_TO_REAR_RIM = 60.0;
     * private static final double DIST_BTWN_FRONT_REAR_WHEELS = 305.0;
     * private static final double DIST_BTWN_2_FRONT_WHEELS = 350.0;
     *
     *
     */

    private static final double DIST_BTWN_MINERALS = 368.3; // 14.5 inches * 25.4 mm/in
    private double FIRST_MOVE_RIGHT = 40;
    private double INITIAL_MOVE_TO_MINERAL = 382.5;
    private double INITIAL_DIST_TO_WALL = 1275.0;

    //private double MOVE_TO_GOLD = 750;
    private double PUSH_GOLD =  382.5;
    private double PULL_BACK = 382.5;
    private double LEFT_GOLD_DISTANCE = -DIST_BTWN_MINERALS;
    private double RIGHT_GOLD_DISTANCE = DIST_BTWN_MINERALS;
    private double MIDDLE_GOLD_DISTANCE = 0;
    private double DISTANCE_TO_WALL = INITIAL_DIST_TO_WALL;
    private double WALL_TO_DEPOT = 663.0; // 1.5 * 597.0 (tile length) - 80.0 (FRONT_WHEEL_CENTER_TO_FRONT_RIM) - 305.0/2 (half of DIST_BTWN_FRONT_REAR_WHEELS) ;
    private double DEPOT_TO_CRATER = 1326.0; // 597.0 (tile length)  * 3 - (80 + 305.0/2) [center to front rim] * 2;

    private double TFOD_CONFIDENCE = 0.8;  // the default value in tfod class is 0.4
    private GoldPosition goldPosition = GoldPosition.UNKNOWN;


    @Override
    public void runOpMode() {

        pidForForearmUpDown = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);
        pidForForearmForthBack = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);
        pidForLiftUpDown = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);


        /** Initialization  */
        Parameters.init(config);

        driveTrainEnc.init(hardwareMap, config);

        // Encoder will be used for controlling the foreArm
        foreArm.initEnc(hardwareMap, config, pidForForearmUpDown, pidForForearmForthBack);
        mineralCollector.init(hardwareMap, config);

        // Encoder will be used for controlling the liftArm
        liftArm.initEnc(hardwareMap, config, pidForLiftUpDown);

        //gd.init(hardwareMap, config);
        TelemetryWrapper.init(telemetry, 10);

        /** waiting for user to press start */
        waitForStart();

        /** Start timing */
        runtime.reset();
        //TelemetryWrapper.setLine(2, "" + opModeIsActive() + runtime.seconds());

        /** 0.1: Landing from the latch */
        TelemetryWrapper.setLine(1,String.format("%8.1f Step 1: Landing off ...",runtime.milliseconds()));
        liftArm.landOffEnc(10000);

        TelemetryWrapper.setLine(1,String.format("%8.1f Step 2: Moving 40mm right...",runtime.milliseconds()));
        driveTrainEnc.moveLeftRightEnc(FIRST_MOVE_RIGHT,5000);

//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 3: Moving forward 40mm right...",runtime.milliseconds()));
//        driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED,INITIAL_MOVE_TO_MINERAL,5000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 4: Moving left/right to search gold...",runtime.milliseconds()));
//        driveTrainEnc.moveLeftRightEnc(AUTO_DRIVE_SPEED,RIGHT_GOLD_DISTANCE,5000);
//        DISTANCE_TO_WALL = INITIAL_DIST_TO_WALL + RIGHT_GOLD_DISTANCE;
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 5: Moving forwards to push gold...",runtime.milliseconds()));
//        driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED,PUSH_GOLD,5000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 6: Moving back after pushing gold...",runtime.milliseconds()));
//        driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED,-PULL_BACK,5000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 7: Moving left towards the wall...",runtime.milliseconds()));
//        driveTrainEnc.moveLeftRightEnc(AUTO_DRIVE_SPEED,-DISTANCE_TO_WALL,10000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 8: Turning - 135 degrees for depot...",runtime.milliseconds()));
//        driveTrainEnc.spinEnc(AUTO_DRIVE_SPEED,-135,5000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 9: Moving towards depot...",runtime.milliseconds()));
//        driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED,WALL_TO_DEPOT,10000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 10: Putting off the Team Marker...",runtime.milliseconds()));
//        foreArm.moveUpDownAngleEnc(0.5,-30,5000);
//        mineralCollector.wipeOut();
//        foreArm.moveUpDownAngleEnc(0.5,+30,5000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 11: Turning - 180 degrees for crater...",runtime.milliseconds()));
//        driveTrainEnc.spinEnc(AUTO_DRIVE_SPEED,-180,10000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 12: Moving towards depot...",runtime.milliseconds()));
//        driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED,DEPOT_TO_CRATER,10000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 13: Putting off the foreArm to touch the crater...",runtime.milliseconds()));
//        foreArm.moveUpDownAngleEnc(0.5,-30,5000);


//        /** 0.2 Adjust the position and the orientation */
//        //adjustInitialPosition();
//
//        /** 0.3 Detect the Gold position */
//        goldPosition = identifyGoldPosition();
//
//        /** 1: Push or collect the Gold */
//
//        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, MOVE_TO_GOLD + PUSH_GOLD, 0, 3);
//        if(goldPosition.equals(GoldPosition.LEFT)) {
//            //driveTrain.moveLeftRightEnc(AUTO_DRIVE_SPEED, MOVE_TO_GOLD + LEFT_GOLD_DISTANCE, 10000);
//            driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED, 3142, 10000);
//        }
//        else if(goldPosition.equals(GoldPosition.RIGHT)) {
//            //driveTrain.moveLeftRightEnc(AUTO_DRIVE_SPEED, MOVE_TO_GOLD + RIGHT_GOLD_DISTANCE, 10000);
//            driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED, PUSH_GOLD, 10000);
//        }
//        else {
//            driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED, PUSH_GOLD, 10000);
//        }
//
//        /** 2: Back out from pushing  */
//
//        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, -PULL_BACK, 0, 3);
//        driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED,PULL_BACK,5000);
//
//        /** 3: Linear shift to wall side */
//        double distanceToMoveTowardsWall = DISTANCE_TO_WALL;
//        switch (goldPosition) {
//            case LEFT:
//                distanceToMoveTowardsWall = DISTANCE_TO_WALL + LEFT_GOLD_DISTANCE;
//            case MIDDLE:
//                distanceToMoveTowardsWall = DISTANCE_TO_WALL + MIDDLE_GOLD_DISTANCE;
//            case RIGHT:
//                distanceToMoveTowardsWall = DISTANCE_TO_WALL + RIGHT_GOLD_DISTANCE;
//            default:
//                distanceToMoveTowardsWall = DISTANCE_TO_WALL;
//        }
//        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, -distanceToMoveTowardsWall, 0, 0, 5);
//        driveTrainEnc.moveLeftRightEnc(AUTO_DRIVE_SPEED, -distanceToMoveTowardsWall,10000);
//
//        /** 4: Turn towards depot */
//        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, 0, -135, 5);
//        driveTrainEnc.spinEnc(AUTO_DRIVE_SPEED,-135, 10000);
//
//        /** 5: Head towards depot */
//        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, WALL_TO_DEPOT, 0, 5);
//        driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED,WALL_TO_DEPOT,10000);
//
//        /** 6: Place team marker  */
//        placeTeamMarker();
//
//        /** 7: Back to crater */
//        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, DEPOT_TO_CRATER, 0, 5);
//        driveTrainEnc.moveForthBackEnc(AUTO_DRIVE_SPEED,-DEPOT_TO_CRATER,10000);
//
//        /** 8: Turn 180 to crater */
//        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, 0, 180, 5);
//        driveTrainEnc.spinEnc(AUTO_DRIVE_SPEED,180,10000);
//
//        /** 9: Put grabber into crater  */
//        //comment the code to pass by the action temporarily
//        //grabArm.automoveDownEnc(10000,opModeIsActive());
    }

    public void landFromLatch() {


//        TelemetryWrapper.setLine(1,"Landing from the latch...");
//        liftArm.moveUpDownEnc(AUTO_LIFT_SPEED, AUTO_LIFT_DOWN_COUNTS, 2000);
//        driveTrainEnc.moveLeftRightEnc(AUTO_DRIVE_SPEED,40,2000); // move 40mm right to come out of the hook
//        liftArm.stop();
//        driveTrainEnc.stop();
    }

    /**
     * Adjust the robot position:
     *   - to turn CW/CCW to make the hAlignSlope (horizontal alignment line slope) close to 0
     *   - to translate left/right to make the first gold angle close to 0
     *   - so the robot is right facing the gold mineral
     */
    public void adjustInitialPosition() {

        //TelemetryWrapper.setLine(1,"landFromLatch...");
        runtime.reset();
        double maxLRMovingDist = 200.0; //millimeters
        double increamentalDist = 50.0;
        while ( runtime.milliseconds() < 5000 ) {
            int loops0 = 0;
            while ((loops0 < 10) && ( mR.getNumM() == 0 )) {
                mR.update();
                loops0 ++;
            }
            if (mR.getNumM() <= 1) {
                int loops = 0;
                while ( mR.getNumM() <= 1 )
                driveTrainEnc.moveLeftRightEnc(increamentalDist, 2000);
                continue;
            } else if ( mR.getHAlignSlope() > 2.0 ) {
                driveTrainEnc.spinEnc(AUTO_DRIVE_SPEED,Math.atan(mR.getHAlignSlope()),2000);
                continue;
            } else if (! mR.isGoldFound()) {

                driveTrainEnc.moveLeftRightEnc(increamentalDist,2000);
                continue;
            } else if ( mR.getFirstGoldAngle() > 1.5 ) {
                driveTrainEnc.spinEnc(AUTO_DRIVE_SPEED, mR.getFirstGoldAngle(),2000);
                continue;
            }
        }
        driveTrainEnc.stop();
    }

    /**
     * To detect the Gold mineral sample and  identify the position
     *
     * @return
     * GoldPosition.LEFT - Gold is on the left
     * GoldPosition.MIDDLE - in the middle
     * GoldPosition.RIGHT - on the right
     * GoldPosition.UNKNOWN - no mineral detected
     *
     */
    public GoldPosition identifyGoldPosition()
    {
        //TelemetryWrapper.setLine(1,"Identifying the gold position ...");
        //mr.update();
        return mR.getGoldPosition();
    }

    public void placeTeamMarker() {
        TelemetryWrapper.setLine(1,"Placing the team maker...");
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = TFOD_CONFIDENCE;  //Added by J.Tu on 2019-04-24 00:23
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }

}
