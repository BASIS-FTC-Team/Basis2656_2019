package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import static org.firstinspires.ftc.teamcode.Parameters.*;


@Autonomous(name="AutoCase01_03",group = "Basis2656_2019")
//@Disabled

public class AutoCase01_03 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Config config = new Config(Config.configFile);



    /** Vuforia localization engine.  */
    private VuforiaLocalizer vuforia;
    /** Tensor Flow Object Detection engine. */
    private TFObjectDetector tfod;
    /** Mineral Recognizer */
    private MineralRecognizer mR;
    /** Mineral Collector */
    private MineralCollector mineralCollector = new MineralCollector();

    private DriveTrainByEncoder driveTrainEnc = new DriveTrainByEncoder();
    private ForeArm foreArm = new ForeArm();
    private LiftArm liftArm = new LiftArm();

    /** Auto drive speed ( actually the power applied to motors, from 0.0 to 1.0 ) */
    //private double AUTO_DRIVE_SPEED = 0.5;

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

    private GoldPosition goldPosition = GoldPosition.UNKNOWN;

    @Override
    public void runOpMode() {


        /** Initialization  */
        Parameters.init(config);

        driveTrainEnc.init();

        // Encoder will be used for controlling the foreArm
        foreArm.initEnc();
        mineralCollector.init();

        // Encoder will be used for controlling the liftArm
        liftArm.initEnc();

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

//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 3: Moving to initial position...",runtime.milliseconds()));
//        driveTrainEnc.moveForthBackEnc(INITIAL_MOVE_TO_MINERAL,5000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 4: Moving left/right to search gold...",runtime.milliseconds()));
//        driveTrainEnc.moveLeftRightEnc(RIGHT_GOLD_DISTANCE,5000);
//        DISTANCE_TO_WALL = INITIAL_DIST_TO_WALL + RIGHT_GOLD_DISTANCE;
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 5: Moving forwards to push gold...",runtime.milliseconds()));
//        driveTrainEnc.moveForthBackEnc(PUSH_GOLD,5000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 6: Moving back after pushing gold...",runtime.milliseconds()));
//        driveTrainEnc.moveForthBackEnc(-PULL_BACK,5000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 7: Moving left towards the wall...",runtime.milliseconds()));
//        driveTrainEnc.moveLeftRightEnc(-DISTANCE_TO_WALL,10000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 8: Turning - 135 degrees for depot...",runtime.milliseconds()));
//        driveTrainEnc.spinEnc(-135,5000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 9: Moving towards depot...",runtime.milliseconds()));
//        driveTrainEnc.moveForthBackEnc(WALL_TO_DEPOT,10000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 10: Putting off the Team Marker...",runtime.milliseconds()));
//        foreArm.moveUpDownAngleEnc(0.5,-30,5000);
//        mineralCollector.wipeOut();
//        foreArm.moveUpDownAngleEnc(0.5,+30,5000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 11: Turning - 180 degrees for crater...",runtime.milliseconds()));
//        driveTrainEnc.spinEnc(-180,10000);
//
//        TelemetryWrapper.setLine(1,String.format("%8.1f Step 12: Moving towards depot...",runtime.milliseconds()));
//        driveTrainEnc.moveForthBackEnc(DEPOT_TO_CRATER,10000);
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
//            //driveTrain.moveLeftRightEnc(MOVE_TO_GOLD + LEFT_GOLD_DISTANCE, 10000);
//            driveTrainEnc.moveForthBackEnc(3142, 10000);
//        }
//        else if(goldPosition.equals(GoldPosition.RIGHT)) {
//            //driveTrain.moveLeftRightEnc(MOVE_TO_GOLD + RIGHT_GOLD_DISTANCE, 10000);
//            driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 10000);
//        }
//        else {
//            driveTrainEnc.moveForthBackEnc(PUSH_GOLD, 10000);
//        }
//
//        /** 2: Back out from pushing  */
//
//        //driveTrain.encoderDrive(0, -PULL_BACK, 0, 3);
//        driveTrainEnc.moveForthBackEnc(PULL_BACK,5000);
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
//        //driveTrain.encoderDrive(-distanceToMoveTowardsWall, 0, 0, 5);
//        driveTrainEnc.moveLeftRightEnc(-distanceToMoveTowardsWall,10000);
//
//        /** 4: Turn towards depot */
//        //driveTrain.encoderDrive(0, 0, -135, 5);
//        driveTrainEnc.spinEnc(-135, 10000);
//
//        /** 5: Head towards depot */
//        //driveTrain.encoderDrive(0, WALL_TO_DEPOT, 0, 5);
//        driveTrainEnc.moveForthBackEnc(WALL_TO_DEPOT,10000);
//
//        /** 6: Place team marker  */
//        placeTeamMarker();
//
//        /** 7: Back to crater */
//        //driveTrain.encoderDrive(0, DEPOT_TO_CRATER, 0, 5);
//        driveTrainEnc.moveForthBackEnc(-DEPOT_TO_CRATER,10000);
//
//        /** 8: Turn 180 to crater */
//        //driveTrain.encoderDrive(0, 0, 180, 5);
//        driveTrainEnc.spinEnc(180,10000);
//
//        /** 9: Put grabber into crater  */
//        //comment the code to pass by the action temporarily
//        //grabArm.automoveDownEnc(10000,opModeIsActive());
    }

    public void landFromLatch() {


//        TelemetryWrapper.setLine(1,"Landing from the latch...");
//        liftArm.moveUpDownEnc(AUTO_LIFT_DOWN_COUNTS, 2000);
//        driveTrainEnc.moveLeftRightEnc(40,2000); // move 40mm right to come out of the hook
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
                driveTrainEnc.spinEnc(Math.atan(mR.getHAlignSlope()),2000);
                continue;
            } else if (! mR.goldIsFound()) {

                driveTrainEnc.moveLeftRightEnc(increamentalDist,2000);
                continue;
            } else if ( mR.getFirstGoldAngle() > 1.5 ) {
                driveTrainEnc.spinEnc(mR.getFirstGoldAngle(),2000);
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
