package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

@Autonomous(name="Auto30s-Case01",group = "Auto30s")

public class Auto30sCase01 extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    //color of the team
//    private String teamColor = "RED";
//    private String foundColor = null;
    //ColorSensor colorSensor;
    private Config config = new Config(Config.configFile);
    private GoldDetector gd = new GoldDetector();
    private DriveTrainByEncoder driveTrain = new DriveTrainByEncoder();
    //private ForeArm grabArm = new ForeArm();


    // Auto drive speed ( from 0.0 to 1.0 )
    private double AUTO_DRIVE_SPEED = 0.3;

    // Distances in mm
    private int MOVE_TO_GOLD = 750;
    private int PUSH_GOLD = 150;
    private int PULL_BACK = 250;
    private int LEFT_GOLD_DISTANCE = -400;
    private int RIGHT_GOLD_DISTANCE = 400;
    private int MIDDLE_GOLD_DISTANCE = 0;
    private int DISTANCE_TO_WALL = 1230;
    private int WALL_TO_DEPOT = 1100;
    private int DEPOT_TO_CRATER = 1700;

    private GoldPosition goldPosition = GoldPosition.MIDDLE;

    @Override
    public void runOpMode() {

        /** Initialization  */
        driveTrain.init(hardwareMap, config);
        //grabArm.init(hardwareMap, config);
        //gd.init(hardwareMap, config);
        TelemetryWrapper.init(telemetry, 10);

        /** waiting for user to press start */
        waitForStart();

        /** Start timing */
        runtime.reset();
        TelemetryWrapper.setLine(2, "" + opModeIsActive() + runtime.seconds());

        /** 0.1: Landing from the latch */
        landFromLatch();

        /** 0.2 Adjust the position and the orientation */
        adjustInitialPosition();

        /** 0.3 Detect the Gold position */
        goldPosition = identifyGoldPosition();

        /** 1: Push or collect the Gold */

        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, MOVE_TO_GOLD + PUSH_GOLD, 0, 3);
        if(goldPosition.equals(GoldPosition.LEFT)) {
            driveTrain.moveLeftRightEnc(AUTO_DRIVE_SPEED, MOVE_TO_GOLD + LEFT_GOLD_DISTANCE, 10000, opModeIsActive());
            driveTrain.moveForthBackEnc(AUTO_DRIVE_SPEED, PUSH_GOLD, 10000, opModeIsActive());
        }
        else if(goldPosition.equals(GoldPosition.RIGHT)) {
            driveTrain.moveLeftRightEnc(AUTO_DRIVE_SPEED, MOVE_TO_GOLD + RIGHT_GOLD_DISTANCE, 10000, opModeIsActive());
            driveTrain.moveForthBackEnc(AUTO_DRIVE_SPEED, PUSH_GOLD, 10000, opModeIsActive());
        }
        else {
            driveTrain.moveForthBackEnc(AUTO_DRIVE_SPEED, MOVE_TO_GOLD + PUSH_GOLD, 10000, opModeIsActive());
        }

        /** 2: Back out from pushing  */

        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, -PULL_BACK, 0, 3);
        driveTrain.moveForthBackEnc(AUTO_DRIVE_SPEED,PULL_BACK,5000,opModeIsActive());

        /** 3: Linear shift to wall side */
        int distanceToMoveTowardsWall = DISTANCE_TO_WALL;
        switch (goldPosition) {
            case LEFT:
                distanceToMoveTowardsWall = DISTANCE_TO_WALL + LEFT_GOLD_DISTANCE;
            case MIDDLE:
                distanceToMoveTowardsWall = DISTANCE_TO_WALL + MIDDLE_GOLD_DISTANCE;
            case RIGHT:
                distanceToMoveTowardsWall = DISTANCE_TO_WALL + RIGHT_GOLD_DISTANCE;
            default:
                distanceToMoveTowardsWall = DISTANCE_TO_WALL;
        }
        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, -distanceToMoveTowardsWall, 0, 0, 5);
        driveTrain.moveLeftRightEnc(AUTO_DRIVE_SPEED, -distanceToMoveTowardsWall,10000,opModeIsActive());

        /** 4: Turn towards depot */
        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, 0, -135, 5);
        driveTrain.spinEnc(AUTO_DRIVE_SPEED,-135, 10000,opModeIsActive());

        /** 5: Head towards depot */
        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, WALL_TO_DEPOT, 0, 5);
        driveTrain.moveForthBackEnc(AUTO_DRIVE_SPEED,WALL_TO_DEPOT,10000, opModeIsActive());

        /** 6: Place team marker  */
        placeTeamMarker();

        /** 7: Back to crater */
        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, DEPOT_TO_CRATER, 0, 5);
        driveTrain.moveForthBackEnc(AUTO_DRIVE_SPEED,-DEPOT_TO_CRATER,10000,opModeIsActive());

        /** 8: Turn 180 to crater */
        //driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, 0, 180, 5);
        driveTrain.spinEnc(AUTO_DRIVE_SPEED,180,10000,opModeIsActive());

        /** 9: Put grabber into crater  */
        //comment the code to pass by the action temporarily
        //grabArm.automoveDownEnc(10000,opModeIsActive());
    }


    public void landFromLatch() {

    }

    public void adjustInitialPosition() {

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

    public GoldPosition identifyGoldPosition() {
        return GoldPosition.LEFT;
    }

    public void placeTeamMarker() {

    }

//    public void waitForTest() {
//
//        int detectID = 0;
//        while (opModeIsActive()) {
//            while ((runtime.seconds() < 10) && detectID == 0) {
//                TelemetryWrapper.setLine(3, "Start Target detect...");
//                gd.doDetect();
//                detectID = gd.getId();
//                //returns which color the jewel is
//                switch (detectID) {
//                    case 1:
//                        TelemetryWrapper.setLine(4, "Left is Gold");
//                        break;
//                    case 2:
//                        TelemetryWrapper.setLine(4, "Middle is Gold");
//                        break;
//                    case 3:
//                        TelemetryWrapper.setLine(4, "Right is Gold");
//                        break;
//                    default:
//                        TelemetryWrapper.setLine(4, "No Gold found!");
//                }
//            }
//        }
//    }
}
