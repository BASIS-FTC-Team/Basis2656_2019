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
    private ForeArm grabArm = new ForeArm();


    // Auto drive speed ( from 0.0 to 1.0 )
    private double AUTO_DRIVE_SPEED = 0.5;

    // Distances in mm
    private int MOVE_TO_GOLD = 750;
    private int PUSH_GOLD = 150;
    private int PULL_BACK = 150;
    private int LEFT_GOLD_DISTANCE = 0;
    private int RIGHT_GOLD_DISTANCE = 800;
    private int MIDDLE_GOLD_DISTANCE = 400;
    private int DISTANCE_TO_WALL = 830;
    private int WALL_TO_DEPOT = 1100;
    private int DEPOT_TO_CRATER = 1700;

    private int goldPostion = 0;

    @Override
    public void runOpMode() {

        /** Initialization  */
        driveTrain.init(hardwareMap, config);
        grabArm.init(hardwareMap, config);
        gd.init(hardwareMap, config);
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
        goldPostion = identifyGoldPosition();

        /** 1: Push or collect the Gold */

        driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, MOVE_TO_GOLD + PUSH_GOLD, 0, 3);

        /** 2: Back out from pushing  */

        driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, -PULL_BACK, 0, 3);

        /** 3: Linear shift to wall side */
        int distanceToMoveTowardWall = DISTANCE_TO_WALL;
        switch (goldPostion) {
            case 1:
                distanceToMoveTowardWall = DISTANCE_TO_WALL + LEFT_GOLD_DISTANCE;
            case 2:
                distanceToMoveTowardWall = DISTANCE_TO_WALL + MIDDLE_GOLD_DISTANCE;
            case 3:
                distanceToMoveTowardWall = DISTANCE_TO_WALL + RIGHT_GOLD_DISTANCE;
            default:
                distanceToMoveTowardWall = DISTANCE_TO_WALL;
        }
        driveTrain.encoderDrive(AUTO_DRIVE_SPEED, -distanceToMoveTowardWall, 0, 0, 5);

        /** 4: Turn towards depot */
        driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, 0, -135, 5);

        /** 5: Head towards depot */
        driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, WALL_TO_DEPOT, 0, 5);

        /** 6: Place team marker  */
        placeTM();

        /** 7: Back to crater */
        driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, DEPOT_TO_CRATER, 0, 5);

        /** 8: Turn 180 to crater */
        driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, 0, 180, 5);

        /** 9: Put grabber into crater  */
        //comment the code to pass by the action temporarily
        //grabArm.automoveDown();
    }


    public void landFromLatch() {

    }

    public void adjustInitialPosition() {

    }

    /**
     * To detect the Gold mineral sample and  identify the position
     *
     * @return 1 - Gold is on the left
     * 2 - in the middle
     * 3 - on the right
     * 0 - no mineral detected
     */

    public int identifyGoldPosition() {

        return 1;
    }

    public void placeTM() {

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
