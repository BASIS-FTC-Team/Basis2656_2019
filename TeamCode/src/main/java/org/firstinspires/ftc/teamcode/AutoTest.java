package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

@TeleOp(name="AutoTest",group = "Tests")
public class AutoTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Config config = new Config(Config.configFile);
    //private GoldDetector gd = new GoldDetector();
    private DriveTrainByEncoder driveTrain = new DriveTrainByEncoder();
    //private ForeArm grabArm = new ForeArm();

    private double AUTO_DRIVE_SPEED = 0.1;

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

    private GoldPosition goldPostion = GoldPosition.LEFT;

    @Override
    public void runOpMode() {

        /** Initialization  */
        driveTrain.init(hardwareMap, config);
        //grabArm.init(hardwareMap, config);
        //gd.init(hardwareMap, config);
        TelemetryWrapper.init(telemetry, 10);

        /** waiting for user to press start */
        waitForStart();

        while(opModeIsActive()) {

            /** Start timing */
            runtime.reset();
            TelemetryWrapper.setLine(2, "" + opModeIsActive() + runtime.seconds());

            /** 0.1: Landing from the latch */
            //landFromLatch();

            /** 0.2 Adjust the position and the orientation */
            //adjustInitialPosition();

            /** 0.3 Detect the Gold position */
            goldPostion = identifyGoldPosition();

            /** 1: Push or collect the Gold */

            if (gamepad1.dpad_left)
                driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, (MOVE_TO_GOLD + PUSH_GOLD), 0, 10);

            /** 2: Back out from pushing  */

            if (gamepad1.dpad_up)
                driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, -PULL_BACK, 0, 10);

            /** 3: Linear shift to wall side */
            int distanceToMoveTowardsWall = DISTANCE_TO_WALL;
            switch (goldPostion) {
                case LEFT:
                    distanceToMoveTowardsWall = DISTANCE_TO_WALL + LEFT_GOLD_DISTANCE;
                case MIDDLE:
                    distanceToMoveTowardsWall = DISTANCE_TO_WALL + MIDDLE_GOLD_DISTANCE;
                case RIGHT:
                    distanceToMoveTowardsWall = DISTANCE_TO_WALL + RIGHT_GOLD_DISTANCE;
                default:
                    distanceToMoveTowardsWall = DISTANCE_TO_WALL;
            }

            if (gamepad1.dpad_right)
                driveTrain.encoderDrive(AUTO_DRIVE_SPEED, -distanceToMoveTowardsWall, 0, 0, 10);

            /** 4: Turn towards depot */

            if (gamepad1.dpad_down)
                driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, 0, 135, 10);

            /** 5: Head towards depot */
            if (gamepad1.a)
                driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, WALL_TO_DEPOT, 0, 10);

            /** 6: Place team marker  */
            //placeTeamMarker();

            if (gamepad1.b)
                driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, 0, 180, 10);

            /** 7: Back to crater */
            if (gamepad1.x)
                driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, -DEPOT_TO_CRATER, 0, 10);

            /** 8: Turn 180 to crater */
            if (gamepad1.y)
                driveTrain.encoderDrive(AUTO_DRIVE_SPEED, 0, 0, 180, 10);

            /** 9: Put grabber into crater  */
            //comment the code to pass by the action temporarily
            //grabArm.automoveDown();
        }
    }

    public GoldPosition identifyGoldPosition() {
        return GoldPosition.LEFT;
    }

}
