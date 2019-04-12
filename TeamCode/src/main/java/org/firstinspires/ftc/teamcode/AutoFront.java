package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

public class AutoFront extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //color of the team
    String teamColor = "RED";
    String foundColor = null;
    //ColorSensor colorSensor;
    private Config config = new Config(Config.configFile);
    private GoldDetector gd;

    private int moveToGold = 750;
    private int push = 150;
    private int pullBack = 150;
    private int rightGoldDistance = 800;
    private int middleGoldDistance = 400;
    private int distanceToWall = 830;
    private int wallToDepot = 1100;
    private int depotToCrater = 1700;

    @Override
    public void runOpMode() {
        DriveTrainByEncoder driveTrain = new DriveTrainByEncoder();
        driveTrain.init(hardwareMap, config);

        ForeArm grabArm = new ForeArm();
        grabArm.init(hardwareMap, config);

        //waiting for user to press start
        waitForStart();

        //[Code for lowering robot]

        TelemetryWrapper.init(telemetry, 7);

        //turn on color sensor light
        //colorSensor.enableLed(true);
        //drop jewel arm so color sensor can detect
        runtime.reset();
        TelemetryWrapper.setLine(2, "" + opModeIsActive() + runtime.seconds());
        gd.init(hardwareMap, config);
        int detectID = 0;
        while (opModeIsActive()) {
            while ((runtime.seconds() < 10) && detectID == 0) {
                TelemetryWrapper.setLine(3, "Start Target detect...");
                gd.doDetect();
                detectID = gd.getId();
                //returns which color the jewel is
                switch (detectID) {
                    case 1:
                        TelemetryWrapper.setLine(4, "Left is Gold");
                        break;
                    case 2:
                        TelemetryWrapper.setLine(4, "Middle is Gold");
                        break;
                    case 3:
                        TelemetryWrapper.setLine(4, "Right is Gold");
                        break;
                    default:
                        TelemetryWrapper.setLine(4, "No Gold found!");
                }
            }

            // 1: Detect Gold, push

            driveTrain.encoderDrive(1, 0, moveToGold, 0, 10);
            driveTrain.encoderDrive(1, 0, push, 0, 5);

            // 2: Back out from pushing

            driveTrain.encoderDrive(1, 0, -pullBack, 0, 7.5);

            // 3: Linear shift to wall side
            // 3.1: Gold is on the right

            if (detectID == 2) {
                driveTrain.encoderDrive(1, -(distanceToWall + rightGoldDistance), 0, 0, 10);
            }

            // 3.2: Gold is in the middle
            if (detectID == 3) {
                driveTrain.encoderDrive(1, -(distanceToWall + middleGoldDistance), 0, 0, 10);
            }

            // 3.3: Gold is on the left
            else {
                driveTrain.encoderDrive(1, -distanceToWall, 0, 0, 10);
            }

            // 4: Turn towards depot

            driveTrain.encoderDrive(1, 0, 0, -135, 7.5);

            // 5: Head towards depot

            driveTrain.encoderDrive(1, 0, wallToDepot, 0, 10);

            // 6: Place mascot

            // 7: Back to crater

            driveTrain.encoderDrive(1, 0, depotToCrater, 0, 20);

            // 8: Turn 180 to crater

            driveTrain.encoderDrive(1, 0, 0, 180, 20);

            // 9: Put grabber into crater

            grabArm.moveDown();


        }
    }
}
