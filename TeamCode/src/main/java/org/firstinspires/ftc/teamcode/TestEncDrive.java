package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveTrainByEncoder;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

@TeleOp(name="Test Encoder Drive",group="Test")
@Disabled
public class TestEncDrive extends LinearOpMode {


    private Config config = new Config(Config.configFile);
    //private DriveTrainByEncoder trainByEncoder = new DriveTrainByEncoder()
    private DriveTrainByEncoder driveTrain = new DriveTrainByEncoder();

    @Override
    public void runOpMode() {

        driveTrain.initEnc(hardwareMap,config);
        TelemetryWrapper.init(telemetry,11);

        while (opModeIsActive()) {
            //Move forward
            if (gamepad1.dpad_up) {
                driveTrain.moveForthBackEnc(0.5,500., 10000);
                TelemetryWrapper.setLine(1,"50cm FORWARD moved");
                sleep(200);
            }
            //Move backward
            if (gamepad1.dpad_down) {
                driveTrain.moveForthBackEnc(0.5,-500.,10000);
                TelemetryWrapper.setLine(1,"50cm BACKWARD moved");
                sleep(200);
            }
            //Move left
            if (gamepad1.dpad_left) {
                driveTrain.moveLeftRightEnc(0.5,-500., 10000);
                TelemetryWrapper.setLine(1,"50cm LEFT moved");
                sleep(200);
            }
            //Move right
            if (gamepad1.dpad_right) {
                driveTrain.moveLeftRightEnc(0.5,500., 10000);
                TelemetryWrapper.setLine(1,"50cm RIGHT moved");
                sleep(200);
            }
            //Turn CCW
            if (gamepad1.x) {
                driveTrain.spinEnc(0.5,-45., 10000);
                TelemetryWrapper.setLine(1,"45 degree Clock-Wise turned");
                sleep(200);
            }
            // Turn CW
            if (gamepad1.y) {
                driveTrain.spinEnc(0.5,45., 10000);
                TelemetryWrapper.setLine(1,"45 degree Counter-Clock-Wise turned");
                sleep(200);
            }
            if (gamepad1.b) {
                break;
            }
        }
    }


}
