package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import static org.firstinspires.ftc.teamcode.util.ButtonHelper.back;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_down;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_left;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_right;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_up;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.start;

@TeleOp(name="Test Enc Drive Distance",group = "Test")
//@Disabled
public class TestEncDriveDist extends LinearOpMode {

    ButtonHelper bH1 = null;
    ButtonHelper bH2 = null;

    /** Define your variables here */
    DriveTrainByEncoder driveDist = new DriveTrainByEncoder();
    Config config = new Config(Config.configFile);

    double dist = 100.0;
    double angle = 90.0;
    double power = 0.5;

    /** End of Definition **********/

    @Override
    public void runOpMode() {


        bH1 = new ButtonHelper(gamepad1);
        bH2 = new ButtonHelper(gamepad2);
        TelemetryWrapper.init(telemetry,10);

        /** Initialize the variables here */
        driveDist.initEnc(hardwareMap,config);
        /** End of Initialization *********/

        waitForStart();
        while (opModeIsActive()) {

            bH1.update();
            bH2.update();

            if (bH1.pressed(dpad_up)) {
                driveDist.moveForthBackEnc(power, dist, 10000);
                sleep(250);
            }
            if (bH1.pressed(dpad_down)) {
                driveDist.moveForthBackEnc(power, -dist, 10000);
                sleep(250);
            }
            if (bH1.pressed(dpad_left)) {
                driveDist.moveLeftRightEnc(-dist, 10000);
                sleep(250);
            }
            if (bH1.pressed(dpad_right)) {
                driveDist.moveLeftRightEnc(dist, 10000);
                sleep(250);
            }
            if (bH1.pressed(back)) {
                driveDist.spinEnc(power, angle,10000);
                sleep(250);
            }
            if (bH1.pressed(start)) {
                driveDist.spinEnc(power, -angle,10000);
                sleep(250);

            }

            if(gamepad1.left_bumper) {
                while(!gamepad1.right_bumper) {
                    if (gamepad1.b) {
                        dist += 50;
                        if (dist > 2000) {
                            dist = 100;
                        }
                        TelemetryWrapper.setLine(8,"Distance: " + dist);
                        sleep(200);
                    }
                    if (gamepad1.a) {
                        angle += 15;
                        if (dist > 360) {
                            dist = 0;
                        }
                        TelemetryWrapper.setLine(9,"Angle: "+angle);
                        sleep(200);
                    }
                    if (gamepad1.x) {
                        power += 0.1;
                        if (power > 1.0) {
                            power = 0;
                        }
                        TelemetryWrapper.setLine(7,"Power: "+power);
                        sleep(200);
                    }
                }
            }


//            // Quit by button 'back'
//            if (bH1.pressed(back)){
//                break;
//            }
//
//            // Functions by x, y
//            if (bH1.held(x)) {
//
//            }
//            if (bH1.releasing(x)) {
//
//            }
//            if (bH1.held(y)) {
//
//            }
//            if (bH1.releasing(y)) {
//
//            }

//            /** Functions by 'dpad_left' and 'dpad_right' */
//            if (bH1.held(dpad_left)) {
//
//            }
//            if (bH1.releasing(dpad_left)){
//
//            }
//            if (bH1.held(dpad_right)) {
//
//            }
//            if (bH1.releasing(dpad_right)){
//
//            }

        }

    }

}
