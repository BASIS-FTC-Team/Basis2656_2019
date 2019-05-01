package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import static org.firstinspires.ftc.teamcode.util.ButtonHelper.back;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_down;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_left;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_right;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.dpad_up;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.x;
import static org.firstinspires.ftc.teamcode.util.ButtonHelper.y;

@TeleOp(name="Test Servo 6621MG360",group = "Test")
//@Disabled
public class TestServo6621MG360 extends LinearOpMode {

    ButtonHelper bH1 = null;
    ButtonHelper bH2 = null;

    /** Define your variables here */
    //CRServo servo = null;
    public CRServo servo= null;

    @Override
    public void runOpMode() {

        bH1 = new ButtonHelper(gamepad1);
        bH2 = new ButtonHelper(gamepad2);
        TelemetryWrapper.init(telemetry,10);
        ElapsedTime runtime = new ElapsedTime();
        double power= 1;
        int times = 0;

        /** Initialize the variables here */
        servo = hardwareMap.crservo.get("Servo12");

        // Set the rotation servo for extended PWM range
        if (servo.getController() instanceof ServoControllerEx) {

            // Confirm its an extended range servo controller before we try to set to avoid crash
            ServoControllerEx theControl = (ServoControllerEx) servo.getController();
            int thePort = servo.getPortNumber();
            PwmControl.PwmRange theCurrentRange = theControl.getServoPwmRange(thePort);
            TelemetryWrapper.setLine(1,String.format("The range is : %.4f, %.4f",theCurrentRange.usPulseLower, theCurrentRange.usPulseUpper));
            PwmControl.PwmRange theRangetoSet = new PwmControl.PwmRange(500, 2500);
            theControl.setServoPwmRange(thePort, theRangetoSet);
            theCurrentRange = theControl.getServoPwmRange(thePort);
            TelemetryWrapper.setLine(3,String.format("The range is : %.4f, %.4f",theCurrentRange.usPulseLower, theCurrentRange.usPulseUpper));

        }
        /**********************************/

        waitForStart();
        while (opModeIsActive()) {

            bH1.update();
            bH2.update();

            if (bH1.pressed(back)){
                break;
            }
            if (bH1.held(x)) {

            }
            if (bH1.releasing(x)) {

            }
            if (bH1.held(y)) {

            }
            if (bH1.releasing(y)) {

            }

            if(bH1.pressed(dpad_up)) {
                power += 0.05;
                if (power > 1) {
                    power = 1.0;
                }
                TelemetryWrapper.setLine(0,String.format("Power: %.2f",power));
                sleep(200);
            }
            if(bH1.pressed(dpad_down)) {
                power -= 0.05;
                if (power <  0) {
                    power = 0;
                }
                TelemetryWrapper.setLine(0,String.format("Power: %.2f",power));
                sleep(200);
            }
            if (bH1.held(dpad_left)) {
                times ++;
                if (times == 1) {
                    runtime.reset();
                    TelemetryWrapper.setLine(9,String.format("Times: %d, Runtime: %.2f", times, runtime.milliseconds()));
                }
                servo.setDirection(CRServo.Direction.REVERSE);
                servo.setPower(power);
            }
            if (bH1.releasing(dpad_left)){
                TelemetryWrapper.setLine(9,String.format("Times: %d, Runtime: %.2f", times, runtime.milliseconds()));
                times = 0;
                servo.setDirection(CRServo.Direction.REVERSE);
                servo.setPower(0);
            }
            if (bH1.held(dpad_right)) {
                times ++;
                if (times == 1) {
                    runtime.reset();
                    TelemetryWrapper.setLine(9,String.format("Times: %d, Runtime: %.2f", times, runtime.milliseconds()));
                }
                servo.setDirection(CRServo.Direction.FORWARD);
                servo.setPower(power);
            }
            if (bH1.releasing(dpad_right)){
                TelemetryWrapper.setLine(9,String.format("Times: %d, Runtime: %.2f", times, runtime.milliseconds()));
                times = 0;
                servo.setDirection(CRServo.Direction.FORWARD);
                servo.setPower(0);
            }

        }

    }

}
