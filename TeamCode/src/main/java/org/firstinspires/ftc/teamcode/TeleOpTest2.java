package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;


@TeleOp(name = "TeleOp Mode Test 2",group = "Test")
public class TeleOpTest2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){

        //DriveTrain driveTrain = new DriveTrain();

        //driveTrain.init(hardwareMap,config);

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "fl_drive");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "fr_drive");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "rl_drive");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rr_drive");
        double speedx;
        double speedy;
        double offset;

        waitForStart();
        runtime.reset();

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setPower(0);
        rightFront.setPower(0);


        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setPower(0);
        rightBack.setPower(0);

        while(opModeIsActive()){

            double drivey =  gamepad1.left_stick_y;
            double drivex =  gamepad1.left_stick_x;
            double turn  =  -gamepad1.right_stick_x;

            speedx = drivex;
            speedy = drivey;
            offset = turn;
            leftFront.setPower(Range.clip(speedy-speedx+offset,-1,1));
            rightFront.setPower(Range.clip(speedy+speedx-offset,-1,1));
            leftBack.setPower(Range.clip(speedy+speedx+offset,-1,1));
            rightBack.setPower(Range.clip(speedy-speedx-offset,-1,1));
            /*if(Math.abs(drivey)>0.01||Math.abs(drivex)>0.01||Math.abs(turn)>0.01) {
                driveTrain.move(drivex,drivey,turn);
            }
            else{
                drivey =  gamepad2.left_stick_y*0.25;
                drivex =  gamepad2.left_stick_x*0.25;
                turn  =  -gamepad2.right_stick_x*0.25;
                driveTrain.move(drivex,drivey,turn);
            }*/
            // Show the elapsed game time and wheel power.

            //     if(vuMark != RelicRecoveryVuMark.UNKNOWN) sleep(CYCLE_MS);
            idle();
        }

        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
