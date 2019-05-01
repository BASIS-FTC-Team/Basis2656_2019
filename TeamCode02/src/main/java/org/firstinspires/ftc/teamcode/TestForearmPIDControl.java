/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import static org.firstinspires.ftc.teamcode.Parameters.ANGLE_FOR_ONE_STEP;

//import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;

/**
 * This test script is copied from TeleOp_Basis2019 and modified to focus on Encoder control test of the forearm.
 *
 *      - PID or Non-PID control
 *      - Auto moving or manually controlled moveing
 */
@TeleOp(name = "TeleOp_Basis2656_2019",group = "Test")
@Disabled
public class TestForearmPIDControl extends LinearOpMode {

    LiftArm liftArm = new LiftArm();
    ForeArm foreArm = new ForeArm();
    MineralCollector mineralCollector = new MineralCollector();
    DriveTrain driveTrain = new DriveTrain();
    PID pidForForearmUpDown = null;
    PID pidForForearmForthBack = null;
    PID pidForLiftUpDown = null;

    private Config config = new Config(Config.configFile);

    boolean foreArmIsUping = false;
    boolean foreArmIsDowning = false;
    boolean foreArmIsForwarding = false;
    boolean foreArmIsBacking = false;
    boolean mineralCollectorIsWipingOut = false;
    boolean mineralCollectorIsWipingIn = false;
    boolean liftIsUping = false;
    boolean liftIsDowning = false;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private ButtonHelper helper;
    private ButtonHelper helper2;

    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {

        helper = new ButtonHelper(gamepad1);
        helper2 = new ButtonHelper(gamepad2);

        pidForForearmUpDown = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);
        pidForForearmForthBack = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);
        pidForLiftUpDown = new PID(0.5,0.05,0.05,-3.0,3.0,-0.1,0.1);

        driveTrain.init(hardwareMap,config);
        liftArm.init(hardwareMap,config,pidForLiftUpDown);
        foreArm.init(hardwareMap,config,pidForForearmUpDown,pidForForearmForthBack);
        mineralCollector.init(hardwareMap,config);


        // Wait for the start button
        telemetry.addData(">", "Press Start to Servo test for arm and container." );
        telemetry.update();

        TelemetryWrapper.init(telemetry,11);


        waitForStart();
        runtime.reset();

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            helper.update();
            helper2.update();

            double drivey =  -gamepad1.left_stick_y;
            double drivex =  -gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;

            // Show the elapsed game time and wheel power.
            TelemetryWrapper.setLine(4,"Motors in drivex: " + drivex +"drivey: " + drivey+" turn: "+turn);
            TelemetryWrapper.setLine(5, "Press Stop to end test." );


            /** The following part is for automove testing */

            if (gamepad2.dpad_down) {
                foreArm.moveUpDownAngleEnc(ANGLE_FOR_ONE_STEP,2000);
            }
            if (gamepad2.dpad_up) {
                foreArm.moveUpDownAngleEnc(ANGLE_FOR_ONE_STEP,2000);
            }

            if (gamepad2.x) {
                foreArm.moveUpDownAngleEncPID(10.0,45.0,2000);
            }
            if (gamepad2.b) {
                foreArm.moveUpDownAngleEncPID(10.0,-45.0,2000);
            }

            if (gamepad2.y) {
                foreArm.stopUpDownEnc();
            }


            /** End of automove testing */


            idle();

        }


        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }

}

