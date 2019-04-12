package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.util.Config;

@TeleOp(name="Check the Config Data")
public class ConfigData extends LinearOpMode {

    private Config config = new Config(Config.configFile);

    double FOREARM_UPDOWN_POWER = 0.01;
    double FOREARM_FORTHBACK_POWER = 0.01;

    int FOREARM_COUNTS_PER_UPDOWN_EFFORT = 10;
    int FOREARM_COUNTS_PER_FORTHBACK_EFFORT = 10;

    double HOLDER_CLOSED = 0.01;
    double HOLDER_OPEN = 0.01;
    double WIPE_SPEED = 0.01;

    double LIFT_POWER = 0.01;
    int LIFT_COUNTS_PER_UPDOWN_EFFORT = 10;


    @Override
    public void runOpMode() {

        waitForStart();

        int loops = 0;

        TelemetryWrapper.init(telemetry, 11);

        while (opModeIsActive()) {

            loops ++;
            TelemetryWrapper.setLine(0,"Loops in opModeIsActive():" + loops);

            if (gamepad1.a) {
                FOREARM_UPDOWN_POWER = config.getDouble("forearm_updown_power", 0.01);
                FOREARM_FORTHBACK_POWER = config.getDouble("forearm_forthback_power", 0.01);

                FOREARM_COUNTS_PER_UPDOWN_EFFORT = config.getInt("forearm_counts_per_updown_effort", 10);
                FOREARM_COUNTS_PER_FORTHBACK_EFFORT = config.getInt("forearm_counts_per_forthback_effort", 10);

                HOLDER_CLOSED = config.getDouble("holder_closed_pos", 0.01);
                HOLDER_OPEN = config.getDouble("holder_open_pos", 0.01);
                WIPE_SPEED = config.getDouble("wipe_rotation_speed", 0.01);

                LIFT_POWER = config.getDouble("lift_power", 0.01);
                LIFT_COUNTS_PER_UPDOWN_EFFORT = config.getInt("lift_counts_per_updown_effort", 10);

                TelemetryWrapper.setLine(1, "FOREARM_U/D_PWR: " + FOREARM_UPDOWN_POWER);
                TelemetryWrapper.setLine(2, "FOREARM_F/B_PWR: " + FOREARM_FORTHBACK_POWER);
                TelemetryWrapper.setLine(3, "FOREARM_COUNTS_PER_UPDOWN_EFFORT: " + FOREARM_COUNTS_PER_UPDOWN_EFFORT);
                TelemetryWrapper.setLine(4, "FOREARM_COUNTS_PER_FORTHBACK_EFFORT: " + FOREARM_COUNTS_PER_FORTHBACK_EFFORT);
                TelemetryWrapper.setLine(5, "HOLDER_CLOSED: " + HOLDER_CLOSED);
                TelemetryWrapper.setLine(6, "HOLDER_OPEN: " + HOLDER_OPEN);
                TelemetryWrapper.setLine(7, "WIPE_SPEED: " + WIPE_SPEED);
                TelemetryWrapper.setLine(8, "LIFT_POWER: " + LIFT_POWER);
                TelemetryWrapper.setLine(9, "LIFT_COUNTS_PER_UPDOWN_EFFORT: " + LIFT_COUNTS_PER_UPDOWN_EFFORT);

                sleep(200);
            }
        }
    }
}
