package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.GoldDetector;
import javax.microedition.khronos.opengles.GL;



@Autonomous(name = "AutoRedFront")
public class AutoRedFront extends LinearOpMode {
    //define the parts that will be moving

    private ElapsedTime runtime = new ElapsedTime();
    //color of the team
    String teamColor = "RED";
    String foundColor = null;
    //ColorSensor colorSensor;
    private Config config = new Config(Config.configFile);
    private GoldDetector gd;

    @Override
    public void runOpMode() {
        DriveTrainByEncoder driveTrain = new DriveTrainByEncoder();
        driveTrain.init(hardwareMap,config);

//        double kickColorTime1 = config.getDouble("red_front_kick_color_time1",1);
//        double kickColorPower1 = config.getDouble("red_front_kick_color_power1",0.45);
//        double kickColorTime2 = config.getDouble("red_front_kick_color_time2",1);
//        double kickColorPower2 = config.getDouble("red_front_kick_color_power2",0.45);
//        double timeToRunLeft = config.getDouble("red_front_time_to_run_left",4);
//        double timeToRunCenter = config.getDouble("red_front_time_to_run_center",3.5);
//        double timeToRunRight = config.getDouble("red_front_time_to_run_right",3);
//        double timeToRunPower = config.getDouble("red_front_time_to_run_power",-0.2);
//        double turnTime = config.getDouble("red_front_turn_time",3);
//        double turnPower = config.getDouble("red_front_turn_power",-0.2);
//        double approachTime = config.getDouble("red_front_approach_time",1);
//        double approachPower = config.getDouble("red_front_approach_power",0.2);
//        double adjustPosTime = config.getDouble("red_front_adjust_pos_time",1);
//        double adjustPosPower = config.getDouble("red_front_adjust_pos_power",0.2);


        //waiting for user to press start
        waitForStart();

        TelemetryWrapper.init(telemetry,7);

        //turn on color sensor light
        //colorSensor.enableLed(true);
        //drop jewel arm so color sensor can detect
        runtime.reset();
        TelemetryWrapper.setLine(2,"" + opModeIsActive() + runtime.seconds());
        gd.init(hardwareMap,config);
        int detectID = 0;
        while (opModeIsActive() && (runtime.seconds() < 10)&& detectID == 0 ) {
            TelemetryWrapper.setLine(3,"Start Target detect...");
            gd.doDetect();
            detectID = gd.getId();
            //returns which color the jewel is
            switch (detectID){
                case 1: TelemetryWrapper.setLine(4,"Left is Gold");break;
                case 2: TelemetryWrapper.setLine(4,"Middle is Gold");break;
                case 3: TelemetryWrapper.setLine(4,"Right is Gold");break;
                default: TelemetryWrapper.setLine(4,"No Gold found!");
            }
        }
    }
}