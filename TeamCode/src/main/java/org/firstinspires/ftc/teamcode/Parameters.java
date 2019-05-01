package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.Config;

public class Parameters {

    // from ForeArm.java
    public static double MIN_POWER_FOR_FOREARM_UPDOWN = 0.1;
    public static double MAX_POPER_FOR_FOREARM_UPDOWN = 0.8;
    public static double ACCELERATION_FOR_FOREARM_UPDOWN = 0.01;


    public static double FOREARM_UPDOWN_POWER = 0.5;
    public static double FOREARM_FORTHBACK_POWER = 0.5;

    public static double ANGLE_FOR_ONE_STEP = 45; //in degrees


    public static int FOREARM_COUNTS_PER_UPDOWN_EFFORT = 50;
    public static int FOREARM_COUNTS_PER_FORTHBACK_EFFORT =50;

    public static int COUNTS_PER_REV_FOR_UPDOWN = 2240;
    public static int GEAR_REDUCTION_FOR_UPDOWN_MOTOR = 12; // 20:40(chain) * 15:90 = 1:12

    // from LiftArm.java
//    public static double LIFT_POWER = 0.9;
//    public static int LIFT_COUNTS_PER_UPDOWN_EFFORT =50;
//    public static double LIFT_AUTO_MOVE_DIST = 125.0; //Measured mannually
//    public static int LIFT_AUTO_MOVE_COUNTS = 431; // (int) 125 / 97.5 [15 segments of chain * 6.5 mm/seg] * 56/24 [Reduction rate] * 144 (counts_per_rev)
//    public static int LIFT_AUTO_LANDING_COUNTS = 436; // LIFT_AUTO_MOVE_COUNTS + 5
//    public static int LIFT_AUTO_LATCHING_COUNTS = 556; // LIFT_AUTO_MOVE_COUNTS + 125

//
//    Config config = new Config(Config.configFile);
    // Initialization
    public static void init(Config config) {

        // from ForeArm.java
        MIN_POWER_FOR_FOREARM_UPDOWN = config.getDouble("min_power_for_forearm_updown",0.1);
        MAX_POPER_FOR_FOREARM_UPDOWN = config.getDouble("max_power_for_forearm_updown",0.8);
        ACCELERATION_FOR_FOREARM_UPDOWN = config.getDouble("acceleration_for_forearm_updown",0.01);

        FOREARM_UPDOWN_POWER = config.getDouble("forearm_updown_power", 0.5);
        FOREARM_FORTHBACK_POWER = config.getDouble("forearm_forthback_power",0.5);

        FOREARM_COUNTS_PER_UPDOWN_EFFORT = config.getInt("forearm_counts_per_updown_effort", 50);
        FOREARM_COUNTS_PER_FORTHBACK_EFFORT = config.getInt("forearm_counts_per_forthback_effort",50);

        // from LiftArm.java
//        LIFT_POWER = config.getDouble("lift_power", 0.8);
//        LIFT_COUNTS_PER_UPDOWN_EFFORT = config.getInt("lift_counts_per_updown_effort", 50);
//
//        LIFT_AUTO_MOVE_DIST = config.getDouble("lift_auto_move_dist",125.0);
//        LIFT_AUTO_MOVE_COUNTS = config.getInt("lift_auto_move_counts",431);
//        LIFT_AUTO_LANDING_COUNTS = config.getInt("lift_auto_landing_counts", 436);
//        LIFT_AUTO_LATCHING_COUNTS = config.getInt("lift_auto_latching_counts",556);

    }
}
