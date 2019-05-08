package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.Config;

public class Parameters {

    // General parameters //////////////
    public static double TFOD_CONFIDENCE = 0.4;  // the default value in tfod class is 0.4
    public static double MINERAL_DETECT_TIMELIMIT = 2000; // the default timeout for detecting a mineral
    public static double CAMERA_TO_CENTER = -10.0; // the distance from camera to the left-right center line of the robot, negative on the left, positive on the right

    // for TeleOp //////////////////////
    public static double ANGLE_FOR_ONE_STEP = 45; //in degrees

    // for Autonomous //////////////////
    /** Distances are in millimeters */
    public static double DIST_BTWN_MINERALS = 368.3; // 14.5 inches * 25.4 mm/in
    public static double FIRST_MOVE_RIGHT = 40;
    public static double INITIAL_MOVE_TO_MINERAL = 382.5;
    public static double INITIAL_DIST_TO_WALL = 1275.0;
    public static double PUSH_GOLD =  382.5;
    public static double PULL_BACK = 382.5;
    public static double LEFT_GOLD_DISTANCE = -DIST_BTWN_MINERALS;
    public static double RIGHT_GOLD_DISTANCE = DIST_BTWN_MINERALS;
    public static double MIDDLE_GOLD_DISTANCE = 0;
    public static double DISTANCE_TO_WALL = INITIAL_DIST_TO_WALL;
    public static double WALL_TO_DEPOT = 663.0; // 1.5 * 597.0 (tile length) - 80.0 (FRONT_WHEEL_CENTER_TO_FRONT_RIM) - 305.0/2 (half of DIST_BTWN_FRONT_REAR_WHEELS) ;
    public static double DEPOT_TO_CRATER = 1326.0; // 597.0 (tile length)  * 3 - (80 + 305.0/2) [center to front rim] * 2;
    //public static double AUTO_DRIVE_SPEED = 0.8;

    // from ForeArm.java ////////////////
    /** for auto_updown */
    public static double MIN_POWER_FOR_FOREARM_UPDOWN = 0.1;
    public static double MAX_POPER_FOR_FOREARM_UPDOWN = 0.35;
    public static double ACCELERATION_FOR_FOREARM_UPDOWN = 0.01;
    public static double FOREARM_FORTHBACK_POWER = 0.5;
    /** for mannual updown */
    public static double FOREARM_UPDOWN_POWER_FOR_MANNUAL = 0.25;
    public static double ACCELERATION_FOR_FOREARM_UPDOWN_FOR_MANNUAL = 0.005;


    public static int FOREARM_COUNTS_PER_UPDOWN_EFFORT = 50;
    public static int FOREARM_COUNTS_PER_FORTHBACK_EFFORT =50;

    public static int COUNTS_PER_REV_FOR_UPDOWN = 2240;
    public static double GEAR_REDUCTION_FOR_UPDOWN_MOTOR = 6; // 20:40(chain) * 30:90 = 1:6

    // from driveTrainByEncoder ///////////
    public static int     COUNTS_PER_MOTOR_REV     = 1440 ;    // eg: TETRIX Motor Encoder
    public static double     DRIVE_GEAR_REDUCTION     = 1.0 ;     // This is < 1.0 if geared UP
    public static double     WHEEL_DIAMETER_MM        = 100.0 ;     // For figuring circumference
    /** for the  distance between two diagonal wheel */
    public static double     WHEEL_DIAGONAL_DISTANCE  = 464.2 ;     // the competition robot ( wheel distances: sqrt( 350mm ^ 2 * 305mm ^ 2) = 464.0
    public static double     DEGREE_CORRECTION        = 1.543;
    public static double     LINE_CORRECTION          = 1.0;
    public static double     XY_CORRECTION            = 1.2;
    public static int       COUNTS_PER_MM            = (int) ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415));
    public static int       COUNTS_PER_DEGREE        = (int) (WHEEL_DIAGONAL_DISTANCE / WHEEL_DIAMETER_MM  * COUNTS_PER_MOTOR_REV / 360. * DEGREE_CORRECTION );
    /** MIN_DRIVE_SPEED should be less than or equal to MAX_DRIVE_SPEED */
    public static double MIN_DRIVE_SPEED              = 0.2;
    public static double MAX_DRIVE_SPEED              = 0.9;
    public static int    COUNTS_THRESHOLD_FOR_SLOWDOWN = 50;

    // from LiftArm.java ////////////////////
    public static double LIFT_POWER = 0.9;
    public static int LIFT_COUNTS_PER_UPDOWN_EFFORT =50;
    public static double LIFT_AUTO_MOVE_DIST = 125.0; //Measured mannually
    public static int LIFT_AUTO_MOVE_COUNTS = 431; // (int) 125 / 97.5 [15 segments of chain * 6.5 mm/seg] * 56/24 [Reduction rate] * 144 (counts_per_rev)
    public static int LIFT_AUTO_LANDING_COUNTS = 436; // LIFT_AUTO_MOVE_COUNTS + 5
    public static int LIFT_AUTO_LATCHING_COUNTS = 556; // LIFT_AUTO_MOVE_COUNTS + 125

    // from MineralCollector.java ///////////
    public static double HOLDER_OPEN_POS = 0.65;     // Servo Position for holder_open
    public static double HOLDER_CLOSED_POS = 0.23;   // Servo Position for holder_closed
    public static double WIPE_ROTATION_SPEED = 0.99;

    // Smart phone's camera displacement, used by RobotLocator.java ///////
    public static int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
    public static int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    public static int CAMERA_LEFT_DISPLACEMENT     = 50;     // eg: Camera is 50 mm right of the robot's center line
    public static final float  mmPerInch        = 25.4f;
    public static final float  mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    public static final float  mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Initialization
    public static void init(Config config) {

        // General
        TFOD_CONFIDENCE = config.getDouble("tfod_confidence",0.4);  // the default value in tfod class is 0.4
        MINERAL_DETECT_TIMELIMIT = config.getDouble("mineral_detect_timelimit",2000);
        CAMERA_TO_CENTER = config.getDouble("camera_to_center", -10.0);

        // for TeleOp
        ANGLE_FOR_ONE_STEP = config.getDouble("angle_for_one_step",45.0); //in degrees

        // for Autonomous
        DIST_BTWN_MINERALS = config.getDouble("dist_btwn_minerals",368.3); // 14.5 inches * 25.4 mm/in
        FIRST_MOVE_RIGHT = config.getDouble("first_move_right",40);
        INITIAL_MOVE_TO_MINERAL = config.getDouble("initial_move_to_mineral",382.5);
        INITIAL_DIST_TO_WALL = config.getDouble("initial_dist_to_wall",1275.0);
        PUSH_GOLD =  config.getDouble("push_gold",382.5);
        PULL_BACK = config.getDouble("pull_back",382.5);
        LEFT_GOLD_DISTANCE = -DIST_BTWN_MINERALS;
        RIGHT_GOLD_DISTANCE = DIST_BTWN_MINERALS;
        MIDDLE_GOLD_DISTANCE = 0;
        DISTANCE_TO_WALL = INITIAL_DIST_TO_WALL;
        WALL_TO_DEPOT = config.getDouble("wall_to_depot",663.0); // 1.5 * 597.0 (tile length) - 80.0 (FRONT_WHEEL_CENTER_TO_FRONT_RIM) - 305.0/2 (half of DIST_BTWN_FRONT_REAR_WHEELS) ;
        DEPOT_TO_CRATER = config.getDouble("depot_to_crater",1326.0); // 597.0 (tile length)  * 3 - (80 + 305.0/2) [center to front rim] * 2;

        //AUTO_DRIVE_SPEED = config.getDouble("auto_drive_speed",0.8);
        // from ForeArm.java
        MIN_POWER_FOR_FOREARM_UPDOWN = config.getDouble("min_power_for_forearm_updown",0.1);
        MAX_POPER_FOR_FOREARM_UPDOWN = config.getDouble("max_power_for_forearm_updown",0.5);
        ACCELERATION_FOR_FOREARM_UPDOWN = config.getDouble("acceleration_for_forearm_updown",0.01);

        FOREARM_UPDOWN_POWER_FOR_MANNUAL = config.getDouble("forearm_updown_power_for_mannual", 0.25);
        ACCELERATION_FOR_FOREARM_UPDOWN_FOR_MANNUAL = config.getDouble("acceleration_for_forearm_updown_for_mannual",0.005);

        FOREARM_FORTHBACK_POWER = config.getDouble("forearm_forthback_power",0.5);

        FOREARM_COUNTS_PER_UPDOWN_EFFORT = config.getInt("forearm_counts_per_updown_effort", 50);
        FOREARM_COUNTS_PER_FORTHBACK_EFFORT = config.getInt("forearm_counts_per_forthback_effort",50);

        COUNTS_PER_REV_FOR_UPDOWN = config.getInt("counts_per_rev_for_updown",2240);
        GEAR_REDUCTION_FOR_UPDOWN_MOTOR = config.getDouble("gear_reduction_for_updown_motor",6); // 20:40(chain) * 30:90 = 1:6

        // from DriveTrainByEncoder
        COUNTS_PER_MOTOR_REV = config.getInt("counts_per_motor_rev", 1440);
        DRIVE_GEAR_REDUCTION = config.getDouble("drive_gear_reduction",1.0);
        WHEEL_DIAGONAL_DISTANCE = config.getDouble("wheel_diagonal_distance", 450.);
        WHEEL_DIAMETER_MM = config.getDouble("wheel_diameter", 100.);
        DEGREE_CORRECTION = config.getDouble("degree_correction", 1.543);
        LINE_CORRECTION = config.getDouble("line_correction", 1.0);
        XY_CORRECTION = config.getDouble("xy_correction", 1.2);
        COUNTS_PER_MM           = (int) ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415) * LINE_CORRECTION);
        COUNTS_PER_DEGREE       = (int) (WHEEL_DIAGONAL_DISTANCE / WHEEL_DIAMETER_MM  * COUNTS_PER_MOTOR_REV / 360. * DEGREE_CORRECTION);
        MIN_DRIVE_SPEED = config.getDouble("min_drive_speed",0.2);
        MAX_DRIVE_SPEED = config.getDouble("max_dirve_speed",0.9);
        COUNTS_THRESHOLD_FOR_SLOWDOWN = config.getInt("counts_threshold_for_slowdown",50);

        // from LiftArm.java
        LIFT_POWER = config.getDouble("lift_power", 0.8);
        LIFT_COUNTS_PER_UPDOWN_EFFORT = config.getInt("lift_counts_per_updown_effort", 50);

        LIFT_AUTO_MOVE_DIST = config.getDouble("lift_auto_move_dist",125.0);
        LIFT_AUTO_MOVE_COUNTS = config.getInt("lift_auto_move_counts",862);
        LIFT_AUTO_LANDING_COUNTS = config.getInt("lift_auto_landing_counts", 1000);
        LIFT_AUTO_LATCHING_COUNTS = config.getInt("lift_auto_latching_counts",1000);

        // from MineralCollector.java
        HOLDER_CLOSED_POS = config.getDouble("holder_closed_pos", 0.23);
        HOLDER_OPEN_POS = config.getDouble("holder_open_pos", 0.65);
        WIPE_ROTATION_SPEED = config.getDouble("wipe_rotation_speed", 0.99);
    }
}
