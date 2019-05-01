package org.firstinspires.ftc.teamcode;

// from com.qualcomm.robotcore.*
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// from org.firstinspires.ftc.robotcore.*
import org.firstinspires.ftc.robotcore.external.Telemetry;

// from standard java.*
import java.util.Map;

// from org.firstinspires.ftc.teamcode.*
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import static org.firstinspires.ftc.teamcode.Parameters.*;


public class ForeArm {

    ElapsedTime runtime = new ElapsedTime();

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;

    private  DigitalChannel touchSensor1;
    //private  DigitalChannel touchSensor2;

    private PID pidForUpDown = null;
    private PID pidForForthBack = null;

    boolean isUping = false;
    boolean isDowning = false;
    boolean isUpDownStopped = true;
    boolean isForwarding = false;
    boolean isBackwarding = false;
    boolean isForthBackStopped = true;

//    double FOREARM_UPDOWN_POWER = 1.0;
//    double FOREARM_FORTHBACK_POWER = 1.0;
//    int FOREARM_COUNTS_PER_UPDOWN_EFFORT = 50;
//    int FOREARM_COUNTS_PER_FORTHBACK_EFFORT =50;
//
//    int COUNTS_PER_REV_FOR_UPDOWN = 2240;
//    int GEAR_REDUCTION_FOR_UPDOWN_MOTOR = 12; // 20:40(chain) * 15:90 = 1:12

    HardwareMap hwMap = null;
    public ElapsedTime time = new ElapsedTime();


    public void init(HardwareMap Map, Config config, PID pidForUpDown, PID pidForForthBack) {
        hwMap = Map;
        motor1 = hwMap.get(DcMotor.class, "forearm1");
        motor2 = hwMap.get(DcMotor.class, "forearm2");
        motor3 = hwMap.get(DcMotor.class, "forearm3");
        touchSensor1 = hwMap.get(DigitalChannel.class, "forearm_touchsensor1");
        //touchSensor2 = hwMap.get(DigitalChannel.class, "forearm_touchsensor2");

        this.pidForUpDown = pidForUpDown;
        this.pidForForthBack = pidForForthBack;

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor1.setPower(0);
        motor2.setPower(0);

        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor3.setPower(0);

//        motor1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//        motor2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//        motor3.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//
//        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor1.setPower(0);
//        motor2.setPower(0);
//        motor3.setPower(0);

//
//        FOREARM_UPDOWN_POWER = config.getDouble("forearm_updown_power", 0.8);
//        FOREARM_FORTHBACK_POWER = config.getDouble("forearm_forthback_power",0.8);
//
//        FOREARM_COUNTS_PER_UPDOWN_EFFORT = config.getInt("forearm_counts_per_updown_effort", 50);
//        FOREARM_COUNTS_PER_FORTHBACK_EFFORT = config.getInt("forearm_counts_per_forthback_effort",50);

    }

    public void initEnc(HardwareMap Map, Config config, PID pidForUpDown, PID pidForForthBack) {
        hwMap = Map;
        motor1 = hwMap.get(DcMotor.class, "forearm1");
        motor2 = hwMap.get(DcMotor.class, "forearm2");
        motor3 = hwMap.get(DcMotor.class, "forearm3");

        touchSensor1 = hwMap.get(DigitalChannel.class, "forearm_touchsensor1");
        //touchSensor2 = hwMap.get(DigitalChannel.class, "forearm_touchsensor2");

        this.pidForUpDown = pidForUpDown;
        this.pidForForthBack = pidForForthBack;

//        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor1.setDirection(DcMotor.Direction.FORWARD);
//        motor2.setDirection(DcMotor.Direction.REVERSE);
//        motor1.setPower(0);
//        motor2.setPower(0);
//
//        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor3.setDirection(DcMotor.Direction.FORWARD);
//        motor3.setPower(0);

        motor1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setTargetPosition(motor1.getCurrentPosition());
        motor2.setTargetPosition(motor2.getCurrentPosition());
        motor3.setTargetPosition(motor3.getCurrentPosition());

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);

//        FOREARM_UPDOWN_POWER = config.getDouble("forearm_updown_power", 0.5);
//        FOREARM_FORTHBACK_POWER = config.getDouble("forearm_forthback_power",0.5);
//
//        FOREARM_COUNTS_PER_UPDOWN_EFFORT = config.getInt("forearm_counts_per_updown_effort", 50);
//        FOREARM_COUNTS_PER_FORTHBACK_EFFORT = config.getInt("forearm_counts_per_forthback_effort",50);

    }

/** **************** Begin of Part I **************************/
/**
 *  The following methods are used for manual control of forearm (Up/Down and Forth/Back)
 *  WITHOUT Encoder
 *
 * */
    public void moveUp() { moveUp( FOREARM_UPDOWN_POWER ); }
    public void moveDown() { moveDown( FOREARM_UPDOWN_POWER ); }

    public void moveUp(double power) { moveUpDown( - Math.abs(power) ); }
    public void moveDown(double power) { moveUpDown( Math.abs(power) ); }
    /** power: positive for moving down
     *         negative for moving up
     * @param power
     */
    public void moveUpDown(double power) {
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor1.setPower(power);
        motor2.setPower(power);
    }
    public void stopUpDown() {
        motor1.setPower(0);
        motor2.setPower(0);
    }

    public void moveForward() { moveForward(FOREARM_FORTHBACK_POWER);}
    public void moveBackward() { moveBackward(FOREARM_FORTHBACK_POWER);}
    public void moveForward(double power) { moveForthBack(Math.abs(power)); }
    public void moveBackward(double power) { moveForthBack(-Math.abs(power)); }
    /**
     * @param power
     *          - positive for forwards
     *          - negative for backwards
     */
    public void moveForthBack(double power) {
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor3.setPower(power);
    }
    public void stopForthBack() {
        motor3.setPower(0);
    }
/******************** End of Part I ************************************/

/** **************** Begin of Part II *************************/

/**
 *  The following methods are used for manual control of forearm (Up/Down and Forth/Back)
 *  WITH Encoder
 *
 * */
    public void moveForwardEnc(int timeoutMS) {

        ElapsedTime runtime = new ElapsedTime();
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newTarget3;
        newTarget3  = motor3.getCurrentPosition() - FOREARM_COUNTS_PER_FORTHBACK_EFFORT;
        motor3.setTargetPosition(newTarget3);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        motor3.setPower(FOREARM_FORTHBACK_POWER);
        while (motor3.isBusy()  && runtime.milliseconds() < timeoutMS ) {
            //wait
        }
    }
    public void moveBackwardEnc(int timeoutMS) {

        ElapsedTime runtime = new ElapsedTime();
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newTarget3;
        newTarget3  = motor3.getCurrentPosition() + FOREARM_COUNTS_PER_FORTHBACK_EFFORT;
        motor3.setTargetPosition(newTarget3);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        motor3.setPower(FOREARM_FORTHBACK_POWER);
        while (motor3.isBusy()  && runtime.milliseconds() < timeoutMS ) {
            //wait
        }

    }
    public void keepForwardingEnc() {
        double power = motor3.getPower();
        if (Math.abs(motor3.getTargetPosition() - motor3.getCurrentPosition()) < 100) {
            motor3.setTargetPosition(motor3.getCurrentPosition() - 1000);
        }
        if (power < FOREARM_FORTHBACK_POWER) {
            power += 0.01;
            if (power > FOREARM_FORTHBACK_POWER) {
                power = FOREARM_FORTHBACK_POWER;
            }
        }
        isForwarding = true;
        isBackwarding = false;
        isForthBackStopped = false;
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setPower(power);
    }
    public void keepBackwardingEnc() {
        double power = motor3.getPower();
        if (Math.abs(motor3.getTargetPosition() - motor3.getCurrentPosition()) < 100) {
            motor3.setTargetPosition(motor3.getCurrentPosition() + 1000);
        }
        if (power < FOREARM_FORTHBACK_POWER) {
            power += 0.01;
            if (power > FOREARM_FORTHBACK_POWER) {
                power = FOREARM_FORTHBACK_POWER;
            }
        }
        isForwarding = false;
        isBackwarding = true;
        isForthBackStopped = false;
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setPower(power);
    }
    public void stopForthBackEnc(){
//        double power = motor3.getPower();
//        if ((power > 0.1) && (motor3.isBusy())) {
//            if (isForwarding) {
//                motor3.setTargetPosition(motor3.getCurrentPosition() - 20);
//            } else if (isBackwarding) {
//                motor3.setTargetPosition(motor3.getCurrentPosition() + 20);
//            }
//            runtime.reset();
//            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor3.setPower(0.1);
//        }
//        while ((motor3.isBusy() ) && (runtime.milliseconds() < 100) ) {
//            // Waiting
//        }
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setTargetPosition(motor3.getCurrentPosition());
        motor3.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor3.setPower(0.0);

        isUping = false;
        isDowning = false;
        isUpDownStopped = true;

    }

    public void moveUpEnc(int timeoutMS) {

        ElapsedTime runtime = new ElapsedTime();
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newTarget1,newTarget2;
        newTarget1  = motor1.getCurrentPosition() + FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        newTarget2  = motor2.getCurrentPosition() - FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        motor1.setPower(FOREARM_UPDOWN_POWER);
        motor2.setPower(FOREARM_UPDOWN_POWER);

        while ((motor1.isBusy() && motor2.isBusy()) && runtime.milliseconds() < timeoutMS ) {
            //wait
        }
    }
    public void moveDownEnc(int timeoutMS) {

        ElapsedTime runtime = new ElapsedTime();
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newTarget1,newTarget2;
        newTarget1  = motor1.getCurrentPosition() - FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        newTarget2  = motor2.getCurrentPosition() + FOREARM_COUNTS_PER_UPDOWN_EFFORT;
        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        motor1.setPower(FOREARM_UPDOWN_POWER);
        motor2.setPower(FOREARM_UPDOWN_POWER);
        while ((motor1.isBusy() && motor2.isBusy()) && runtime.milliseconds() < timeoutMS)  {
            //wait
        }
    }
    public void keepUpingEnc() {
        double power = Math.max(motor1.getPower(),motor2.getPower());
        if (sensor1IsTouched()) {
            stopUpDownEnc();
            return;
        }
        if ((Math.abs(motor1.getTargetPosition() - motor1.getCurrentPosition()) < 100) ||
                (Math.abs(motor2.getTargetPosition() - motor2.getCurrentPosition()) < 100))   {
            motor1.setTargetPosition(motor1.getCurrentPosition() + 1000);
            motor2.setTargetPosition(motor2.getCurrentPosition() - 1000);
        }
        if (power < MAX_POPER_FOR_FOREARM_UPDOWN) {
            power += ACCELERATION_FOR_FOREARM_UPDOWN;
            if (power > MAX_POPER_FOR_FOREARM_UPDOWN) {
                power = MAX_POPER_FOR_FOREARM_UPDOWN;
            }
        }
        isUping = true;
        isDowning = false;
        isUpDownStopped = false;
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(power);
        motor2.setPower(power);
    }
    public void keepDowningEnc() {
        double power = Math.max(motor1.getPower(),motor2.getPower());
        if ((Math.abs(motor1.getTargetPosition() - motor1.getCurrentPosition()) < 100) ||
                (Math.abs(motor2.getTargetPosition() - motor2.getCurrentPosition()) < 100))   {
            motor1.setTargetPosition(motor1.getCurrentPosition() - 1000);
            motor2.setTargetPosition(motor2.getCurrentPosition() + 1000);
        }
        if (power < MAX_POPER_FOR_FOREARM_UPDOWN) {
            power += ACCELERATION_FOR_FOREARM_UPDOWN;
            if (power > MAX_POPER_FOR_FOREARM_UPDOWN) {
                power = MAX_POPER_FOR_FOREARM_UPDOWN;
            }
        }
        isUping = false;
        isDowning = true;
        isUpDownStopped = false;
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(power);
        motor2.setPower(power);
    }
    public void stopUpDownEnc() {

//        double power = Math.max(motor1.getPower(),motor2.getPower());
//        if ((power > 0.1) && (motor1.isBusy() || motor2.isBusy())) {
//            if (isUping) {
//                motor1.setTargetPosition(motor1.getCurrentPosition() - 20);
//                motor2.setTargetPosition(motor2.getCurrentPosition() + 20);
//            } else if (isDowning) {
//                motor1.setTargetPosition(motor1.getCurrentPosition() + 20);
//                motor2.setTargetPosition(motor2.getCurrentPosition() - 20);
//            }
//            runtime.reset();
//            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor1.setPower(0.1);
//            motor2.setPower(0.1);
//        }
//        while ((motor1.isBusy() && motor2.isBusy()) && (runtime.milliseconds() < 100) ) {
//            // Waiting
//        }
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(motor1.getCurrentPosition());
        motor2.setTargetPosition(motor2.getCurrentPosition());
        motor1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor1.setPower(0.0);
        motor2.setPower(0.0);

        isUping = false;
        isDowning = false;
        isUpDownStopped = true;
    }
/******************** End of Part II ************************************/


/******************** Start of Part III *********************************/
    /**
     * Function: Move the foreArm UP or DOWN automatically,
     * but WITHOUT PID adjustment by calling PID Class methods
     * <p>
     * power: is used for drive the motor
     * moveAngle: is the angle the ForeArm is going to move up/down automatically
     * - positive for moving up
     * - negative for moving down
     */
    public void moveUpDownAngleEnc(/**double topPower, */double moveAngle,int timeoutMS) {

        ElapsedTime runtime = new ElapsedTime();
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newTarget1,newTarget2;
        int MOVE_COUNTS = (int)( (moveAngle / 360) * COUNTS_PER_REV_FOR_UPDOWN * GEAR_REDUCTION_FOR_UPDOWN_MOTOR);
        newTarget1  = motor1.getCurrentPosition() + MOVE_COUNTS;
        newTarget2  = motor2.getCurrentPosition() - MOVE_COUNTS;

        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (MOVE_COUNTS > 0) {
            isUping = true;
            isDowning = false;
            isUpDownStopped = false;
        } else if (MOVE_COUNTS < 0 ) {
            isUping = false;
            isDowning = true;
            isUpDownStopped = false;
        } else {
            isUping = false;
            isDowning = false;
            isUpDownStopped = true;
        }
        runtime.reset();
        motor1.setPower(MIN_POWER_FOR_FOREARM_UPDOWN);
        motor2.setPower(MIN_POWER_FOR_FOREARM_UPDOWN);

        while ((motor1.isBusy() && motor2.isBusy()) && runtime.milliseconds()<timeoutMS) {
            //wait
            double power = Math.max(motor1.getPower(),motor2.getPower());
            if (sensor1IsTouched()) {
                break;
            }
            if (Math.abs(MOVE_COUNTS) >= (COUNTS_THRESHOLD_FOR_SLOWDOWN * 2 + 50)) {
                if ((Math.abs(motor1.getTargetPosition() - motor1.getCurrentPosition()) > (Math.abs(MOVE_COUNTS) - COUNTS_THRESHOLD_FOR_SLOWDOWN)) &&
                        (Math.abs(motor2.getTargetPosition() - motor2.getCurrentPosition()) > (Math.abs(MOVE_COUNTS) - COUNTS_THRESHOLD_FOR_SLOWDOWN)) ) {
                    if (power < MAX_POPER_FOR_FOREARM_UPDOWN) {
                        power += ACCELERATION_FOR_FOREARM_UPDOWN;
                        if (power > MAX_POPER_FOR_FOREARM_UPDOWN) {
                            power = MAX_POPER_FOR_FOREARM_UPDOWN;
                        }
                    }
                }
                if ((Math.abs(motor1.getTargetPosition() - motor1.getCurrentPosition()) <  COUNTS_THRESHOLD_FOR_SLOWDOWN) ||
                        (Math.abs(motor2.getTargetPosition() - motor2.getCurrentPosition()) <  COUNTS_THRESHOLD_FOR_SLOWDOWN)) {
                    if (power > MIN_POWER_FOR_FOREARM_UPDOWN) {
                        power -= ACCELERATION_FOR_FOREARM_UPDOWN;
                        if (power < MIN_POWER_FOR_FOREARM_UPDOWN) {
                            power = MIN_POWER_FOR_FOREARM_UPDOWN;
                        }
                    }
                }
            }
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(power);
            motor2.setPower(power);

            //////////////////////////////
        }
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(motor1.getCurrentPosition());
        motor2.setTargetPosition(motor2.getCurrentPosition());
        motor1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motor1.setPower(0);
        motor2.setPower(0);

        isUping = false;
        isDowning = false;
        isUpDownStopped = true;

    }
    public void moveUpDownAngleEncPID(double targetSpeed, double moveAngle,int timeoutMS) {
        /**
         * Function: Move the foreArm UP or DOWN automatically,
         *           WITH PID adjustment by calling PID class methods
         *
         * targetSpeed: is target ANGULAR speed in DEGREEs
         * moveAngle: is the angle the ForeArm is going to move up/down automatically
         *              - positive for moving up
         *              - negative for moving down
         */
        int newTarget1,newTarget2;
        ElapsedTime runtime = new ElapsedTime();

        double t1 = 0;
        double t2 = 0;
        double dt = 0;
        int pos1 = 0;
        int pos2 = 0;
        double actualSpeed = 0;
        double power = 0;

        int MOVE_COUNTS = (int)( (moveAngle / 360) * COUNTS_PER_REV_FOR_UPDOWN * GEAR_REDUCTION_FOR_UPDOWN_MOTOR);
        newTarget1  = motor1.getCurrentPosition() + MOVE_COUNTS;
        newTarget2  = motor2.getCurrentPosition() - MOVE_COUNTS;

        motor1.setTargetPosition(newTarget1);
        motor2.setTargetPosition(newTarget2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        t1 = runtime.milliseconds();
        pos1 = motor1.getCurrentPosition();

        power = MAX_POPER_FOR_FOREARM_UPDOWN;
        motor1.setPower(power);
        motor2.setPower(power);

        while ((motor1.isBusy() && motor2.isBusy()) && runtime.milliseconds()<timeoutMS) {
            //wait
            t2 = runtime.seconds();
            pos2 = motor1.getCurrentPosition();
            dt = t2 - t1;
            actualSpeed = ((((pos2 - pos1) / COUNTS_PER_REV_FOR_UPDOWN) * 360 ) /
                            GEAR_REDUCTION_FOR_UPDOWN_MOTOR ) / (t2 - t1);
            power += pidForUpDown.update(targetSpeed, actualSpeed, dt);
            motor1.setPower(power);
            motor2.setPower(power);
            pos1 = pos2;
            t1 = t2;
        }
    }

/******************** End of Part III *********************************/

    public boolean sensor1IsTouched() {
        return !touchSensor1.getState();
    }
}
