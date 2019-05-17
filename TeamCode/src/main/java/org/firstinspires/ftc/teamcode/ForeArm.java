package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Hardware2019.*;
import static org.firstinspires.ftc.teamcode.Parameters.*;

public class ForeArm {

    boolean isUping = false;
    boolean isDowning = false;
    boolean isAutoUping = false;
    boolean isAutoDowning = false;

    boolean isForwarding = false;
    boolean isBackwarding = false;

    int AUTO_MOVE_COUNTS = 0;

    public void init() {

        setModeForUpDown(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor1.setPower(0);
        motor2.setPower(0);

        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor3.setPower(0);

    }

    public void initEnc() {

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

    }

/** **************** Begin of Part I **************************/
/**
 *  The following methods are used for manual control of forearm (Up/Down and Forth/Back)
 *  WITHOUT Encoder
 *
 * */
    public void moveUp() { moveUp( FOREARM_UPDOWN_POWER_FOR_MANNUAL ); }

    public void moveDown() { moveDown( FOREARM_UPDOWN_POWER_FOR_MANNUAL ); }

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
        moveForwardEnc(FOREARM_COUNTS_PER_FORTHBACK_EFFORT, timeoutMS);
    }

    public void moveForwardEnc(int counts, int timeoutMS) {

        ElapsedTime runtime = new ElapsedTime();
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newTarget3;
        newTarget3  = motor3.getCurrentPosition() - counts;
        motor3.setTargetPosition(newTarget3);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        motor3.setPower(FOREARM_FORTHBACK_POWER);
        while (motor3.isBusy()  && runtime.milliseconds() < timeoutMS ) {
            //wait
        }
    }

    public void moveBackwardEnc(int timeoutMS) {
        moveBackwardEnc(FOREARM_COUNTS_PER_FORTHBACK_EFFORT, timeoutMS);
    }

    public void moveBackwardEnc(int counts, int timeoutMS) {

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
        motor1.setPower(FOREARM_UPDOWN_POWER_FOR_MANNUAL);
        motor2.setPower(FOREARM_UPDOWN_POWER_FOR_MANNUAL);

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
        motor1.setPower(FOREARM_UPDOWN_POWER_FOR_MANNUAL);
        motor2.setPower(FOREARM_UPDOWN_POWER_FOR_MANNUAL);
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
        if (power < FOREARM_UPDOWN_POWER_FOR_MANNUAL) {
            power += ACCELERATION_FOR_FOREARM_UPDOWN_FOR_MANNUAL;
            if (power > FOREARM_UPDOWN_POWER_FOR_MANNUAL) {
                power = FOREARM_UPDOWN_POWER_FOR_MANNUAL;
            }
        }
        updateUpDownStatus(true,false,false,false);
        setModeForUpDown(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerForUpDown(power);
    }

    public void keepDowningEnc() {
        double power = Math.max(motor1.getPower(),motor2.getPower());
        if ((Math.abs(motor1.getTargetPosition() - motor1.getCurrentPosition()) < 100) ||
                (Math.abs(motor2.getTargetPosition() - motor2.getCurrentPosition()) < 100))   {
            motor1.setTargetPosition(motor1.getCurrentPosition() - 1000);
            motor2.setTargetPosition(motor2.getCurrentPosition() + 1000);
        }
        if (power < FOREARM_UPDOWN_POWER_FOR_MANNUAL) {
            power += ACCELERATION_FOR_FOREARM_UPDOWN_FOR_MANNUAL;
            if (power > FOREARM_UPDOWN_POWER_FOR_MANNUAL) {
                power = FOREARM_UPDOWN_POWER_FOR_MANNUAL;
            }
        }
        updateUpDownStatus(false,true,false,false);
        setModeForUpDown(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerForUpDown(power);
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
    }

    public void keepUpingEnc2() {
        double power = Math.max(motor1.getPower(),motor2.getPower());
        if (sensor1IsTouched()) {
            stopUpDownEnc2();
            return;
        }
        setTargetPositionForUpDown(motor1.getCurrentPosition(),motor2.getCurrentPosition());
        if ((Math.abs(motor1.getTargetPosition() - motor1.getCurrentPosition()) < 100) ||
                (Math.abs(motor2.getTargetPosition() - motor2.getCurrentPosition()) < 100))   {
            setTargetPositionForUpDown(motor1.getCurrentPosition() + 1000,motor2.getCurrentPosition() - 1000);
        }
        if (power < FOREARM_UPDOWN_POWER_FOR_MANNUAL) {
            power += ACCELERATION_FOR_FOREARM_UPDOWN_FOR_MANNUAL;
            if (power > FOREARM_UPDOWN_POWER_FOR_MANNUAL) {
                power = FOREARM_UPDOWN_POWER_FOR_MANNUAL;
            }
        }

        isUping = true;
        isDowning = false;

        setModeForUpDown(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerForUpDown(power);
    }

    public void keepDowningEnc2() {
        double power = Math.max(motor1.getPower(),motor2.getPower());

        setTargetPositionForUpDown(motor1.getCurrentPosition(),motor2.getCurrentPosition());

        if ((Math.abs(motor1.getTargetPosition() - motor1.getCurrentPosition()) < 100) ||
                (Math.abs(motor2.getTargetPosition() - motor2.getCurrentPosition()) < 100))   {
            setTargetPositionForUpDown(motor1.getCurrentPosition() - 1000, motor2.getCurrentPosition() + 1000);
        }

        if (power < FOREARM_UPDOWN_POWER_FOR_MANNUAL) {
            power += ACCELERATION_FOR_FOREARM_UPDOWN_FOR_MANNUAL;
            if (power > FOREARM_UPDOWN_POWER_FOR_MANNUAL) {
                power = FOREARM_UPDOWN_POWER_FOR_MANNUAL;
            }
        }
        isUping = false;
        isDowning = true;
        setModeForUpDown(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerForUpDown(power);
    }

    public void stopUpDownEnc2() {

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

        setModeForUpDown(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPositionForUpDown(motor1.getCurrentPosition(),motor2.getCurrentPosition());
        setZeroPowerBehaviorForUpDown(ZeroPowerBehavior.BRAKE);
        setPowerForUpDown(0.0);

        isUping = false;
        isDowning = false;

    }

/******************** End of Part II ************************************/





/******************** Start of Part III *********************************/
    /**
     * Function: Move the foreArm UP or DOWN automatically
     * <p>
     * power: is used for drive the motor
     * moveAngle: is the angle the ForeArm is going to move up/down automatically
     * - positive for moving up
     * - negative for moving down
     */
    public void moveUpDownAngleEnc(double moveAngle,int timeoutMS) {

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
        } else if (MOVE_COUNTS < 0 ) {
            isUping = false;
            isDowning = true;
        } else {
            isUping = false;
            isDowning = false;
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

            setModeForUpDown(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerForUpDown(power);

        }

        setModeForUpDown(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clearTargetPositionForUpDown();
        setZeroPowerBehaviorForUpDown(ZeroPowerBehavior.BRAKE);
        setPowerForUpDown(0);

        isUping = false;
        isDowning = false;

    }

    public void moveUpDownAngleEnc2(double moveAngle) {

        if ((isAutoUping) || (isAutoDowning)) {
            double power = Math.max(motor1.getPower(), motor2.getPower());
            if ((sensor1IsTouched()) || ((!motor1.isBusy())||(!motor2.isBusy()))) {
                power = 0;
            } else {
                if (Math.abs(AUTO_MOVE_COUNTS) >= (COUNTS_THRESHOLD_FOR_SLOWDOWN * 2 + 50)) {
                    if ((Math.abs(motor1.getTargetPosition() - motor1.getCurrentPosition()) > (Math.abs(AUTO_MOVE_COUNTS) - COUNTS_THRESHOLD_FOR_SLOWDOWN)) &&
                            (Math.abs(motor2.getTargetPosition() - motor2.getCurrentPosition()) > (Math.abs(AUTO_MOVE_COUNTS) - COUNTS_THRESHOLD_FOR_SLOWDOWN))) {
                        if (power < MAX_POPER_FOR_FOREARM_UPDOWN) {
                            power += ACCELERATION_FOR_FOREARM_UPDOWN;
                            if (power > MAX_POPER_FOR_FOREARM_UPDOWN) {
                                power = MAX_POPER_FOR_FOREARM_UPDOWN;
                            }
                        }
                    }
                    if ((Math.abs(motor1.getTargetPosition() - motor1.getCurrentPosition()) < COUNTS_THRESHOLD_FOR_SLOWDOWN) ||
                            (Math.abs(motor2.getTargetPosition() - motor2.getCurrentPosition()) < COUNTS_THRESHOLD_FOR_SLOWDOWN)) {
                        if (power > MIN_POWER_FOR_FOREARM_UPDOWN) {
                            power -= ACCELERATION_FOR_FOREARM_UPDOWN;
                            if (power < MIN_POWER_FOR_FOREARM_UPDOWN) {
                                power = MIN_POWER_FOR_FOREARM_UPDOWN;
                            }
                        }
                    }
                }
            }
            if (power == 0) {
                updateUpDownStatus(isUping(),isDowning(),false,false);
                setModeForUpDown(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                clearTargetPositionForUpDown();
                setZeroPowerBehaviorForUpDown(ZeroPowerBehavior.BRAKE);
                setPowerForUpDown(0);
                updateUpDownStatus(false, false, false, false);
                return;
            } else {
                setModeForUpDown(DcMotor.RunMode.RUN_TO_POSITION);
                setPowerForUpDown(power);
                return;
            }
        } else {
            ElapsedTime runtime = new ElapsedTime();
            setModeForUpDown(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int motor1Target, motor2Target;
            AUTO_MOVE_COUNTS = (int) ((moveAngle / 360) * COUNTS_PER_REV_FOR_UPDOWN * GEAR_REDUCTION_FOR_UPDOWN_MOTOR);
            motor1Target = motor1.getCurrentPosition() + AUTO_MOVE_COUNTS;
            motor2Target = motor2.getCurrentPosition() - AUTO_MOVE_COUNTS;
            setTargetPositionForUpDown(motor1Target, motor2Target);
            setModeForUpDown(DcMotor.RunMode.RUN_TO_POSITION);

            if (AUTO_MOVE_COUNTS > 0) {
                updateUpDownStatus(false, false, true, false);
            } else if (AUTO_MOVE_COUNTS < 0) {
                updateUpDownStatus(false, false, false, true);
            } else {
                updateUpDownStatus(false, false, false, false);
            }
            runtime.reset();
            setPowerForUpDown(MIN_POWER_FOR_FOREARM_UPDOWN);
        }
    }

/******************** End of Part III *********************************/


    public void setModeForUpDown(DcMotor.RunMode runMode) {
        motor1.setMode(runMode);
        motor2.setMode(runMode);
    }

    public void setPowerForUpDown(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void setZeroPowerBehaviorForUpDown(ZeroPowerBehavior zeroPowerBehaviorForUpDown) {
        motor1.setZeroPowerBehavior(zeroPowerBehaviorForUpDown);
        motor2.setZeroPowerBehavior(zeroPowerBehaviorForUpDown);
    }

    public void clearTargetPositionForUpDown() {
        motor1.setTargetPosition(motor1.getCurrentPosition());
        motor2.setTargetPosition(motor2.getCurrentPosition());
    }

    public void setDirectionDOwn() {
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setDirectionUp() {
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setTargetPositionForUpDown(int motor1Target,int motor2Target) {
        motor1.setTargetPosition(motor1Target);
        motor2.setTargetPosition(motor2Target);
    }

    public boolean sensor1IsTouched() {
        return !touchSensor1.getState();
    }

    public void updateUpDownStatus(boolean isUpping, boolean isDowning,boolean isAutoUping,boolean isAutoDowning) {
        this.isUping = isUpping;
        this.isDowning = isDowning;
        this.isAutoUping = isAutoUping;
        this.isAutoDowning = isAutoDowning;
    }

    public boolean isUping() { return isUping; }

    public boolean isDowning() { return isDowning; }

    public boolean isUpDownStopped() { return ((!isUping) && (!isDowning)); }

    public boolean isForwarding() { return isForwarding; }

    public boolean isBackwarding() { return isBackwarding; }

    public boolean isForthBackStopped() { return ((!isForwarding)&&(!isBackwarding)); }

    public double calPower(int currCount, int startCount, int endCount, double startPower, double endPower)  {
        if (startPower == endPower) {
            return startPower;
        }
        if (((startCount <= endCount) && (currCount >= endCount)) || ((startCount >= endCount)&&(currCount <= endCount))) {
            return endPower;
        }
        if (((startCount <= endCount) && (currCount <= startCount)) || ((startCount >= endCount)&&(currCount >= startCount))) {
            return startPower;
        }
        return (startPower + (Math.sin(((currCount-startCount)/(endCount-startCount))*Math.PI / 2))*(endPower - startPower));
    }

}


