package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Config;

public class MineralCollector {
    private CRServo servo1;  // Rotate to wipe in
    private Servo servo2;  // Turn over the holder

    private double HOLDER_OPEN = 0.65;
    private double HOLDER_CLOSED = 0.23;

    private double WIPE_SPEED = 0.99;
    private boolean isRunning = false;
    private boolean isWipeOut = false;



    HardwareMap hwMap = null;
    public ElapsedTime time = new ElapsedTime();

    public void init(HardwareMap Map, Config config) {

        HOLDER_CLOSED = config.getDouble("holder_closed_pos", 0.23);
        HOLDER_OPEN = config.getDouble("holder_open_pos", 0.65);
        WIPE_SPEED = config.getDouble("wipe_rotation_speed", 0.99);

        hwMap = Map;
        servo1 = hwMap.crservo.get("wipe_servo");
        servo2 = hwMap.get(Servo.class, "turn_servo");

        servo1.setPower(0);
        servo2.setPosition(HOLDER_CLOSED);

    }

    public void wipeIn() {
        servo1.setDirection(CRServo.Direction.FORWARD);
        servo1.setPower(WIPE_SPEED);
        isWipeOut = false;
        isRunning = true;
    }

    public void wipeOut() {
        servo1.setDirection(CRServo.Direction.REVERSE);
        servo1.setPower(WIPE_SPEED);
        isWipeOut = true;
        isRunning = true;
    }

    public void wipeStop() {
        servo1.setPower(0);
    }

    public void wipePause() {
        isRunning = isRunning?false:true;
        if(isRunning){
            if(isWipeOut) wipeOut();
            else wipeIn();
        }
        else wipeStop();
    }

    public void wipeInOut(float power) {
        servo1.setPower(Range.clip(power,-1.0,1.0));
    }

    public void openHolder(){
        servo2.setPosition(HOLDER_OPEN);
    }

    public void closeHolder() {
        servo2.setPosition(HOLDER_CLOSED);
    }

}
