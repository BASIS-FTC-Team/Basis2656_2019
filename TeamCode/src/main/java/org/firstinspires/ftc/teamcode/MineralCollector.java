package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import static org.firstinspires.ftc.teamcode.Hardware2019.*;
import static org.firstinspires.ftc.teamcode.Parameters.*;

public class MineralCollector {

    private boolean isWipingOut = false;
    private boolean isWipingIn = false;
    private boolean holderIsClosed = true;


    public void init() {
        // For wiping
        servo1.setPower(0);
        isWipingIn = false;
        isWipingOut = false;

        // For holder open/close
        servo2.setPosition(HOLDER_CLOSED_POS);
        holderIsClosed = true;

    }

    public void wipeIn() {
        servo1.setDirection(CRServo.Direction.FORWARD);
        servo1.setPower(WIPE_ROTATION_SPEED);
        isWipingIn = true;
        isWipingOut = false;

    }

    public void wipeOut() {
        servo1.setDirection(CRServo.Direction.REVERSE);
        servo1.setPower(WIPE_ROTATION_SPEED);
        isWipingIn = false;
        isWipingOut = true;
    }

    public void wipeStop() {
        servo1.setPower(0);
        isWipingIn = false;
        isWipingOut = false;
    }


    public void openHolder(){
        servo2.setPosition(HOLDER_OPEN_POS);
        holderIsClosed = false;
    }

    public void closeHolder() {
        servo2.setPosition(HOLDER_CLOSED_POS);
        holderIsClosed = true;
    }

    public void dropTeamMarker() {
        servo2.setPosition(DROP_TEAMARKER_POS);
    }

    public boolean isStopped() {
        return ((!isWipingIn) && (!isWipingOut));
    }

    public boolean isWipingOut() {
        return isWipingOut;
    }

    public boolean isWipingIn() {
        return isWipingIn;
    }

    public boolean holderIsClosed() {
        return holderIsClosed;
    }

    public boolean holderIsOpen() {
        return !holderIsClosed;
    }
}
