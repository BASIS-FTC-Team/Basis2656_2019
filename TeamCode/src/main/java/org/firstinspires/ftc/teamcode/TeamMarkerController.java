package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Hardware2019.servo3;
import static org.firstinspires.ftc.teamcode.Parameters.TM_PUSHOFF_POS;
import static org.firstinspires.ftc.teamcode.Parameters.TM_STAYON_POS;

public class TeamMarkerController {

    private Servo teamMarkerServo;

    public TeamMarkerController() {
        teamMarkerServo = servo3;
    }

    public void stayOn() {
        teamMarkerServo.setPosition(TM_STAYON_POS);
    }

    public void pushOff() {
        teamMarkerServo.setPosition(TM_PUSHOFF_POS);
    }

    public void setPostion( double postion) { teamMarkerServo.setPosition(postion);}

}
