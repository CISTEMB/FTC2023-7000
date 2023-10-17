package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakePivot extends SubsystemBase {

    private ServoEx servo;

    public IntakePivot(HardwareMap hardwareMap){
        servo = new SimpleServo(hardwareMap, "intakePivot", 0, 270);
        servo.setInverted(false);
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }

    @Override
    public void periodic(){

    }

}
