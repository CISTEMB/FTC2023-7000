package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Clamp extends SubsystemBase {

    private ServoEx servo;

    public Clamp(HardwareMap hardwareMap){
        servo = new SimpleServo(hardwareMap, "clamp", 0, 300);
        servo.setInverted(false);
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }
    public void open(){
        setAngle(200);
    }
    public void close(){
        setAngle(65);
    }

    @Override
    public void periodic(){
    }

}
