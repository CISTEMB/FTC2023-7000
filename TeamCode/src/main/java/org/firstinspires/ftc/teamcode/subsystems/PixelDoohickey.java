package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PixelDoohickey extends SubsystemBase {

    private ServoEx servo;

    public PixelDoohickey(HardwareMap hardwareMap){
        servo = new SimpleServo(hardwareMap, "pixelDoohickey", 0, 300);
        servo.setInverted(false);
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }
    public void drop(){
        setAngle(90);
    }
    public void close(){
        setAngle(0);
    }

    @Override
    public void periodic(){
    }

}
