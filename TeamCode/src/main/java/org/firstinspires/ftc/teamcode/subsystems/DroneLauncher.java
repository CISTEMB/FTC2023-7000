package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DroneLauncher extends SubsystemBase {

    private ServoEx servo;

    public DroneLauncher(HardwareMap hardwareMap){
        servo = new SimpleServo(hardwareMap, "droneLauncher", 0, 300);
        servo.setInverted(false);
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }
    public void shoot(){
        setAngle(240);
    }
    public void stay(){
        setAngle(0);
    }
    @Override
    public void periodic(){
    }

}
