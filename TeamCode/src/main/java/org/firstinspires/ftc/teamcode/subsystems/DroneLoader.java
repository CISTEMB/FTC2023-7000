package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DroneLoader extends SubsystemBase {

    private ServoEx servo;

    public DroneLoader(HardwareMap hardwareMap){
        servo = new SimpleServo(hardwareMap, "loader", 0, 300);
        servo.setInverted(false);
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }

    @Override
    public void periodic(){

    }

}