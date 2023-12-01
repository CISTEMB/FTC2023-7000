package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlywheelLoader extends SubsystemBase {

    private ServoEx servo;

    public FlywheelLoader(HardwareMap hardwareMap){
        servo = new SimpleServo(hardwareMap, "loader", 0, 270);
        servo.setInverted(false);
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }

    @Override
    public void periodic(){

    }

}