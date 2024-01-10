package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PixelPlacer extends SubsystemBase {

    private ServoEx servo;

    public PixelPlacer(HardwareMap hardwareMap){
        servo = new SimpleServo(hardwareMap, "pixelPlacer", 0, 300);
        servo.setInverted(false);
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }
    public void dropPixel(){
        setAngle(180);
    }

    @Override
    public void periodic(){
    }

}
