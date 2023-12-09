package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftPivot extends SubsystemBase {

    private ServoEx servo;
    private Elevator elevator;

    public LiftPivot(HardwareMap hardwareMap, Elevator elevator){
        servo = new SimpleServo(hardwareMap, "liftPivot", 0, 300);
        servo.setInverted(false);
        this.elevator = elevator;
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }
    public void down(){
            setAngle(0);
    }
    public void up(){
        setAngle(60);
    }

    @Override
    public void periodic(){
    }

}
