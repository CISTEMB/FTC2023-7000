package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClampPivot extends SubsystemBase {

    private ServoEx servo;
    private Elevator elevator;

    public ClampPivot(HardwareMap hardwareMap, Elevator elevator){
        servo = new SimpleServo(hardwareMap, "clampPivot", 0, 300);
        servo.setInverted(false);
        this.elevator = elevator;
    }

    public void setAngle(double angle){
        servo.turnToAngle(angle);
    }
    public void stow(){
        if (elevator.getDistance()<700) {
            return;
        }
        setAngle(95);
    }
    public void score(){
        if (elevator.getDistance()< 700) {
            return;
        }
        setAngle(200);
    }

    @Override
    public void periodic(){
    }

}
