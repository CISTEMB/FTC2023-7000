package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator extends SubsystemBase {

    private Telemetry telemetry;
    private DcMotorEx motor;
    private TouchSensor bottomLimit;

    public Elevator(HardwareMap hm, Telemetry tm){
        motor = hm.get(DcMotorEx.class, "elevatorMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLimit = hm.get(TouchSensor.class, "bl");
        telemetry = tm;
    }

    public void extend(){
        telemetry.addData("ElevatorState", "extend");

        if (isExtended()){
            brake();
        }else {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(0.2);
        }
    }

    public void retract(){
        telemetry.addData("ElevatorState", "retract");

        if (isRetracted()){
            brake();
        }else {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(-0.2);
        }
    }

    public void brake(){
        telemetry.addData("ElevatorState", "stop");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    public double getDistance(){
        return motor.getCurrentPosition();
    }

    public boolean isRetracted(){
        return bottomLimit.isPressed();
    }

    public boolean isExtended(){
        return getDistance() >= 2000;
    }
}
