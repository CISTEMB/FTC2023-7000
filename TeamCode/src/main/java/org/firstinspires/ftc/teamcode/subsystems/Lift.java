package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends SubsystemBase {

    private Telemetry telemetry;
    private DcMotorEx motor;

    public Lift(HardwareMap hm, Telemetry tm){
        motor = hm.get(DcMotorEx.class, "liftMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry = tm;
    }

    public void up(){
        telemetry.addData("LiftState", "up");

//        if (isExtended()){
//            brake();
//        }else {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(1);
        }
//    }

    public void retract(){
        telemetry.addData("LiftState", "retract");

//        if (isRetracted()){
//            brake();
//        }else {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(-1);
        }
//    }

    public void brake(){
        telemetry.addData("LiftState", "stop");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    public double getDistance(){
        return motor.getCurrentPosition();
    }

    public boolean isRetracted(){
        return getDistance() <= 0;
    }

    public boolean isExtended(){
        return getDistance() >= 500;
    }

    @Override
    public void periodic() {
        telemetry.addData("LiftIsRetracted", isRetracted());
        telemetry.addData("LiftIsExtended", isExtended());
        telemetry.addData("LiftDistance", getDistance());
        telemetry.update();

//        if (isRetracted()) {
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
    }
}
