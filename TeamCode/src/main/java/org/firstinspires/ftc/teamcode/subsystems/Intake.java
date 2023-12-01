package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake extends SubsystemBase {
    private DcMotor motor;

    public Intake(HardwareMap hm){

        motor = hm.get(DcMotorEx.class, "intakeMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void rotate(){
        //telemetry.addData("IntakeState", "spinning");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0.5);
    }

    public void stop(){
        //telemetry.addData("IntakeState", "stop");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }
}
