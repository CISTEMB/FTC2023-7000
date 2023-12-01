package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Flywheel extends SubsystemBase {
    private DcMotor motor;

    public Flywheel(HardwareMap hm, Telemetry tm){
        motor = hm.get(DcMotorEx.class, "flywheel");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void rotate(){
        telemetry.addData("FlywheelState", "spinning");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(1);
    }
    public void stop(){
        telemetry.addData("FlywheelState", "stop");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }
}