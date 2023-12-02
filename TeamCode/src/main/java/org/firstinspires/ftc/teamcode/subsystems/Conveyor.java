package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Conveyor extends SubsystemBase {

    private DcMotor conveyorMotor;

    public Conveyor(HardwareMap hardwareMap){
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");

        conveyorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        conveyorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void up(){
        conveyorMotor.setPower(1);
    }
    public void stop(){
        conveyorMotor.setPower(0);
    }
    public void down(){
        conveyorMotor.setPower(-1);
    }
}
