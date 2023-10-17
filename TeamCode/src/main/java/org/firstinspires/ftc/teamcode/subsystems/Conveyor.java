package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Conveyor extends SubsystemBase {

    private DcMotor conveyorMotor;

    public Conveyor(HardwareMap hardwareMap){

        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");

        conveyorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        conveyorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}
