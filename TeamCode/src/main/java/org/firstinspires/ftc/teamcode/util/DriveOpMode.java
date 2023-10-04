package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "TeleOp")

public class DriveOpMode extends CommandOpMode {

    private Drive drive;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap, telemetry);
        drive.setDefaultCommand(
                new DriveWithGamepadCommand(gamepad1, drive)
        );

    }
}
