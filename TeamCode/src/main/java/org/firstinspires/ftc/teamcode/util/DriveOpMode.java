package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorRetractCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSpinComand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "TeleOp")

public class DriveOpMode extends CommandOpMode {

    private Drive drive;
    private Elevator elevator;
    private Intake intake;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap, telemetry);
        drive.setDefaultCommand(
                new DriveWithGamepadCommand(gamepad1, drive)
        );

        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        GamepadEx driver = new GamepadEx(gamepad1);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new ElevatorExtendCommand(elevator));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new ElevatorRetractCommand(elevator));
        driver.getGamepadButton(GamepadKeys.Button.A).whileHeld(new IntakeSpinComand(intake));

    }
}
