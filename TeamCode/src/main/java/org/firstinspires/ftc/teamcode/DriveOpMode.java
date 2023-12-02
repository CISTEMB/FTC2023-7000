package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClampAngleCommand;
import org.firstinspires.ftc.teamcode.commands.ConveyorCommand;
import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorRetractCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSpinComand;
import org.firstinspires.ftc.teamcode.commands.SetPixelDoohickeyAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Clamp;
import org.firstinspires.ftc.teamcode.subsystems.ClampPivot;
import org.firstinspires.ftc.teamcode.subsystems.Conveyor;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PixelDoohickey;

@TeleOp(name = "TeleOp")

public class DriveOpMode extends CommandOpMode {

    private Drive drive;
    private Elevator elevator;
    private Intake intake;
    private Conveyor conveyor;
    private PixelDoohickey pdivot;
    private ClampPivot pivot;
    private Clamp clamp;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap, telemetry);
        drive.setDefaultCommand(
                new DriveWithGamepadCommand(gamepad1, drive)
        );

        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        intake.setDefaultCommand(new RunCommand(intake::stop, intake));
        conveyor = new Conveyor(hardwareMap);
        conveyor.setDefaultCommand(new RunCommand(conveyor::up, conveyor));
        pdivot = new PixelDoohickey(hardwareMap);
        pdivot.setDefaultCommand(new RunCommand(pdivot::close, pdivot));
        pivot = new ClampPivot(hardwareMap, elevator);
        pivot.setDefaultCommand(new RunCommand(pivot::stow, pivot));
        clamp = new Clamp(hardwareMap);

        GamepadEx driver = new GamepadEx(gamepad1);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(new IntakeSpinComand(intake));
        driver.getGamepadButton(GamepadKeys.Button.BACK).whileHeld(new InstantCommand(pdivot::drop, pdivot));
        driver.getGamepadButton(GamepadKeys.Button.Y).whileHeld(new InstantCommand(conveyor::stop, conveyor));

        GamepadEx driver2 = new GamepadEx(gamepad2);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new ElevatorExtendCommand(elevator));
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new ElevatorRetractCommand(elevator));

        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new RunCommand(pivot::score, pivot));
        driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new RunCommand(clamp::close, clamp));
        driver2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new RunCommand(clamp::open, clamp));

        driver2.getGamepadButton(GamepadKeys.Button.BACK).whileHeld(new InstantCommand(conveyor::down, conveyor).alongWith(new InstantCommand(intake::eject, intake)));
    }
}