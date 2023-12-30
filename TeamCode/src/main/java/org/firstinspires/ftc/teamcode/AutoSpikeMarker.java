package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Clamp;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@Autonomous(name = "Auto-Spike")

public class AutoSpikeMarker extends CommandOpMode {

    private Drive drive;
    private Clamp clamp;
    @Override
    public void initialize() {
        drive = new Drive(hardwareMap, telemetry);
        schedule(
                new DriveForwardCommand(telemetry, drive, 57, 0.5)
        );
        clamp = new Clamp(hardwareMap);
        clamp.close();
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}