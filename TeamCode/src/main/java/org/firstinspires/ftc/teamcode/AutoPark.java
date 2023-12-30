package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Clamp;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Autonomous(name = "Auto-Park")

public class AutoPark extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private Clamp clamp;
    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        clamp = new Clamp(hardwareMap);
        clamp.close();

        Pose2d startingPosition =new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startingPosition);

        Trajectory toRigging = drive.trajectoryBuilder(startingPosition)
                .forward(8)
                .splineTo(new Vector2d(26, 1.5), Math.toRadians(0))
                .forward(3)
                .build();

        schedule(new SequentialCommandGroup(
                new TrajectoryFollowerCommand(drive, toRigging)




        ));

    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}