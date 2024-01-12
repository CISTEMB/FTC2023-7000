package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Clamp;
import org.firstinspires.ftc.teamcode.subsystems.Conveyor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.HashMap;

@Autonomous(name = "Auto-Park-Red-Left")

public class AutoParkRedLeft extends AutoOpModeBaseRed {

    public AutoParkRedLeft(){
        super(false, true);
    }
    protected Trajectory headSpot0;
    protected Trajectory headSpot1;
    protected Trajectory headSpot2;

    @Override
    public void initialize() {
        super.initialize();

        headSpot0 = drive.trajectoryBuilder(scoreLeft2.end())
                .splineTo(new Vector2d(5, 0), Math.toRadians(-90))
                .splineTo(new Vector2d(3,-90),Math.toRadians(-90))
                .build();
        headSpot1 = drive.trajectoryBuilder(scoreCenter4.end())
            .splineTo(new Vector2d(5, 0), Math.toRadians(-90))
            .splineTo(new Vector2d(3,-90),Math.toRadians(-90))
            .build();
        headSpot2 = drive.trajectoryBuilder(scoreRight4.end())
                .splineTo(new Vector2d(5, 0), Math.toRadians(-90))
                .splineTo(new Vector2d(3,-90),Math.toRadians(-90))
                .build();


        schedule(new SequentialCommandGroup(
               scorePurplePixle(),
                new SelectCommand(
                        new HashMap<Object, Command>(){{
                            put(0, new SequentialCommandGroup(
                                    new TrajectoryFollowerCommand(drive, headSpot0)
                            ) );
                            put(1, new SequentialCommandGroup(
                                    new TrajectoryFollowerCommand(drive, headSpot1)

                            ));
                            put(2, new SequentialCommandGroup(
                                    new TrajectoryFollowerCommand(drive, headSpot2)
                            ));
                        }},
                        ()-> headSpot
                )

        ));

    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}