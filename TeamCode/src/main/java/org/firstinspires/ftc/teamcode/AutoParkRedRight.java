package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.ElevatorExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorRetractCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Clamp;
import org.firstinspires.ftc.teamcode.subsystems.ClampPivot;
import org.firstinspires.ftc.teamcode.subsystems.Conveyor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.HashMap;

@Autonomous(name = "Auto-Park-Red-Right")

public class AutoParkRedRight extends AutoOpModeBaseRed {

    public AutoParkRedRight(){
        super(true, true);
    }
    protected Trajectory headSpot0;
    protected Trajectory headSpot1;
    protected Trajectory headSpot2;
    @Override
    public void initialize(){
        super.initialize();

        headSpot0 = drive.trajectoryBuilder(scoreLeft2.end(), true)
                .splineTo(new Vector2d(36, -40), Math.toRadians(-90))
                .build();
        headSpot1 = drive.trajectoryBuilder(scoreCenter4.end(), true)
                .splineTo(new Vector2d(28, -40), Math.toRadians(-90))
                .build();
        headSpot2 = drive.trajectoryBuilder(scoreRight4.end(), true)
                .splineTo(new Vector2d(21, -40), Math.toRadians(-90))
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
                ),
                new ParallelRaceGroup(
                    new RunCommand(liftPivot::up, liftPivot),
                    new SequentialCommandGroup(
                        new ElevatorExtendCommand(elevator).interruptOn(
                            ()-> (elevator.getDistance()> 800)
                        ),
                        new RunCommand(clampPivot::score, clampPivot).withTimeout(1000),
                        new ElevatorRetractCommand(elevator).interruptOn(
                                ()-> (elevator.getDistance()< 500)
                        ),
                        new ParallelRaceGroup(
                                new RunCommand(clampPivot::score, clampPivot),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new RunCommand(
                                                clamp::open,clamp
                                        ),
                                        new WaitCommand(1000)
                                )

                        )
                    )
                )

        ));
    }
    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}