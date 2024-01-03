package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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

public abstract class AutoOpModeBase extends CommandOpMode {
    protected MecanumDriveSubsystem drive;
    protected Clamp clamp;
    protected Intake intake;
    protected Conveyor conveyor;
    protected DistanceSensor rightDistanceSensor;
    protected DistanceSensor leftDistanceSensor;
    protected final boolean redCorner;
    protected final boolean redAlliance;
    protected AutoOpModeBase(boolean redCorner, boolean redAlliance){
        this.redCorner = redCorner;
        this.redAlliance =redAlliance;
    }

    protected Trajectory scoreLeft;
    protected Trajectory scoreCenter;
    protected Trajectory scoreRight;
    protected Trajectory toRigging;

    @Override
    public void initialize() {
        double invertTurn = 1;
        if(!redAlliance){
            invertTurn = -1;
        }
        double invertStarting = 1;
        if(!redCorner){
            invertStarting = -1;
        }
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        clamp = new Clamp(hardwareMap);
        clamp.close();

        intake = new Intake(hardwareMap);
        intake.setDefaultCommand(new RunCommand(intake::stop, intake));

        conveyor = new Conveyor(hardwareMap);
        conveyor.setDefaultCommand(new RunCommand(conveyor::stop, conveyor));

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        Pose2d startingPosition = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startingPosition);

        toRigging = drive.trajectoryBuilder(startingPosition)
                .forward(8)
                .splineTo(new Vector2d(26, 1.5 * invertStarting), Math.toRadians(0))
                .forward(3)
                .build();

        scoreLeft = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(90 * invertTurn)))
                .forward(2)
                .build();
        scoreRight = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(90 * invertTurn)))
                .forward(-1.5)
                .build();
        scoreCenter = drive.trajectoryBuilder(toRigging.end())
                .back(1)
                .build();
    }

    public Command scorePurplePixle(){
        double invertTurn = 1;
        if(!redAlliance){
            invertTurn = -1;
        }

        double finalInvertTurn = invertTurn;
        return new SequentialCommandGroup(
                new TrajectoryFollowerCommand(drive, toRigging),
                new SelectCommand(
                        new HashMap<Object, Command>(){{
                            put(0, new SequentialCommandGroup(
                                    new TurnCommand(drive, Math.toRadians(95 * finalInvertTurn)),
                                    new TrajectoryFollowerCommand(drive, scoreLeft)
                            ) );
                            put(1, new TrajectoryFollowerCommand(drive, scoreCenter));
                            put(2, new SequentialCommandGroup(
                                    new TurnCommand(drive, Math.toRadians(95 * finalInvertTurn)),
                                    new TrajectoryFollowerCommand(drive, scoreRight)
                            ));
                        }},
                        ()->{
                            if(leftDistanceSensor.getDistance(DistanceUnit.INCH)<6){
                                return 0;
                            } else if (rightDistanceSensor.getDistance(DistanceUnit.INCH)<6) {
                                return 2;
                            }else{
                                return 1;
                            }
                        }
                ),
                new ParallelCommandGroup(
                        new RunCommand(intake::eject),
                        new RunCommand(conveyor::down)
                ).withTimeout(2000),
                new InstantCommand(intake::stop),
                new InstantCommand(conveyor::stop)

        );
    }
}
