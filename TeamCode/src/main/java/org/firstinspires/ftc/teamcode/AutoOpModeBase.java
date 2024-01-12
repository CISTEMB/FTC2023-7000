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
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Clamp;
import org.firstinspires.ftc.teamcode.subsystems.Conveyor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PixelPlacer;

import java.util.HashMap;

public abstract class AutoOpModeBase extends CommandOpMode {
    protected MecanumDriveSubsystem drive;
    protected Clamp clamp;
    protected Intake intake;
    protected Conveyor conveyor;
    protected DistanceSensor rightDistanceSensor;
    protected DistanceSensor leftDistanceSensor;
    protected PixelPlacer pixelPlacer;
    protected final boolean redCorner;
    protected final boolean redAlliance;
    protected AutoOpModeBase(boolean redCorner, boolean redAlliance){
        this.redCorner = redCorner;
        this.redAlliance =redAlliance;
    }

    protected Trajectory scoreLeft;
    protected Trajectory scoreLeft2;
    protected Trajectory scoreleft1andAHalf;
    protected Trajectory scoreCenter;
    protected Trajectory scoreCenter2;
    protected Trajectory scoreRight;
    protected Trajectory scoreRight2;
    protected Trajectory scoreRight3;
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

        pixelPlacer = new PixelPlacer(hardwareMap);

        Pose2d startingPosition = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startingPosition);

        toRigging = drive.trajectoryBuilder(startingPosition)
                .forward(8)
                .splineTo(new Vector2d(26, 1.5 * invertStarting), Math.toRadians(0))
                .forward(3)
                .build();

        if (redCorner) {
            scoreLeft = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(90 * invertTurn)))
                    .back(9)
                    .build();
        } else {
            scoreLeft = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(90 * invertTurn)))
                    .back(6.5)
                    .build();
        }

        scoreleft1andAHalf = drive.trajectoryBuilder(scoreLeft.end())
                .back(2)
                .build();

        if (redCorner) {
            scoreLeft2 = drive.trajectoryBuilder(scoreleft1andAHalf.end())
                    .forward(12)
                    .build();
        }

        else{
            scoreLeft2 = drive.trajectoryBuilder(scoreLeft.end())
                    .strafeRight(12)
                    .build();
        }

        if (redCorner) {
            scoreRight = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(-90 * invertTurn)))
                    .back(7.5)
                    .build();

        } else {
            scoreRight = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(-90 * invertTurn)))
                    .back(9)
                    .build();

        }
        scoreRight2 = drive.trajectoryBuilder(scoreRight.end())
                .back(2)
                .build();
        scoreRight3 = drive.trajectoryBuilder(scoreRight2.end())
                .forward(24)
                .build();
        scoreCenter = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(90 * invertTurn)))
                .strafeLeft(8)
                .build();
        scoreCenter2 = drive.trajectoryBuilder(scoreCenter.end())
                .strafeRight(12)
                .build();

    }

    public Command scorePurplePixle(){

        return new SequentialCommandGroup(
                new TrajectoryFollowerCommand(drive, toRigging),
                new SelectCommand(
                        new HashMap<Object, Command>(){{
                            put(0, new SequentialCommandGroup(
                                    new TurnCommand(drive, Math.toRadians(-95)),
                                    new TrajectoryFollowerCommand(drive, scoreLeft),
                                    new RunCommand(pixelPlacer::dropPixel, pixelPlacer).withTimeout(1000),
                                    new ConditionalCommand(new TrajectoryFollowerCommand(drive, scoreleft1andAHalf), new PrintCommand("NA"), ()->redCorner),
                                    new TrajectoryFollowerCommand(drive, scoreLeft2)

                            ) );
                            put(1, new SequentialCommandGroup(
                                    new TurnCommand(drive, Math.toRadians(-95)),
                                    new TrajectoryFollowerCommand(drive, scoreCenter),
                                    new RunCommand(pixelPlacer::dropPixel, pixelPlacer).withTimeout(1000),
                                    new TrajectoryFollowerCommand(drive, scoreCenter2)
                            ));
                            put(2, new SequentialCommandGroup(
                                    new TurnCommand(drive, Math.toRadians(95)),
                                    new TrajectoryFollowerCommand(drive, scoreRight),
                                    new RunCommand(pixelPlacer::dropPixel, pixelPlacer).withTimeout(1000),
                                    new TrajectoryFollowerCommand(drive, scoreRight2),
                                    new TrajectoryFollowerCommand(drive, scoreRight3)
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
                )
        );
    }
}
