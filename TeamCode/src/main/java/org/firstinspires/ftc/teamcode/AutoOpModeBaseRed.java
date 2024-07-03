package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.ClampPivot;
import org.firstinspires.ftc.teamcode.subsystems.Conveyor;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LiftPivot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PixelPlacer;

import java.util.HashMap;

public abstract class AutoOpModeBaseRed extends CommandOpMode {
    protected MecanumDriveSubsystem drive;
    protected Clamp clamp;
    protected Intake intake;
    protected Conveyor conveyor;
    protected DistanceSensor rightDistanceSensor;
    protected DistanceSensor leftDistanceSensor;
    protected PixelPlacer pixelPlacer;
    protected Elevator elevator;
    protected LiftPivot liftPivot;
    protected ClampPivot clampPivot;
    protected final boolean redCorner;
    protected final boolean redAlliance;

    protected int headSpot;

    protected AutoOpModeBaseRed(boolean redCorner, boolean redAlliance){
        this.redCorner = redCorner;
        this.redAlliance =redAlliance;
    }

    protected Trajectory scoreLeft;
    protected Trajectory scoreLeft2;
    protected Trajectory scoreleft1andAHalf;
    protected Trajectory scoreCenter;
    protected Trajectory scoreCenter2;
    protected Trajectory scoreCenter3;
    protected Trajectory scoreCenter4;
    protected Trajectory scoreRight;
    protected Trajectory scoreRight2;
    protected Trajectory scoreRight3;
    protected Trajectory scoreRight4;
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

        elevator = new Elevator(hardwareMap, telemetry);

        liftPivot = new LiftPivot(hardwareMap, elevator);

        clampPivot = new ClampPivot(hardwareMap, elevator);

        Pose2d startingPosition = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startingPosition);

        toRigging = drive.trajectoryBuilder(startingPosition)
                .forward(8)
                .splineTo(new Vector2d(26, 1.5 * invertStarting), Math.toRadians(0))
                .forward(3)
                .build();

        if (redCorner) {
            scoreLeft = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(-90)))
                    .back(9)
                    .build();
        } else {
            scoreLeft = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(-90)))
                    .back(6.5)
                    .build();
        }

        scoreleft1andAHalf = drive.trajectoryBuilder(scoreLeft.end())
                .back(2)
                .build();

        if (redCorner) {
            scoreLeft2 = drive.trajectoryBuilder(scoreleft1andAHalf.end())
                    .forward(20)
                    .build();
        }

        else{
            scoreLeft2 = drive.trajectoryBuilder(scoreLeft.end())
                    .strafeRight(12)
                    .build();
        }

        if (redCorner) {
            scoreRight = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(90)))
                    .back(7.5)
                    .build();

        } else {
            scoreRight = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(90)))
                    .back(9)
                    .build();

        }
        scoreRight2 = drive.trajectoryBuilder(scoreRight.end())
                .back(2)
                .build();
        scoreRight3 = drive.trajectoryBuilder(scoreRight2.end())
                .forward(9.5)
                .build();
        scoreRight4 = drive.trajectoryBuilder(scoreRight3.end())
                .strafeLeft(25)
                .build();
        scoreCenter = drive.trajectoryBuilder(new Pose2d(toRigging.end().getX(), toRigging.end().getY(), Math.toRadians(90 * invertTurn)))
                .strafeRight(11)
                .build();
        scoreCenter2 = drive.trajectoryBuilder(scoreCenter.end())
                .back(2)
                .build();
        scoreCenter3 = drive.trajectoryBuilder(scoreCenter2.end())
                .forward(1)
                .build();
        scoreCenter4 = drive.trajectoryBuilder(scoreCenter3.end())
                .strafeLeft(16)
                .build();

    }

    public Command scorePurplePixle(){

        return new SequentialCommandGroup(
                new TrajectoryFollowerCommand(drive, toRigging),
                new SelectCommand(
                        new HashMap<Object, Command>(){{
                            put(0, new SequentialCommandGroup(
                                    new InstantCommand(()-> headSpot = 0),
                                    new TurnCommand(drive, Math.toRadians(-95)),
                                    new TrajectoryFollowerCommand(drive, scoreLeft),
                                    new RunCommand(pixelPlacer::dropPixel, pixelPlacer).withTimeout(1000),
                                    new ConditionalCommand(new TrajectoryFollowerCommand(drive, scoreleft1andAHalf), new PrintCommand("NA"), ()->redCorner),
                                    new TrajectoryFollowerCommand(drive, scoreLeft2)

                            ) );
                            put(1, new SequentialCommandGroup(
                                    new InstantCommand(()-> headSpot = 1),
                                    new TurnCommand(drive, Math.toRadians(95)),
                                    new TrajectoryFollowerCommand(drive, scoreCenter),
                                    new RunCommand(pixelPlacer::dropPixel, pixelPlacer).withTimeout(1000),
                                    new TrajectoryFollowerCommand(drive, scoreCenter2),
                                    new TrajectoryFollowerCommand(drive, scoreCenter3),
                                    new TrajectoryFollowerCommand(drive, scoreCenter4)

                            ));
                            put(2, new SequentialCommandGroup(
                                    new InstantCommand(()-> headSpot = 2),
                                    new TurnCommand(drive, Math.toRadians(95)),
                                    new TrajectoryFollowerCommand(drive, scoreRight),
                                    new RunCommand(pixelPlacer::dropPixel, pixelPlacer).withTimeout(1000),
                                    new TrajectoryFollowerCommand(drive, scoreRight2),
                                    new TrajectoryFollowerCommand(drive, scoreRight3),
                                    new TrajectoryFollowerCommand(drive, scoreRight4)
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
