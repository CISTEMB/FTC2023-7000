package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;

import java.util.HashMap;

@Autonomous(name = "Auto-Park-Red-Left-LeftOfBack")

public class AutoParkRedLeft_LeftOfBack extends AutoOpModeBaseRed {

    public AutoParkRedLeft_LeftOfBack(){
        super(false, true);
    }
    protected Trajectory headSpot0;
    protected Trajectory headSpot1;
    protected Trajectory headSpot2;

    protected Trajectory strafe;

    @Override
    public void initialize() {
        super.initialize();

        headSpot0 = drive.trajectoryBuilder(scoreLeft2.end())
                .splineTo(new Vector2d(5, 0), Math.toRadians(-90))
                .splineTo(new Vector2d(3,-26.5),Math.toRadians(-90))
                .build();
        headSpot1 = drive.trajectoryBuilder(scoreCenter4.end())
                .splineTo(new Vector2d(5, 0), Math.toRadians(-90))
                .splineTo(new Vector2d(3,-26.5),Math.toRadians(-90))
                .build();
        headSpot2 = drive.trajectoryBuilder(scoreRight4.end())
                .splineTo(new Vector2d(5, 0), Math.toRadians(-90))
                .splineTo(new Vector2d(3,-26.5),Math.toRadians(-90))
                .build();

        strafe = drive.trajectoryBuilder(headSpot0.end())
                .strafeLeft(42)
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
                new TrajectoryFollowerCommand(drive, strafe)


        ));

    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}