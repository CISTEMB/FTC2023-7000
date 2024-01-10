package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;

@Autonomous(name = "Auto-Park-Blue-Left")

public class AutoParkBlueLeft extends AutoOpModeBase {

    protected Trajectory scoreCenterBackdrop;


    public AutoParkBlueLeft(){
        super(false, false);
    }

    @Override
    public void initialize() {
        super.initialize();

        scoreCenterBackdrop = drive.trajectoryBuilder(new Pose2d(scoreCenter.end().getX(), scoreCenter.end().getY(), Math.toRadians(-90)))
                .back(34)
                .build();

        schedule(new SequentialCommandGroup(
               scorePurplePixle()
                //new TrajectoryFollowerCommand(drive, scoreCenterBackdrop)
        ));

    }
    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}