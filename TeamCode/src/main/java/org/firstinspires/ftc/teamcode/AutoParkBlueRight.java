package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto-Park-Blue-Right")

public class AutoParkBlueRight extends AutoOpModeBase {

    public AutoParkBlueRight(){
        super(true, false);
    }
    @Override
    public void initialize(){
        super.initialize();
        schedule(new SequentialCommandGroup(
                scorePurplePixle()
        ));
    }
    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}