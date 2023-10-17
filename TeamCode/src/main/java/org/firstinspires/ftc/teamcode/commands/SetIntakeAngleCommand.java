package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakePivot;

public class SetIntakeAngleCommand extends CommandBase {

    private IntakePivot pivot;
    private double angle;

    public SetIntakeAngleCommand(IntakePivot pivot, double angle){
        this.pivot = pivot;
        this.angle = angle;
        addRequirements(pivot);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        pivot.setAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
