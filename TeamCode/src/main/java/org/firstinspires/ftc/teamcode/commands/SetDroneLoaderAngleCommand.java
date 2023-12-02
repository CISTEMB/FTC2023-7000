package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DroneLoader;

public class SetDroneLoaderAngleCommand extends CommandBase {

    private DroneLoader pivot;
    private double angle;

    public SetDroneLoaderAngleCommand(DroneLoader pivot, double angle){
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