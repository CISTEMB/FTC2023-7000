package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

public class FlywheelSpinCommand extends CommandBase {
    private Flywheel flywheel;

    public FlywheelSpinCommand(Flywheel flywheel){
        this.flywheel = flywheel;
        addRequirements(flywheel);
    }

    @Override
    public void execute(){
        flywheel.rotate();
    }

    @Override
    public void end(boolean interrupted){
        flywheel.stop();
    }

}
