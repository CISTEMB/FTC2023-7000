package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftExtendCommand extends CommandBase {

    private Lift lift;

    public LiftExtendCommand(Lift lift){
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.up();
    }

    @Override
    public void end(boolean interrupted) {
        lift.brake();
    }
}
