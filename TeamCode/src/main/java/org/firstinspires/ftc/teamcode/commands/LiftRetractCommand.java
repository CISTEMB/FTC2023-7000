package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftRetractCommand extends CommandBase {

    private Lift lift;

    public LiftRetractCommand(Lift lift){
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.retract();
    }

    @Override
    public void end(boolean interrupted) {
        lift.brake();
    }
}
