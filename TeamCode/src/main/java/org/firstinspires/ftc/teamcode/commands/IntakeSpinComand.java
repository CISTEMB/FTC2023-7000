package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeSpinComand extends CommandBase {

    private Intake intake;

    public IntakeSpinComand(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.rotate();
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

}
