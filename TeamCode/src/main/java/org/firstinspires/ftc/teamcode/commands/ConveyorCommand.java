package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Conveyor;

public class ConveyorCommand extends CommandBase {

    private Conveyor conveyor;

    public ConveyorCommand(Conveyor conveyor){
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void execute(){
        conveyor.up();
    }

    @Override
    public void end(boolean interrupted){
        conveyor.stop();
    }

}
