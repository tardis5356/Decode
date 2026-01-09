package org.firstinspires.ftc.teamcode.Zenith.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake.Direction;

public class IntakeToggleCommand extends CommandBase {
    Intake intake;
    Direction commandedDirection;
    public IntakeToggleCommand(Intake intake, Direction commandedDirection){
        this.intake = intake;
        this.commandedDirection = commandedDirection;
    }

    @Override
    public void execute(){
        switch(commandedDirection){
            case IN:
                if(intake.currentDirection == Direction.IN){
                    intake.stop();
                }
                else if(intake.currentDirection == Direction.OFF){
                    intake.in();
                }
                else {
                    new SequentialCommandGroup(
                            new InstantCommand(intake::stop),
                            new WaitCommand(200),
                            new InstantCommand(intake::in)
                    ).schedule();
                }
                break;
            case OUT:
                if(intake.currentDirection == Direction.IN){
                    new SequentialCommandGroup(
                            new InstantCommand(intake::stop),
                            new WaitCommand(200),
                            new InstantCommand(intake::out)
                    ).schedule();

                }
                else if(intake.currentDirection == Direction.OFF){
                    intake.out();
                }
                else {
                    intake.stop();
                }
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
