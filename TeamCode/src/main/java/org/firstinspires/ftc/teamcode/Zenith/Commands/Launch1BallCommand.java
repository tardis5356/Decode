package org.firstinspires.ftc.teamcode.Zenith.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;

public class Launch1BallCommand extends CommandBase {
    private Storage storage;
    private Shooter shooter;

    private ElapsedTime runtime = new ElapsedTime();
    private Double timeout = .8;

    public Launch1BallCommand(Storage storage, Shooter shooter){
        this.storage = storage;
        this.shooter = shooter;
    }



    @Override
    public void execute() { // runs continuously
       if(shooter.getFlyWheelSpeed() > shooter.targetFlyWheelSpeed - 10 && shooter.getFlyWheelSpeed() < shooter.targetFlyWheelSpeed + 10){
           new SequentialCommandGroup(new InstantCommand(storage::raiseKicker)).schedule();
       }
       if(!storage.kickerDown){
           runtime.reset();
       }

    }

    @Override
    public boolean isFinished() { // returns true when finished

        if (runtime.seconds() > timeout) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        new SequentialCommandGroup(
                new InstantCommand(storage::lowerKicker)
        ).schedule();

    }
}
