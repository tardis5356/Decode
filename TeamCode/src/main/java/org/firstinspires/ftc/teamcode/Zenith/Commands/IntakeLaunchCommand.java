package org.firstinspires.ftc.teamcode.Zenith.Commands;

import static org.firstinspires.ftc.teamcode.Zenith.Commands.LaunchSequenceCommand.launcOne;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;


public class IntakeLaunchCommand extends CommandBase {//This is a separate command used to actually set the target position of the lift for the PID
    private Storage storage;//create a lift object. It will have all the associated code of the lift file since that file outlines a class
    private Intake intake;
    private ElapsedTime runtime = new ElapsedTime();
    private Double timeout = 2.0;

    public IntakeLaunchCommand(Storage storage, Intake intake) {
        // this is the actual method itself
this.intake = intake;
        this.storage = storage;

    }

    @Override
    public void initialize() { // runs once

        //TODO Test run this

                new InstantCommand(intake::in);


        runtime.reset();
//        lift.setTargetPosition(targetPosition);

    }

    @Override
    public void execute() { // runs continuously
        //  lift.setTolerance(tolerance);
        // the execute is like the periodic of subsystems, just for commands instead.
        // here we finally set the lift target position to the target position

        //  lift.updatePIDValues();

    }

    @Override
    public boolean isFinished() { // returns true when finished
        if (GlobalVariables.currentArtifacts.charAt(1) != '_' || (runtime.seconds() > timeout)) {
            return true;
        }


        return false;
    }

    @Override
    public void end(boolean interrupted) {

                new SequentialCommandGroup(
                        new InstantCommand(intake::stop),
                        launcOne(storage)
                );



    }

}
