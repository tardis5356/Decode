package org.firstinspires.ftc.teamcode.Zenith.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;

public class MoveInArtifactCommand extends CommandBase{

    private Intake intake;
    private ElapsedTime runtime = new ElapsedTime();
    private Double timeout = .4;

    public MoveInArtifactCommand(Intake intake) {
        // this is the actual method itself
        this.intake = intake;

    }

    @Override
    public void initialize() { // runs once

        //TODO Test run this

       // new InstantCommand(intake::in)/*.schedule()*/;

        intake.in();
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
        if (GlobalVariables.currentArtifacts.charAt(0) != '_' || (runtime.seconds() > timeout)) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        //new SequentialCommandGroup(

        //).schedule();



    }



}
