package org.firstinspires.ftc.teamcode.Zenith.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;

import java.util.Objects;

public class IntakeWaitCommand extends CommandBase {//This is a separate command used to actually set the target position of the lift for the PID
    private Intake intake;//create a lift object. It will have all the associated code of the lift file since that file outlines a class
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timeIntakeFull = new ElapsedTime();
    private double timeout = 10.0;
    private double intakeTimeout = 0.5;
    public IntakeWaitCommand() {


    }

    @Override
    public void initialize() { // runs once
        runtime.reset();
        timeIntakeFull.reset();
    }

    @Override
    public void execute() { // runs continuously
        long emptySlots = GlobalVariables.currentArtifacts.chars()
                .filter(c -> c == '_')
                .count();

        if (emptySlots != 0){
            timeIntakeFull.reset();
        }

    }

    @Override
    public boolean isFinished() { // returns true when finished
        if (timeIntakeFull.seconds()>intakeTimeout ||runtime.seconds()>timeout){
            return true;
        }


        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
