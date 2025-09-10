package org.firstinspires.ftc.teamcode.DecodeBot.Commands;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.getCurrentPosition;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;

public class TurretToStateCommand extends CommandBase {
    private final String turnDirection;//This is a separate command used to actually set the target position of the lift for the PID
    private Turret turret;//create a lift object. It will have all the associated code of the lift file since that file outlines a class

    public double targetPosition;
    int tolerance;//This is a +/- # of ticks on the lift. We have this so that the PID doesn't get stuck oscillating trying to reach an exact value.

    public TurretToStateCommand(Turret turret,String turnDirection, int tolerance) {
        // this is the actual method itself. It takes a lift as an input to associate with its own, that way it can change the target position value of the lift.
        // Does the same thing with the target position, taking a double in as an input
        // and same thing with the tolerance

        this.turret = turret;
        this.turnDirection = turnDirection;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() { // runs once
        turret.tracking = "false";

        if(turnDirection == "left"){
            targetPosition = getCurrentPosition() - BotPositions.TURRET_TURN_TICKS;

        }else if (turnDirection == "right"){
            targetPosition = getCurrentPosition() + BotPositions.TURRET_TURN_TICKS;
        }
//        lift.setTargetPosition(targetPosition);
    }

    @Override
    public void execute() { // runs continuously
        //  lift.setTolerance(tolerance);
        // the execute is like the periodic of subsystems, just for commands instead.
        // here we finally set the lift target position to the target position
        Turret.setTargetPosition(targetPosition);
        //  lift.updatePIDValues();
    }

    @Override
    public boolean isFinished() { // returns true when finished
        if (targetPosition == 10)
            return true;
        else
            return Math.abs(getCurrentPosition() - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
    Turret.tracking = "true";
    }

}