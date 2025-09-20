package org.firstinspires.ftc.teamcode.DecodeBot.Commands;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_360_TURN_TICKS;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.getCurrentPosition;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;

public class TurretFlipCommand extends CommandBase {

    private Turret turret;
    public double targetPosition;
    int tolerance;
    public TurretFlipCommand(Turret turret, int tolerance) {


        this.turret = turret;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() { // runs once
      //  turret.tracking = false;



        if (Turret.getCurrentPosition() > 0) {
            targetPosition = getCurrentPosition() - TURRET_360_TURN_TICKS;

        } else if (Turret.getCurrentPosition() < 0) {
            targetPosition = getCurrentPosition() + TURRET_360_TURN_TICKS;
        }
//        lift.setTargetPosition(targetPosition);

    }

    @Override
    public void execute() { // runs continuously

        Turret.setTargetPosition(targetPosition);

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
        //Turret.tracking = true;
    }

}