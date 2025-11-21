package org.firstinspires.ftc.teamcode.DecodeBot.Commands;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.getCurrentPosition;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.getTargetPosition;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;

public class TurretFlipCommand extends CommandBase {

    private Turret turret;
    public double targetPosition;
    public static double newTargetOffset = 0;
    int tolerance;
    public TurretFlipCommand( int tolerance, Turret turret) {
        this.tolerance = tolerance;
        this.turret = turret;
    }

    @Override
    public void initialize() { // runs once

turret.turretFlipping = true;


        if (turret.getTargetPosition() > 0) {
            targetPosition = getTargetPosition() - TURRET_TICKS_PER_DEGREE;


        } else if (turret.getTargetPosition() < 0) {
            targetPosition = getTargetPosition() + TURRET_TICKS_PER_DEGREE;

        }

        Turret.setTargetPosition(targetPosition);
    }

    @Override
    public void execute() { // runs continuously



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
        turret.turretFlipping = false;


    }

}