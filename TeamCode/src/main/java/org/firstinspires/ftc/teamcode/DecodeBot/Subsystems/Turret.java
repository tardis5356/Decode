package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_D;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_I;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_P;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.DecodeBot.DecodeTeleOp;

public class Turret extends SubsystemBase {

    public static DcMotorEx mT;
    public PIDController controller;

    public static double targetPosition; // ticks
    public double motorPower;

    public static boolean PIDDisabled = false;

    // conversion constants


    public Turret(HardwareMap hardwareMap) {
        mT = hardwareMap.get(DcMotorEx.class, "mT");

        mT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mT.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(BotPositions.TURRET_P, BotPositions.TURRET_I, BotPositions.TURRET_D);
    }

    @Override
    public void periodic() {
        if (!PIDDisabled) {
            motorPower = controller.calculate(mT.getCurrentPosition(), targetPosition);
        } else {
            motorPower = 0;
        }
     //   mT.setPower(motorPower);
    }

    public static double getCurrentPosition() {
        return mT.getCurrentPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public static void setTargetPosition(double newTargetPosition) {
        targetPosition = newTargetPosition;
    }
     public double getTurretThetaRAD() {
        return -(Turret.getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER);
     }

    public double getCurrentMotorPower() {
        return motorPower;
    }
}
