package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DecodeBot.Util.vectorFToPose2d;
import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Commands.TurretFlipCommand;

public class Turret extends SubsystemBase {

    public static DcMotorEx mT;
    private PIDController controller;

    private static double targetPositionTicks;
    private double motorPower;
    private boolean PIDDisabled = false;

    // === TURRET CONSTANTS ===

    private static final double MAX_ANGLE_DEG = 200; // Flip threshold

    private double lastTurretAngle = 0.0; // radians
    public static boolean turretFlipping = false;

    public Turret(HardwareMap hardwareMap) {
        mT = hardwareMap.get(DcMotorEx.class, "mT");
        mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mT.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mT.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(BotPositions.TURRET_P, BotPositions.TURRET_I, BotPositions.TURRET_D);
    }

    @Override
    public void periodic() {
        // Run PID control if enabled
        if (!PIDDisabled) {
            motorPower = controller.calculate(mT.getCurrentPosition(), targetPositionTicks);
        } else {
            motorPower = 0;
        }
        mT.setPower(motorPower);

        // Always manage turret flip automatically
        manageFlip(MAX_ANGLE_DEG);
    }

    // === BASIC CONTROLS ===
    public static double getCurrentPosition() {
        return mT.getCurrentPosition();
    }

    public double getCurrentMotorPower() {
        return motorPower;
    }

    public double getTurretThetaRAD() {
        return -(getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER);
    }

    public static void setTargetPosition(double ticks) {
        targetPositionTicks = ticks;
    }

    public double getTargetPosition() {
       return targetPositionTicks;
    }

    public void disablePID() {
        PIDDisabled = true;
    }

    public void enablePID() {
        PIDDisabled = false;
    }

    // === TRACKING TO APRILTAG (with offset) ===
    public void updateTurretTracking(MecanumDrive drive, Telemetry telemetry) {

        if (turretFlipping) {
            telemetry.addLine("Turret flipping — tracking paused.");
            return;
        }


        // Select tag based on alliance color
        int desiredTagID = (GlobalVariables.aColor.equals("red")) ? 24 : 20;

        // Shooting Target Offset relative to AprilTag
        // Positive Offset = further behind apriltag
        double targetTagXOffset = 0, targetTagYOffset = 0;

        Pose2d targetPos = vectorFToPose2d(getCurrentGameTagLibrary().lookupTag(desiredTagID).fieldPosition, 0);
        // === Apply offset away from origin ===
        double goalX = targetPos.position.x + Math.signum(targetPos.position.x) * targetTagXOffset;
        double goalY = targetPos.position.y + Math.signum(targetPos.position.y) * targetTagYOffset;

        // === Compute turret tracking ===
        double robotX = drive.localizer.getPose().position.x;
        double robotY = drive.localizer.getPose().position.y;
        double robotHeadingRad = drive.localizer.getPose().heading.toDouble();

        double turretFieldX = robotX + Math.cos(robotHeadingRad) * TURRET_OFFSET_X
                - Math.sin(robotHeadingRad) * TURRET_OFFSET_Y;
        double turretFieldY = robotY + Math.sin(robotHeadingRad) * TURRET_OFFSET_X
                + Math.cos(robotHeadingRad) * TURRET_OFFSET_Y;

        double desiredFieldTurretAngle = Math.atan2(goalY - turretFieldY, goalX - turretFieldX);
        double desiredTurretOnBotAngle = (desiredFieldTurretAngle - robotHeadingRad) % (2 * Math.PI);
        double desiredTurretAngleRobot = unwrapAngle(desiredTurretOnBotAngle, lastTurretAngle);
        lastTurretAngle = desiredTurretAngleRobot;

        int desiredTicks = (int) Math.round(desiredTurretAngleRobot / TURRET_TICK_TO_RADIAN_MULTIPLIER);
        setTargetPosition(desiredTicks);

        double turretDistance = Math.hypot(goalY - turretFieldY, goalX - turretFieldX);

        // Telemetry
        telemetry.addData("Tag ID", desiredTagID);
        telemetry.addData("Offset X", targetTagXOffset);
        telemetry.addData("Offset Y", targetTagYOffset);
        telemetry.addData("Turret Angle (deg)", Math.toDegrees(desiredTurretAngleRobot));
        telemetry.addData("Turret Distance", turretDistance);
        telemetry.addData("Target Pos (ticks)", desiredTicks);
        telemetry.addData("Encoder", getCurrentPosition());
    }

    // === FLIP MANAGEMENT ===
    private void manageFlip(double maxAngleDeg) {
        double currentAngleDeg = Math.toDegrees(getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER);

        if (Math.abs(currentAngleDeg) > maxAngleDeg && !turretFlipping) {

            turretFlipping = true;
            new TurretFlipCommand(10);

        }
    }

    // === ANGLE UTILITY ===
    private double unwrapAngle(double currentAngle, double lastAngle) {
        double delta = currentAngle - lastAngle;
        while (delta <= -Math.PI) delta += 2.0 * Math.PI;
        while (delta > Math.PI) delta -= 2.0 * Math.PI;
        return lastAngle + delta;
    }
}
