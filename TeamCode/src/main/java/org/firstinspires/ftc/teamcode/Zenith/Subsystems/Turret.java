package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_RADIANS_PER_TICK;
//import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_S;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.MAX_TURRET_ANGLE_DEG;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_TOLERANCE_DEG;

import static org.firstinspires.ftc.teamcode.Zenith.Util.vectorFToPose2d;
import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Zenith.Auto.MecanumDrive;

public class Turret extends SubsystemBase {

    public static DcMotorEx mT;
    //public static TouchSensor lT;
    private PIDController pidController;
    private SimpleMotorFeedforward feedforwardController;

    private double kF = 0.0;   // velocity gain
    private double kS = 0.0;   // static gain


    private double lastTicks = 0;
    private double lastTime = 0;
    private double turretVelocityTicksPerSec = 0;
    private final VoltageSensor voltageSensor;
    private static double targetPositionTicks;
    private double motorPower;
    private double pidPower;
    public static double turretOffset = 0;
    public static int manualOffset;
    private boolean PIDDisabled = false;

    public static int desiredTicks;

    // === TURRET CONSTANTS ===

    private double lastTurretAngle = 0.0; // radians

    public Turret(HardwareMap hardwareMap) {
        mT = hardwareMap.get(DcMotorEx.class, "mT");
        //lT = hardwareMap.get(TouchSensor.class, "lT");

        mT.setDirection(DcMotorSimple.Direction.REVERSE);
        mT.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Run without encoder because we don't want to use the firmware PID controller
        mT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        pidController = new PIDController(BotPositions.TURRET_P, BotPositions.TURRET_I, BotPositions.TURRET_D);
        // feedforwardController = new SimpleMotorFeedforward(TURRET_S, TURRET_V);
        // pidController.setTolerance(BotPositions.TURRET_TOLERANCE);
    }

    @Override
    public void periodic() {
        // Run PID control if enabled
        //if (lT.isPressed()) {
//            turretOffset += mT.getCurrentPosition();
//        }


        pidController.setPID(BotPositions.TURRET_P, BotPositions.TURRET_I, BotPositions.TURRET_D);

        if ((Math.abs(getCurrentPosition() - desiredTicks) / TURRET_TICKS_PER_DEGREE) > TURRET_TOLERANCE_DEG) {
            motorPower = pidController.calculate(getCurrentPosition(), targetPositionTicks + manualOffset) + Math.signum((desiredTicks + manualOffset) - getCurrentPosition()) * (TURRET_S / voltageSensor.getVoltage());

        } else {
            motorPower = 0;
        }


        mT.setPower(motorPower);


    }

    // === BASIC CONTROLS ===
    public static double getCurrentPosition() {
        return -(mT.getCurrentPosition() - turretOffset);
    }

    public double getCurrentMotorPower() {
        return motorPower;
    }

    public double getTurretThetaRAD() {
        return (getCurrentPosition() * TURRET_RADIANS_PER_TICK);
    }

    public static void setTargetPosition(double ticks) {
        targetPositionTicks = ticks;
    }

    public static double getTargetPosition() {
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


        // Select tag based on alliance color
        int desiredTagID = (GlobalVariables.aColor.equals("red")) ? 24 : 20;

        // Shooting Target Offset relative to AprilTag
        // Positive Offset = further behind apriltag
        double targetTagXOffset = 0, targetTagYOffset = 0;

        Pose2d targetAprilTagPos = vectorFToPose2d(getCurrentGameTagLibrary().lookupTag(desiredTagID).fieldPosition, 0);
        // === Apply offset away from origin ===
        double goalX = targetAprilTagPos.position.x + Math.signum(targetAprilTagPos.position.x) * targetTagXOffset;
        double goalY = targetAprilTagPos.position.y + Math.signum(targetAprilTagPos.position.y) * targetTagYOffset;

        // === Compute turret tracking ===
        double robotX = drive.localizer.getPose().position.x;
        double robotY = drive.localizer.getPose().position.y;
        double robotHeadingRad = drive.localizer.getPose().heading.toDouble();

        double turretFieldX = robotX + Math.cos(robotHeadingRad) * TURRET_OFFSET_X - Math.sin(robotHeadingRad) * TURRET_OFFSET_Y;
        double turretFieldY = robotY + Math.sin(robotHeadingRad) * TURRET_OFFSET_X + Math.cos(robotHeadingRad) * TURRET_OFFSET_Y;

        double desiredFieldTurretAngleRAD = Math.atan2(goalY - turretFieldY, goalX - turretFieldX) + Math.PI;
//The following makes sures that the target angle are always between (0, 2PI)
        double desiredTurretOnBotAngleRAD = (desiredFieldTurretAngleRAD - robotHeadingRad) % (2 * Math.PI);
//This makes sure that the turret is never pointed at angle past the maxAngle
        //Avoids cable wrapping
        if (desiredTurretOnBotAngleRAD > Math.toRadians(MAX_TURRET_ANGLE_DEG)) {
            desiredTurretOnBotAngleRAD -= 2 * Math.PI;
        }


        desiredTicks = (int) Math.round(desiredTurretOnBotAngleRAD / TURRET_RADIANS_PER_TICK);


        setTargetPosition(desiredTicks);

        GlobalVariables.distanceFromTarget = Math.hypot(goalY - turretFieldY, goalX - turretFieldX);


        // Telemetry
        telemetry.addData("TargetAprilTagXPose", targetAprilTagPos.position.x);
        telemetry.addData("TargetAprilTagYPose", targetAprilTagPos.position.y);
        //check these on the testbed to see if the preset april tag positions are correct
        telemetry.addData("Tag ID", desiredTagID);
        telemetry.addData("Offset X", targetTagXOffset);
        telemetry.addData("Offset Y", targetTagYOffset);
        telemetry.addData("Target Field Turret Angle (deg)", Math.toDegrees(desiredFieldTurretAngleRAD));
        telemetry.addData("Target Turret On Bot Angle (deg)", Math.toDegrees(desiredTurretOnBotAngleRAD));
        telemetry.addData("TurretTheta", Math.toDegrees(getTurretThetaRAD()));
        telemetry.addData("TurretError", (Math.abs(getCurrentPosition() - desiredTicks) / TURRET_TICKS_PER_DEGREE));
        telemetry.addData("Turret Distance", GlobalVariables.distanceFromTarget);
        telemetry.addData("Target Pos (ticks)", desiredTicks);
        telemetry.addData("Radianspertick", TURRET_RADIANS_PER_TICK);
        telemetry.addData("Ticksperdegree", TURRET_TICKS_PER_DEGREE);
        //   telemetry.addData("RawTurretTicks", mT.getCurrentPosition());
        telemetry.addData("ZeroedTurretTicks", getCurrentPosition());
        telemetry.addData("TurretRawMotorPower", mT.getPower());
    }

    // === FLIP MANAGEMENT ===


    // === ANGLE UTILITY ===
    private double unwrapAngle(double currentAngle, double lastAngle) {
        double delta = currentAngle - lastAngle;
        while (delta <= -Math.PI) delta += 2.0 * Math.PI;
        while (delta > Math.PI) delta -= 2.0 * Math.PI;
        return lastAngle + delta;
    }
}
