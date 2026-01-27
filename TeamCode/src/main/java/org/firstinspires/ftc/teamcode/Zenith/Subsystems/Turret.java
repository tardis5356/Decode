package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.CAMERA_RADIUS;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_RADIANS_PER_TICK;
//import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_S;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.MAX_TURRET_ANGLE_DEG;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_TOLERANCE_DEG;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_V;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TurretAngle_kSMatrix;
import static org.firstinspires.ftc.teamcode.Zenith.Util.vectorFToPose2d;
import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;

import static java.lang.Math.signum;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Zenith.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Zenith.InterpolatingDoubleTreeMap;

public class Turret extends SubsystemBase {

    public static DcMotorEx mT;
    public static double turretOffset = 0;
    public static int manualOffset = 0;
    public static double turretError;
    public boolean mustWrap;

    public static double lastTurretError;

    public static int desiredTicks;
    private static double targetPositionTicks;
    private final VoltageSensor voltageSensor;
    public static double startingvoltage;
    public static TouchSensor lT, lT2;

    public boolean turretLocalized = true;
    public int cwORccw = 1;
    public int desiredTagID;

    private PIDController pidController;
    //    private PIDController pidfController;
    InterpolatingDoubleTreeMap CCWTurretAnglekS = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap CWTurretAnglekS = new InterpolatingDoubleTreeMap();
    private SimpleMotorFeedforward feedforwardController;
    private double kF = 0.0;   // velocity gain
    private double kA = 0.0; //acceleration gain
    private double kS = 0.0;   // static gain
    private double lastTicks = 0;
    private double lastTime = 0;
    private double turretVelocityDegreesPerSec = 0;
    private double robotVelocityDegreesPerSec = 0;
    private double turretToFieldAngularVelocity_Deg = 0;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime timeSinceWrapped = new ElapsedTime();
    private ElapsedTime timeFromLastTurretError = new ElapsedTime();
    private double motorPower;
    private double pidPower;
    private double powerAdded = 0;
    private boolean PIDDisabled = false;

    private MecanumDrive drive;
    public Servo liT;
    public GoBildaPinpointDriver driver;

    // === TURRET CONSTANTS ===
    private double lastTurretAngle = 0.0; // radians

    public Turret(HardwareMap hardwareMap) {
        mT = hardwareMap.get(DcMotorEx.class, "mT");
        lT = hardwareMap.get(TouchSensor.class, "lT");
        lT2 = hardwareMap.get(TouchSensor.class, "lT2");

       liT  = hardwareMap.get(Servo.class, "liT");

        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        PIDDisabled = false;

        mT.setDirection(DcMotorSimple.Direction.REVERSE);
        mT.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Run without encoder because we don't want to use the firmware PID controller
        mT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        pidController = new PIDController(BotPositions.TURRET_P, BotPositions.TURRET_I, BotPositions.TURRET_D);
//        pidController =  new PIDController(BotPositions.TURRET_P, BotPositions.TURRET_I, BotPositions.TURRET_D);
        feedforwardController = new SimpleMotorFeedforward(TURRET_S, TURRET_V);

        //gets data from bot positions, reads the angle and then refers to the CW/CCW motor power
        for (int i = 0; i < TurretAngle_kSMatrix.length; i++) {
           CCWTurretAnglekS.put(TurretAngle_kSMatrix[i][0],TurretAngle_kSMatrix[i][1]);
           CWTurretAnglekS.put(TurretAngle_kSMatrix[i][0],TurretAngle_kSMatrix[i][2]);
        }

        startingvoltage = voltageSensor.getVoltage();
        // pidController.setTolerance(BotPositions.TURRET_TOLERANCE);

        manualOffset = 0;
    }

    // === BASIC CONTROLS ===
    public static double getCurrentPosition() {
        return -mT.getCurrentPosition() + turretOffset;
    }

    public static double getTargetPosition() {
        return targetPositionTicks;
    }

    public static void setTargetPosition(double ticks) {
        targetPositionTicks = ticks;
    }

    @Override
    public void periodic() {

        // Run PID control if enabled

        //if (lT.isPressed()) {
//            turretOffset += mT.getCurrentPosition();
//        }

        //These values are to compensate for angular velocity of the robot when applying power to turret
        turretVelocityDegreesPerSec = mT.getVelocity() / TURRET_TICKS_PER_DEGREE;
        robotVelocityDegreesPerSec = driver.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        turretToFieldAngularVelocity_Deg = turretVelocityDegreesPerSec - robotVelocityDegreesPerSec;



         desiredTagID = (GlobalVariables.aColor.equals("red")) ? 24 : 20;
        turretError = (Math.abs(getCurrentPosition() - getTargetPosition()) / TURRET_TICKS_PER_DEGREE);



        pidController.setPID(BotPositions.TURRET_P, BotPositions.TURRET_I, BotPositions.TURRET_D);
        //pidController.setIntegrationBounds();
        if (!PIDDisabled) {
        if(turretLocalized) {

                if (turretError > TURRET_TOLERANCE_DEG) {
                    motorPower = pidController.calculate(getCurrentPosition(), targetPositionTicks);
                         //   + feedforwardController.calculate(turretToFieldAngularVelocity_Deg);
                } else {
                    motorPower = 0;
                }

        }else{
            motorPower = pidController.calculate(getCurrentPosition(), targetPositionTicks);
            //TODO: actually debug this before next comp
//            motorPower = -.5 * cwORccw;
//            if(lT2.isPressed()){
//                cwORccw = -2;
//            }
        }
        } else motorPower = pidController.calculate(getCurrentPosition(), 0);

//set the kS according to the turret theta

        if (signum(motorPower)>0){
            kS = CCWTurretAnglekS.get(getTurretThetaDEG());
        } else if(signum(motorPower)<0){
            kS = - CWTurretAnglekS.get(getTurretThetaDEG());
        } else {
            kS = 0;
        }

//+ motor power is CCW
        mT.setPower((motorPower + signum(motorPower) * TURRET_S) * (12/voltageSensor.getVoltage()));


    }

    public double getCurrentMotorPower() {
        return motorPower;
    }

    public double getTurretThetaRAD() {
        return (getCurrentPosition() * TURRET_RADIANS_PER_TICK);
    }

    public double getTurretThetaDEG(){
        return (getCurrentPosition()/TURRET_TICKS_PER_DEGREE);
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

        // Shooting Target Offset relative to AprilTag
        // Positive Offset = further behind apriltag
        double targetTagXOffset = 4 /*8*/, targetTagYOffset = 7/*7*/;

        Pose2d targetAprilTagPos = vectorFToPose2d(getCurrentGameTagLibrary().lookupTag(desiredTagID).fieldPosition, 0);

        // === Apply offset away from origin ===
        double goalX = targetAprilTagPos.position.x + (signum(targetAprilTagPos.position.x) * targetTagXOffset);
        double goalY = targetAprilTagPos.position.y + (signum(targetAprilTagPos.position.y) * targetTagYOffset);

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

        desiredTicks = (int) Math.round(desiredTurretOnBotAngleRAD / TURRET_RADIANS_PER_TICK) + manualOffset;

        setTargetPosition(desiredTicks);

        GlobalVariables.distanceFromTarget = Math.hypot(goalY - turretFieldY, goalX - turretFieldX) + 6 - CAMERA_RADIUS;


        // Telemetry
//        telemetry.addData("TargetAprilTagXPose", targetAprilTagPos.position.x);
//        telemetry.addData("TargetAprilTagYPose", targetAprilTagPos.position.y);
//        //check these on the testbed to see if the preset april tag positions are correct
//        telemetry.addData("Tag ID", desiredTagID);
//        telemetry.addData("Offset X", targetTagXOffset);
//        telemetry.addData("Offset Y", targetTagYOffset);
//        telemetry.addData("Target Field Turret Angle (deg)", Math.toDegrees(desiredFieldTurretAngleRAD));
//        telemetry.addData("Target Turret On Bot Angle (deg)", Math.toDegrees(desiredTurretOnBotAngleRAD));
        telemetry.addData("Turret Theta (deg)       Current         Error\n                                           ", "%.2f\t\t%.2f", Math.toDegrees(getTurretThetaRAD()),((getCurrentPosition() - desiredTicks) / TURRET_TICKS_PER_DEGREE));
//
//        telemetry.addData("TurretPosX", turretFieldX);
//        telemetry.addData("TurretPosY", turretFieldY);
//        telemetry.addLine("Turret Ticks");
//        telemetry.addData("Target", desiredTicks);
//        telemetry.addData("Current", getCurrentPosition());
    }

    public void aprilTagTracking ( Camera camera, Telemetry telemetry){
        GlobalVariables.distanceFromTarget = camera.getDistance();
        desiredTicks = (int) Math.round((camera.getBearing()*TURRET_TICKS_PER_DEGREE) + getCurrentPosition());
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
