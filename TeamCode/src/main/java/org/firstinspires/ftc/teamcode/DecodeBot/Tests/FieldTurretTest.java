package org.firstinspires.ftc.teamcode.DecodeBot.Tests;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.CAMERA_RADIUS;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_DEGREE_TO_TICK_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "GlobalTurretTest", group = "AGen1")
public class FieldTurretTest extends CommandOpMode {
    private GamepadEx driver1, driver2;
    private DcMotorEx mFL, mFR, mBL, mBR;
    private IMU imu;
    private Turret turret;
    private MecanumDrive drive;
    private RRSubsystem rrSubsystem;

    private AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();
    private static VisionPortal portal;

    private AprilTagDetection detectedTag = null;
    private boolean targetFound = false;
    private long lastDetectionTime = 0; // nanoseconds

    private static final int IMG_HEIGHT = 600;
    private static final int IMG_WIDTH = 800;
    private static final long TAG_TIMEOUT_NS = 1_000_000_000; //  1 sec

    // Field constants
    public static double GOAL_FIELD_X = 0.0;
    public static double GOAL_FIELD_Y = 0.0;
    public static double ROBOT_X = 0.0;
    public static double ROBOT_Y = 0.0;

    // Telemetry
    private final MultipleTelemetry telemetry2 =
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void initialize() {
        telemetry = telemetry2;
        CommandScheduler.getInstance().reset();

        // Hardware init
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        imu = hardwareMap.get(IMU.class, "imu");
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        turret = new Turret(hardwareMap);
        rrSubsystem = new RRSubsystem(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.setMsTransmissionInterval(1);

        // Vision init
        portal = new VisionPortal.Builder()
                .addProcessors(aTagP)
                .setCameraResolution(new Size(IMG_WIDTH, IMG_HEIGHT))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        portal.setProcessorEnabled(aTagP, true);

        // Driver controls
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B))
                .whenActive(new InstantCommand(() -> GlobalVariables.aColor = "red"));
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.A))
                .whenActive(new InstantCommand(() -> GlobalVariables.aColor = "blue"));
    }

    @Override
    public void run() {
        super.run();

        // Select target tag based on alliance color
        int desiredTagID = (GlobalVariables.aColor.equals("red")) ? 24 : 20;

        // Get AprilTag detections
        List<AprilTagDetection> detections = aTagP.getDetections();

        Pose2d relocalizedPose = relocalize(detections, rrSubsystem.getYawRadians(), telemetry);
        if (relocalizedPose != null) {
            drive.localizer.setPose(relocalizedPose);
        }

        // Pick target tag if visible
        detectedTag = detections.stream()
                .filter(d -> d.id == desiredTagID)
                .findFirst()
                .orElse(null);

        if (detectedTag != null) {
            // Update goal position from tag metadata
            Pose2d tagPose = vectorFToVector2d(detectedTag.metadata.fieldPosition);
            GOAL_FIELD_X = tagPose.position.x;
            GOAL_FIELD_Y = tagPose.position.y;
            lastDetectionTime = System.nanoTime();
            targetFound = true;
        }

        // Always track turret toward last known field goal
        updateTurretTracking();

       double Rotation = cubicScaling(-gamepad1.right_stick_x) * 0.5;
        double FB = cubicScaling(gamepad1.left_stick_y);
        double LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;

        double mFLPower = FB - LR + Rotation;
        double mFRPower = FB - LR - Rotation;
        double mBLPower = FB + LR + Rotation;
        double mBRPower = FB + LR - Rotation;


        mFL.setPower(mFLPower);
        mFR.setPower(mFRPower);
        mBL.setPower(mBLPower);
        mBR.setPower(mBRPower);

        drive.updatePoseEstimate();

        telemetry.addData("Tag Visible", detectedTag != null);
        telemetry.addData("Tracking Goal", targetFound);
        telemetry.addData("Time Since Last (ms)", (System.nanoTime() - lastDetectionTime) / 1e6);
        telemetry.addData("FPS", portal.getFps());
        telemetry.update();
    }


    private void updateTurretTracking() {
        if (!targetFound) return; // nothing to aim at yet

        // Robot pose
        ROBOT_X = drive.localizer.getPose().position.x;
        ROBOT_Y = drive.localizer.getPose().position.y;
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Turret’s relative angle (ticks → radians)
        double turretAngleRobot = turret.getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER;

        // Desired turret angle in field space (robot → goal)
        double desiredTurretAngle = Math.atan2(GOAL_FIELD_Y - ROBOT_Y, GOAL_FIELD_X - ROBOT_X);

        // Convert back to ticks
        double desiredTicks = desiredTurretAngle / TURRET_TICK_TO_RADIAN_MULTIPLIER;
        turret.setTargetPosition(desiredTicks);

        telemetry.addData("Turret Field Angle (deg)", Math.toDegrees(robotYaw + turretAngleRobot));
        telemetry.addData("Desired Field Angle (deg)", Math.toDegrees(desiredTurretAngle));
        telemetry.addData("Target pos (ticks)", turret.getTargetPosition());
        telemetry.addData("Turret encoder", turret.getCurrentPosition());
        telemetry.addData("PID output", turret.getCurrentMotorPower());
    }




    private static Pose2d vectorFToVector2d(VectorF vector) {
        return new Pose2d(
                new Vector2d(vector.get(0), vector.get(1)),
                Math.toRadians(RRSubsystem.imu.getRobotYawPitchRollAngles().getYaw())
        );
    }

    public static Pose2d relocalize(List<AprilTagDetection> detections, double headingRad, Telemetry telemetry) {
        if (detections.isEmpty()) return null;

        double flippedHeading = -headingRad; // Robot vs field frame
        double sumX = 0, sumY = 0;

        for (AprilTagDetection detection : detections) {
            Pose2d tagPose = vectorFToVector2d(detection.metadata.fieldPosition);
            AprilTagPoseFtc ftcPose = detection.ftcPose;

            // Tag position relative to camera
            double xCamToTag = ftcPose.x;
            double yCamToTag = ftcPose.y;

            // Turret angle
            double thetaTurret = Turret.getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER;

            // Camera offset in robot frame
            double xCam = TURRET_OFFSET_X + CAMERA_RADIUS * Math.cos(thetaTurret);
            double yCam = TURRET_OFFSET_Y + CAMERA_RADIUS * Math.sin(thetaTurret);

            // Transform to robot frame
            double xBotToTag = xCam + (xCamToTag * Math.cos(flippedHeading) - yCamToTag * Math.sin(flippedHeading));
            double yBotToTag = yCam + (xCamToTag * Math.sin(flippedHeading) + yCamToTag * Math.cos(flippedHeading));

            // Robot on field = Tag on field - Bot→Tag
            double xBotOnField = tagPose.position.x - xBotToTag;
            double yBotOnField = tagPose.position.y - yBotToTag;

            sumX += xBotOnField;
            sumY += yBotOnField;
        }

        return new Pose2d(sumX / detections.size(), sumY / detections.size(), headingRad);
    }

    private double cubicScaling(float joystickValue) {
        double v = 0.05 * joystickValue + 0.95 * Math.pow(joystickValue, 3);
        if (joystickValue > 0.02) return 0.1 + v;
        else if (joystickValue < -0.02) return -0.1 + v;
        else return 0;
    }
}
