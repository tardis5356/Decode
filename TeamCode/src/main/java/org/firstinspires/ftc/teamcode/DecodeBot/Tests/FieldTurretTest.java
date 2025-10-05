package org.firstinspires.ftc.teamcode.DecodeBot.Tests;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.CAMERA_RADIUS;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.RRSubsystem.imu;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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

    private Turret turret;
    private MecanumDrive drive;
    private static RRSubsystem rrSubsystem;

    private AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();
    private static VisionPortal portal;

    private AprilTagDetection detectedTag = null;
    private boolean targetFound = false;
    private long lastDetectionTime = 0; // nanoseconds
    private static final double TURRET_ZERO_OFFSET = Math.toRadians(90);
    private static final int IMG_HEIGHT = 720;
    private static final int IMG_WIDTH = 1280;
    private static final long TAG_TIMEOUT_NS = 1_000_000_000; //  1 sec

    // Field constants
    public static double GOAL_FIELD_X = 0.0;
    public static double GOAL_FIELD_Y = 0.0;
    public static double ROBOT_X = 0.0;
    public static double ROBOT_Y = 0.0;
public static int desiredTagID;
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

        mBL.setDirection(DcMotorSimple.Direction.REVERSE);
        mFL.setDirection(DcMotorSimple.Direction.REVERSE);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        turret = new Turret(hardwareMap);
        rrSubsystem = new RRSubsystem(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(270)));

        telemetry.setMsTransmissionInterval(1);

        rrSubsystem.setStartingOffsetDegs(270);

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

        if (GlobalVariables.aColor == "red") {
            desiredTagID = 24;
        }
        if (GlobalVariables.aColor == "blue") {
            desiredTagID = 20;
        }

        // Get AprilTag detections
        List<AprilTagDetection> detections = aTagP.getDetections();

        Pose2d relocalizedPose = relocalize(detections, Math.toRadians(rrSubsystem.getYawDegrees()), telemetry);
        if (relocalizedPose != null) {
            drive.localizer.setPose(relocalizedPose);
        }

        for (AprilTagDetection detection : detections) {
            if (detection.id == desiredTagID) {
                detectedTag = detection;
                targetFound = true;

                lastDetectionTime = System.nanoTime();
                break;
            }
        }

        if (detectedTag != null) {
            // Update goal position from tag metadata
            Pose2d tagPose = vectorFToPose2d(detectedTag.metadata.fieldPosition);
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
        telemetry.addData("Heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.addData("X", drive.localizer.getPose().position.x);
        telemetry.addData("Y", drive.localizer.getPose().position.y);
        telemetry.addData("Tag Visible", detectedTag != null);
        telemetry.addData("Tracking Goal", targetFound);
        telemetry.addData("Time Since Last (ms)", (System.nanoTime() - lastDetectionTime) / 1e6);
        telemetry.addData("FPS", portal.getFps());
        telemetry.update();
    }

    private double lastTurretAngle = 0.0; // radians

    private double unwrapAngle(double currentAngle, double lastAngle) {
        double delta = currentAngle - lastAngle;
        while (delta <= -Math.PI) delta += 2.0 * Math.PI;
        while (delta >  Math.PI) delta -= 2.0 * Math.PI;
        return lastAngle + delta;
    }



        private void updateTurretTracking() {
            if (!targetFound) return;

            // Robot pose
            ROBOT_X = drive.localizer.getPose().position.x;
            ROBOT_Y = drive.localizer.getPose().position.y;
            double robotYaw = Math.toRadians(rrSubsystem.getYawDegrees()); // use radians

            // Desired field angle to goal (absolute, in field frame)
            double desiredFieldAngle = Math.atan2(GOAL_FIELD_Y - ROBOT_Y, GOAL_FIELD_X - ROBOT_X);

            // Convert to turret-relative robot frame + zero offset
            double rawTurretAngle = desiredFieldAngle - robotYaw + TURRET_ZERO_OFFSET;

            // Unwrap so it stays continuous
            double desiredTurretAngleRobot = unwrapAngle(rawTurretAngle, lastTurretAngle);
            lastTurretAngle = desiredTurretAngleRobot;

            // Convert to ticks
            int desiredTicks = (int) Math.round(desiredTurretAngleRobot / TURRET_TICK_TO_RADIAN_MULTIPLIER);

            turret.setTargetPosition(desiredTicks);

            telemetry.addData("Robot Yaw (deg)", Math.toDegrees(robotYaw));
            telemetry.addData("Desired Field Angle (deg)", Math.toDegrees(desiredFieldAngle));
            telemetry.addData("Desired Turret Angle (deg)", Math.toDegrees(desiredTurretAngleRobot));
            telemetry.addData("Target pos (ticks)", turret.getTargetPosition());
            telemetry.addData("Turret encoder", turret.getCurrentPosition());
        }









    private static Pose2d vectorFToPose2d(VectorF vector) {
        return new Pose2d(
                new Vector2d(vector.get(0), vector.get(1)),
                Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw())
        );
    }

    public static Pose2d relocalize(List<AprilTagDetection> detections,
                                    double imuHeadingRad,
                                    Telemetry telemetry) {
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();

        String numFormat = "%.2f";

        // Negative because the robot is rotating, not the field


        double finalX = 0;
        double finalY = 0;
        double finalHeadingDeg = 0;

        double theta_turret = -(Turret.getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER);
        double flippedHeading = -imuHeadingRad;


        telemetry.addData("theta_turret", numFormat, Math.toDegrees(theta_turret));
        telemetry.addData("flipped_heading", numFormat, Math.toDegrees(flippedHeading));

        for (AprilTagDetection detection : detections) {
            Pose2d tagPose = vectorFToPose2d(detection.metadata.fieldPosition);
            AprilTagPoseFtc ftcPose = detection.ftcPose;

            telemetry.addData("tag name", detection.metadata.name);

            // Tag position in camera frame
            double x_cameraToTag = ftcPose.x;  // inches (FTC uses x right, y down, z forward)
            double y_cameraToTag = ftcPose.y;
            double yaw_cameraToTag = ftcPose.yaw;

            telemetry.addData("x_cameraToTag", numFormat, x_cameraToTag);
            telemetry.addData("y_cameraToTag", numFormat, y_cameraToTag);
            telemetry.addData("yaw_cameraToTag", numFormat, yaw_cameraToTag);

            // Turret angle (in radians)



            // Calculate camera position in robot frame (robot → camera)
            //TODO check signs
//            double x_tagToTurret = TURRET_OFFSET_X - CAMERA_RADIUS * Math.cos(theta_turret);
//            double y_tagToTurret = TURRET_OFFSET_Y - CAMERA_RADIUS * Math.sin(theta_turret);
//First translate to turret center
           y_cameraToTag += CAMERA_RADIUS;
            //Next rotate to bot/turret coordinate system
            //TODO Tag to turret numbers aren't always working
            double x_tagToTurret = x_cameraToTag * Math.cos(Math.PI/2 - theta_turret) + (y_cameraToTag) * Math.sin(Math.PI/2 - theta_turret);
            double y_tagToTurret =  -x_cameraToTag * Math.sin(Math.PI/2 - theta_turret) + (y_cameraToTag) * Math.cos(Math.PI/2 - theta_turret);



            telemetry.addData("x_tagToTurret", numFormat, x_tagToTurret);
            telemetry.addData("y_tagToTurret", numFormat, y_tagToTurret);

            //Shift from turret to robot frame
            double x_tagToBot = x_tagToTurret + TURRET_OFFSET_X;
            // changed symbol on the ytagtorobot
            double y_tagToBot = y_tagToTurret - TURRET_OFFSET_Y;


            telemetry.addData("x_tagToBot", numFormat, x_tagToBot);
            telemetry.addData("y_tagToBot", numFormat, y_tagToBot);

            // Get tag position on field
            double x_tagOnField = tagPose.position.x;  // in inches
            double y_tagOnField = tagPose.position.y;

            telemetry.addData("x_tagOnField", numFormat, x_tagOnField);
            telemetry.addData("y_tagOnField", numFormat, y_tagOnField);

            // Robot position on field = Tag on field - Bot-to-Tag vector
            double x_botOnField = (x_tagToBot * Math.cos(flippedHeading) + y_tagToBot *Math.sin(flippedHeading)) + x_tagOnField;
            double y_botOnField =  (-x_tagToBot * Math.sin(flippedHeading) + y_tagToBot *Math.cos(flippedHeading)) + y_tagOnField ;

            telemetry.addData("x_botOnField", numFormat, x_botOnField);
            telemetry.addData("y_botOnField", numFormat, y_botOnField);

            finalX += x_botOnField;
            finalY += y_botOnField;

            telemetry.addLine();
        }

        if (finalX == 0)
            return null;

        // Return average pose from all visible tags
        return new Pose2d(finalX / detections.size(), finalY / detections.size(), imuHeadingRad);

    }

    private double cubicScaling(float joystickValue) {
        double v = 0.05 * joystickValue + 0.95 * Math.pow(joystickValue, 3);
        if (joystickValue > 0.02)
            return 0.1 + v;
        else if (joystickValue < -0.02)
            return -0.1 + v;
        else return 0;
    }
}
