package org.firstinspires.ftc.teamcode.DecodeBot.Commands;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.CAMERA_RADIUS;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;
import java.util.List;
@Config
public class RelocalizationCommand extends CommandBase {
    MecanumDrive drive;
    RRSubsystem rrSubsystem;
    AprilTagProcessor aprilTagProcessor;
    Telemetry telemetry;


    private static Pose2d vectorFToVector2d(VectorF vector) {

        return new Pose2d(new Vector2d(vector.get(0), vector.get(1)), Math.toRadians(RRSubsystem.imu.getRobotYawPitchRollAngles().getYaw()));
    }

    public static double quaternionToHeading(Quaternion q) {
        return Math.atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
    }

    public static AprilTagLibrary aprilTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(24, "RedAlliance",
                        8, new VectorF(62.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(20, "BlueAlliance",
                        8, new VectorF(62.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))

                .build();
    }
    public static Pose2d relocalize(List<AprilTagDetection> detections, double headingRad, Telemetry telemetry) {
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();

        String numFormat = "%.2f";

        // Negative because the robot is rotating, not the field
        double flippedHeading = -headingRad;

        double finalX = 0;
        double finalY = 0;

        for (AprilTagDetection detection : detections) {
            Pose2d tagPose = vectorFToVector2d(detection.metadata.fieldPosition);
            AprilTagPoseFtc ftcPose = detection.ftcPose;

            telemetry.addData("tag name", detection.metadata.name);

            // Tag position in camera frame
            double x_cameraToTag = ftcPose.x;  // inches (FTC uses x right, y down, z forward)
            double y_cameraToTag = ftcPose.y;



            // Turret angle (in radians)
            double theta_turret = Turret.getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER;

            // Calculate camera position in robot frame (robot → camera)
            double x_robotToCamera = TURRET_OFFSET_X + CAMERA_RADIUS * Math.cos(theta_turret);
            double y_robotToCamera = TURRET_OFFSET_Y + CAMERA_RADIUS * Math.sin(theta_turret);

            // Compute tag position in robot frame (robot → tag)
            // Apply turret + camera transform + rotate into robot frame
            double x_botToTag = x_robotToCamera + (x_cameraToTag * Math.cos(flippedHeading) - y_cameraToTag * Math.sin(flippedHeading));
            double y_botToTag = y_robotToCamera + (x_cameraToTag * Math.sin(flippedHeading) + y_cameraToTag * Math.cos(flippedHeading));

            telemetry.addData("x_botToTag", numFormat, x_botToTag);
            telemetry.addData("y_botToTag", numFormat, y_botToTag);

            // Get tag position on field
            double x_tagOnField = tagPose.position.x;  // in inches
            double y_tagOnField = tagPose.position.y;

            telemetry.addData("x_tagOnField", numFormat, x_tagOnField);
            telemetry.addData("y_tagOnField", numFormat, y_tagOnField);

            // Robot position on field = Tag on field - Bot-to-Tag vector
            double x_botOnField = x_tagOnField - x_botToTag;
            double y_botOnField = y_tagOnField - y_botToTag;

            telemetry.addData("x_botOnField", numFormat, x_botOnField);
            telemetry.addData("y_botOnField", numFormat, y_botOnField);

            finalX += x_botOnField;
            finalY += y_botOnField;

            telemetry.addLine();
        }

        if (finalX == 0)
            return null;

        // Return average pose from all visible tags
        return new Pose2d(finalX / detections.size(), finalY / detections.size(), headingRad);
    }

    public RelocalizationCommand(MecanumDrive drive,  RRSubsystem rrSubsystem, AprilTagProcessor aprilTagProcessor, Telemetry telemetry) {
        this.drive = drive;
        this.rrSubsystem = rrSubsystem;
        this.aprilTagProcessor = aprilTagProcessor;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {


        //return null;
    }

    @Override
    public void execute() {
        Pose2d newPose = relocalize(aprilTagProcessor.getDetections(), rrSubsystem.getYawRadians(), telemetry);
        if (newPose != null) {
            drive.localizer.setPose(newPose);
            telemetry.addData("relocalized using apriltags ", newPose);
        } else {
            drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, Math.toRadians((rrSubsystem.getYawDegrees() + 360) % 360)));
            telemetry.addData("relocalized using imu ", (rrSubsystem.getYawDegrees() + 360) % 360);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {


    }
}

