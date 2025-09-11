package org.firstinspires.ftc.teamcode.DecodeBot.Commands;

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
import org.openftc.apriltag.AprilTagPose;

import java.util.ArrayList;
import java.util.List;

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
    public static Pose2d relocalize6(List<AprilTagDetection> detections, double headingRad, Telemetry telemetry) {
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();

        String numFormat = "%.2f";

        /*
        This method assumes we are using the camera in the back of the robot

        **ALL POSITIONS ARE TO CENTERS OF ELEMENTS**

        1. Bot location vs Tag
        2. Tag vs Field
        3. Bot vs Field

        TODO: make case to flip x axis when quaternion x is 0 (for audience wall tags)

         */

        // flip heading because this is an inverse transformation (coordinate system isn't rotating, bot is rotating)
        double Heading = headingRad;

        double finalX = 0;
        double finalY = 0;

        for (AprilTagDetection detection : detections) {
//            if(detection.metadata.fieldOrientation.x == 0){} // if tag is on wall


            Pose2d tagPose = vectorFToVector2d(detection.metadata.fieldPosition);
          //  Pose2d tagPose = vectorFToVector2d(aprilTagLibrary().lookupTag(detection.metadata.id).fieldPosition);
            AprilTagPoseFtc ftcPose = detection.ftcPose;

            telemetry.addData("tag name", detection.metadata.name);

            // 1. Bot location vs Tag
            double x_camera = ftcPose.x;
            double y_camera = ftcPose.y;

            double x_botToTag = (y_camera + /*Turret_WEBCAM_X_OFFSET equation*/) * Math.cos(headingRad - Turret.getCurrentPosition() * BotPositions.TURRET_TICK_TO_DEGREE_MULTIPLIER) - x_camera * Math.sin(headingRad - Turret.getCurrentPosition() * BotPositions.TURRET_TICK_TO_DEGREE_MULTIPLIER);
            double y_botToTag = -(y_camera + /*Turret_WEBCAM_X_OFFSET equation*/) * Math.sin(headingRad - Turret.getCurrentPosition() * BotPositions.TURRET_TICK_TO_DEGREE_MULTIPLIER) - x_camera * Math.cos(headingRad - Turret.getCurrentPosition() * BotPositions.TURRET_TICK_TO_DEGREE_MULTIPLIER);

            telemetry.addData("x_botToTag", numFormat, x_botToTag);
            telemetry.addData("y_botToTag", numFormat, y_botToTag);

            // 2. Tag vs Field
            double x_tagInRR = tagPose.position.x;
            double y_tagInRR = tagPose.position.y;

            telemetry.addData("x_tagInRR", numFormat, x_tagInRR);
            telemetry.addData("y_tagInRR", numFormat, y_tagInRR);

            // 3. Bot vs Field
            double x_botToField = x_botToTag + x_tagInRR;
            double y_botToField = y_botToTag + y_tagInRR;

            telemetry.addData("x_botToField", numFormat, x_botToField);
            telemetry.addData("y_botToField", numFormat, y_botToField);

            finalX += x_botToField;
            finalY += y_botToField;

            telemetry.addLine();
        }
        if (finalX == 0)
            return null;
        else
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


        return null;
    }

    @Override
    public void execute() {
        Pose2d newPose = relocalize6(aprilTagProcessor.getDetections(), rrSubsystem.getYawRadians(), telemetry);
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

