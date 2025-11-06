package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.CAMERA_RADIUS;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DecodeBot.Util.vectorFToPose2d;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

public class Camera extends SubsystemBase {

    // === CAMERA ENUM ===
    public enum ActiveCamera {
        INTAKE,
        TURRET
    }

    // === HARDWARE AND PROCESSORS ===
    private VisionPortal visionPortal;
    private static AprilTagProcessor aprilTagProcessor;
    private ColorBlobLocatorProcessor greenLocator;

    private WebcamName intakeWebcam, turretWebcam;
    private CameraName switchableCamera;

    private ActiveCamera currentCamera = ActiveCamera.TURRET;

    // === CONSTANTS (adjust for your bot) ===

    // === CONSTRUCTOR ===
    public Camera(HardwareMap hardwareMap) {
        intakeWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        turretWebcam = hardwareMap.get(WebcamName.class, "Webcam 2");

        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        greenLocator = new ColorBlobLocatorProcessor.Builder().build();

        // Start with the turret camera
        switchableCamera = turretWebcam;

        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTagProcessor)
                .addProcessor(greenLocator)
                .build();

        // Disable intake camera processors initially
        visionPortal.setProcessorEnabled(greenLocator, false);
    }

    // === CAMERA SWITCHING ===
    public void switchCamera(ActiveCamera camera) {
        currentCamera = camera;

        switch (camera) {
            case INTAKE:
                visionPortal.setActiveCamera(intakeWebcam);
                visionPortal.setProcessorEnabled(greenLocator, true);
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                break;

            case TURRET:
                visionPortal.setActiveCamera(turretWebcam);
                visionPortal.setProcessorEnabled(greenLocator, false);
                visionPortal.setProcessorEnabled(aprilTagProcessor, true);
                break;
        }
    }

    public ActiveCamera getCurrentCamera() {
        return currentCamera;
    }

    // === RELOCALIZATION ===
    public static Pose2d getRelocalizedPose(MecanumDrive drive) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections.isEmpty()) return drive.localizer.getPose();

        double finalX = 0, finalY = 0;
        double aTagAmount = 0;
        double thetaTurretRad = Turret.getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER;

        for (AprilTagDetection detection : detections) {

            if (detection.metadata.name.contains("Obelisk")) continue;
            // Convert tag metadata to field pose
            Pose2d tagPose = vectorFToPose2d(
                    detection.metadata.fieldPosition,
                    detection.metadata.fieldOrientation
                            .toOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS)
                            .secondAngle
            );

            // Camera → Tag translation
            double xCameraToTag = detection.ftcPose.x;
            double yCameraToTag = detection.ftcPose.y + CAMERA_RADIUS;

            // Rotate relative to turret
            double xTagToTurret = xCameraToTag * Math.cos(-Math.PI / 2 + thetaTurretRad)
                    - yCameraToTag * Math.sin(-Math.PI / 2 + thetaTurretRad);
            double yTagToTurret = xCameraToTag * Math.sin(-Math.PI / 2 + thetaTurretRad)
                    + yCameraToTag * Math.cos(-Math.PI / 2 + thetaTurretRad);

            // Offset from turret to bot center
            double xTagToBot = xTagToTurret + TURRET_OFFSET_X;
            double yTagToBot = yTagToTurret + TURRET_OFFSET_Y;

            // Tag orientation
            double headingBotOnFieldRad = drive.localizer.getPose().heading.toDouble();

            // Bot position on field
            double xBotOnField = -(xTagToBot * Math.cos(headingBotOnFieldRad)
                    - yTagToBot * Math.sin(headingBotOnFieldRad));
            double yBotOnField = -(xTagToBot * Math.sin(headingBotOnFieldRad)
                    + yTagToBot * Math.cos(headingBotOnFieldRad));

            xBotOnField += tagPose.position.x;
            yBotOnField += tagPose.position.y;

            finalX += xBotOnField;
            finalY += yBotOnField;
            aTagAmount++;
        }

        if (aTagAmount == 0) return drive.localizer.getPose();

        return new Pose2d(finalX / aTagAmount, finalY / aTagAmount, drive.localizer.getPose().heading.toDouble());
    }


}
