package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.CAMERA_RADIUS;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DecodeBot.Util.vectorFToPose2d;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
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
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

public class Camera extends SubsystemBase {

    PIDController yawController, forwardController;

    private static final int IMG_HEIGHT = 720;
    private static final int IMG_WIDTH = 1280;
    public static double yawPower, forwardPower;

    List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();

    // === CAMERA ENUM ===
    public enum ActiveCamera {
        INTAKE_GREEN,
        INTAKE_PURPLE,
        TURRET
    }

    // === HARDWARE AND PROCESSORS ===
    private VisionPortal visionPortal;
    private static AprilTagProcessor aprilTagProcessor;
    private AprilTagDetection desiredTag = null;
    private WebcamName intakeWebcam, turretWebcam;
    private CameraName switchableCamera;

    ColorBlobLocatorProcessor greenLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(//ColorRange.BLUE
                    new ColorRange(
                            ColorSpace.YCrCb,
                            new Scalar( 16,   0, 0),
                            new Scalar(200, 110, 110))
            ).setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-.75, .5, .75, -1))
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(20)                               // Smooth the transitions between different colors in image
            .setErodeSize(10)
            .build();

    ColorBlobLocatorProcessor purpleLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(//ColorRange.BLUE
                    new ColorRange(
                            ColorSpace.YCrCb,
                            new Scalar( 16,   0, 0),
                            new Scalar(200, 110, 110))
            ).setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-.75, .5, .75, -1))
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(20)                               // Smooth the transitions between different colors in image
            .setErodeSize(10)
            .build();

    private ActiveCamera currentCamera = ActiveCamera.TURRET;

    // === CONSTANTS (adjust for your bot) ===

    // === CONSTRUCTOR ===
    public Camera(HardwareMap hardwareMap) {
        intakeWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        turretWebcam = hardwareMap.get(WebcamName.class, "Webcam 2");

        aprilTagProcessor = new AprilTagProcessor.Builder().build();


        // Start with the turret camera
        switchableCamera = turretWebcam;

        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTagProcessor)
                .addProcessor(purpleLocator)
                .addProcessor(greenLocator)
                .build();

        // Disable intake camera processors initially
        visionPortal.setProcessorEnabled(greenLocator, false);
    }

    public void periodic(){
        if (visionPortal.getProcessorEnabled(greenLocator) || visionPortal.getProcessorEnabled(purpleLocator)){

            if(visionPortal.getProcessorEnabled(greenLocator)){
                blobs = greenLocator.getBlobs();
            }
            else if (visionPortal.getProcessorEnabled(purpleLocator)){
                blobs = purpleLocator.getBlobs();
            }


            yawPower = yawController.calculate(getBlobCenterX(),IMG_WIDTH/2);
            forwardPower = yawController.calculate(getBlobCenterY(),IMG_HEIGHT/4);
        }
        else{
            yawPower = 0;
            forwardPower = 0;
        }


    }

    // === CAMERA SWITCHING ===
    public void switchCamera(ActiveCamera camera) {
        currentCamera = camera;

        switch (camera) {
            case INTAKE_GREEN:
                visionPortal.setActiveCamera(intakeWebcam);
                visionPortal.setProcessorEnabled(greenLocator, true);
                visionPortal.setProcessorEnabled(purpleLocator,false);
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                break;

            case INTAKE_PURPLE:
                visionPortal.setActiveCamera(intakeWebcam);
                visionPortal.setProcessorEnabled(greenLocator, false);
                visionPortal.setProcessorEnabled(purpleLocator,true);
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);

            case TURRET:
                visionPortal.setActiveCamera(turretWebcam);
                visionPortal.setProcessorEnabled(greenLocator, false);
                visionPortal.setProcessorEnabled(purpleLocator,false);
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

    public List<AprilTagDetection> getCurrentAprilTagDetections(){
        return aprilTagProcessor.getDetections();
    }

    public AprilTagDetection getDesiredTag(List<AprilTagDetection> currentDetections, int desiredTagID) {
        boolean targetFound;
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTagID < 0) || (detection.id == desiredTagID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
//                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                return null;
                // This tag is NOT in the library, so we don't have enough information to track to it.
//                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        return desiredTag;
    }

    public double getBlobCenterX(){
        double cx;

        if (!blobs.isEmpty()) {
            ColorBlobLocatorProcessor.Blob BigBlob = blobs.get(0);

            cx = BigBlob.getBoxFit().center.x;

        }
        else{
            cx = IMG_WIDTH/2;
        }

        return cx;
    }

    public double getBlobCenterY(){
        double cy;

        if (!blobs.isEmpty()) {
            ColorBlobLocatorProcessor.Blob BigBlob = blobs.get(0);

            cy = BigBlob.getBoxFit().center.y;

        }
        else{
            cy = IMG_HEIGHT/4;
        }

        return cy;
    }

}
