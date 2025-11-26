package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.CAMERA_RADIUS;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_RADIANS_PER_TICK;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.aColor;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.motif;
import static org.firstinspires.ftc.teamcode.DecodeBot.Util.vectorFToPose2d;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

public class Camera extends SubsystemBase {

    PIDController yawController, forwardController;

    private static final int IMG_HEIGHT = 480;//720
    private static final int IMG_WIDTH = 640;//1280


    public static double yawPower, forwardPower;

    List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
    public double fx = 545.605 * 56.5/58;//911.942 * (55.6 / 57.4);
    public double fy = 545.605 * 56.5/58;//911.942 * (55.6 / 57.4);
    public double cx = 320, cy = 262.311;

    public static boolean manualExposure;


    // === CAMERA ENUM ===
    public enum ActiveCamera {
        INTAKE_GREEN,
        INTAKE_PURPLE,
        TURRET
    }

    // === HARDWARE AND PROCESSORS ===
    public static VisionPortal visionPortal;
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
        //intakeWebcam = hardwareMap.get(WebcamName.class, "Webcam 2");
        turretWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics(fx, fy, cx, cy).build();

aprilTagProcessor.setDecimation(5);
        // Start with the turret camera
        switchableCamera = turretWebcam;

        visionPortal = new VisionPortal.Builder()
                //.setCamera(switchableCamera)
                .setCamera(turretWebcam)
                .addProcessor(aprilTagProcessor)
                .addProcessor(purpleLocator)
                .addProcessor(greenLocator)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(IMG_WIDTH, IMG_HEIGHT))
                .build();



        // Disable intake camera processors initially
        visionPortal.setProcessorEnabled(purpleLocator, false);
        visionPortal.setProcessorEnabled(greenLocator, false);
    }

    public void periodic(){
//        if (visionPortal.getProcessorEnabled(greenLocator) || visionPortal.getProcessorEnabled(purpleLocator)){
//
//            if(visionPortal.getProcessorEnabled(greenLocator)){
//                blobs = greenLocator.getBlobs();
//            }
//            else if (visionPortal.getProcessorEnabled(purpleLocator)){
//                blobs = purpleLocator.getBlobs();
//            }
//
//
//            yawPower = yawController.calculate(getBlobCenterX(),IMG_WIDTH/2);
//            forwardPower = yawController.calculate(getBlobCenterY(),IMG_HEIGHT/4);
//        }
//        else{
//            yawPower = 0;
//            forwardPower = 0;
//        }
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING && !manualExposure){
            setManualExposure(2, 80);
            manualExposure = true;
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
                break;

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
    public static Pose2d getRelocalizedPose(MecanumDrive drive, Telemetry telemetry) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections.isEmpty()) return drive.localizer.getPose();

        double finalX = 0, finalY = 0;
        double aTagAmount = 0;
        double thetaTurretRad = Turret.getCurrentPosition() * TURRET_RADIANS_PER_TICK;

        for (AprilTagDetection detection : detections) {

            if (detection.metadata.name.contains("Obelisk")) continue;
            // Convert tag metadata to field pose
            Pose2d tagPose = vectorFToPose2d(
                    detection.metadata.fieldPosition, // position on the arpil tag, can implement real number positions
                    detection.metadata.fieldOrientation
                            .toOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS)
                            .secondAngle
            );

            // Camera → Tag translation
            double xCameraToTag = detection.ftcPose.x;
            double yCameraToTag = detection.ftcPose.y + CAMERA_RADIUS;
            telemetry.addData("xCameraToTag", xCameraToTag);
            telemetry.addData("yCameraToTag", yCameraToTag);
            // Rotate relative to turret
            double xTagToTurret = xCameraToTag * Math.cos(-Math.PI / 2 + thetaTurretRad)
                    - yCameraToTag * Math.sin(-Math.PI / 2 + thetaTurretRad);
            double yTagToTurret = xCameraToTag * Math.sin(-Math.PI / 2 + thetaTurretRad)
                    + yCameraToTag * Math.cos(-Math.PI / 2 + thetaTurretRad);

            telemetry.addData("xTagToTurret", xTagToTurret);
            telemetry.addData("yTagToTurret", yTagToTurret);


            // Offset from turret to bot center
            double xTagToBot = xTagToTurret + TURRET_OFFSET_X;
            double yTagToBot = yTagToTurret + TURRET_OFFSET_Y;
            telemetry.addData("xTagToBot", xTagToBot);
            telemetry.addData("yTagToBot", yTagToBot);


            // Tag orientation
            double headingBotOnFieldRad = drive.localizer.getPose().heading.toDouble();
            telemetry.addData("headingBotOnFieldRad", headingBotOnFieldRad);


            // Bot position on field
            double xBotOnField = -(xTagToBot * Math.cos(headingBotOnFieldRad)
                    - yTagToBot * Math.sin(headingBotOnFieldRad));
            double yBotOnField = -(xTagToBot * Math.sin(headingBotOnFieldRad)
                    + yTagToBot * Math.cos(headingBotOnFieldRad));




            xBotOnField += tagPose.position.x;
            yBotOnField += tagPose.position.y;

            telemetry.addData("xBotOnField", xBotOnField);
            telemetry.addData("yBotOnField", yBotOnField);
            telemetry.addLine();
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

    public void setObeliskMotif() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) {
            motif = null;
            return;
        }

        // Filter to only obelisk tags
        List<AprilTagDetection> obeliskTags = detections.stream()
                .filter(d -> d.metadata != null && d.metadata.name.contains("Obelisk"))
                .collect(Collectors.toList());
        if (obeliskTags.isEmpty()) {
            motif = null;
            return;
        }

        // Pick tag based on alliance color and yaw direction
        AprilTagDetection motifAprilTag = null;
        if ("red".equals(aColor)) {
            motifAprilTag = obeliskTags.stream()
                    .filter(d -> d.ftcPose.yaw < 0) // red prefers negative yaw
                    .findFirst()
                    .orElse(null);
        } else if ("blue".equals(aColor)) {
            motifAprilTag = obeliskTags.stream()
                    .filter(d -> d.ftcPose.yaw > 0) // blue prefers positive yaw
                    .findFirst()
                    .orElse(null);
        }

        // Fallback if none matched by yaw
        if (motifAprilTag == null && obeliskTags.size() == 1) {
            motifAprilTag = obeliskTags.get(0);
        }

        // Validate chosen tag
        if (motifAprilTag == null) {
            motif = null;
            return;
        }


        // Assign motif based on AprilTag ID
        switch (motifAprilTag.id) {
            case 21:
                motif= "GPP";
                break;
            case 22:
                motif = "PGP";
                break;
            case 23:
                motif = "PPG";
                break;
            default:
                motif = null;
                break;
        }
    }
//2, 94
    private void setManualExposure(int exposureMS, int gain) {



        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);

            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);


            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);


        }
    }

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */


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
