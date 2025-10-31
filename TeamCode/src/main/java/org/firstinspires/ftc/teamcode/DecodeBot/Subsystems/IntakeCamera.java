package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

public class IntakeCamera extends SubsystemBase {

    PIDController yawController, forwardController;

    private static final int IMG_HEIGHT = 720;
    private static final int IMG_WIDTH = 1280;
    public static double yawPower, forwardPower;

    private AprilTagProcessor aprilTagProcessor;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;

    int desiredTagId;

    List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();

    public WebcamName intakeWebcam, turretWebcam;

    VisionPortal visionPortal;

    ColorBlobLocatorProcessor greenLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(//ColorRange.BLUE
                    new ColorRange(
                            ColorSpace.YCrCb,
                            new Scalar( 16,   0, 0),
                            new Scalar(200, 110, 110))
            )         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-.75, .5, .75, -1))
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(20)                               // Smooth the transitions between different colors in image
            .setErodeSize(10)
            .build();

    public IntakeCamera(HardwareMap hardwareMap){

        intakeWebcam = hardwareMap.get(WebcamName.class, "Webcam 1"); //WEBCAM 1
        turretWebcam = hardwareMap.get(WebcamName.class, "Webcam 2");

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(intakeWebcam, turretWebcam);


        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCamera(switchableCamera)
                .addProcessors(aprilTagProcessor, greenLocator)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .enableLiveView(true)
//                .addProcessor(frontCameraStream)
                .build();

        //TODO: set a default camera to be on on init

    }

    @Override
    public void periodic(){
        //List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (visionPortal.getProcessorEnabled(greenLocator)){
            blobs = greenLocator.getBlobs();

            yawPower = yawController.calculate(getGreenCenterX(),IMG_WIDTH/2);
            forwardPower = yawController.calculate(getGreenCenterY(),IMG_HEIGHT/4);
        }
        else{
            yawPower = 0;
            forwardPower = 0;
        }

    }


    public void switchCamera(String c){
        if (c == "Intake"){
            visionPortal.setActiveCamera(intakeWebcam);
            visionPortal.setProcessorEnabled(greenLocator, true);
            visionPortal.setProcessorEnabled(aprilTagProcessor, false);
        }
        else if(c=="Turret"){
            visionPortal.setActiveCamera(turretWebcam);
            visionPortal.setProcessorEnabled(greenLocator, false);
            visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        }
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

    public List<AprilTagDetection> getCurrentAprilTagDetections(){
        return aprilTagProcessor.getDetections();
    }

    public double getGreenCenterX(){
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

    public double getGreenCenterY(){
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
