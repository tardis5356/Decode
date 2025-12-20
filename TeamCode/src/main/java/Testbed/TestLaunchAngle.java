package Testbed;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="TestLaunchAngle", group="Testbed")
@Disabled
public class TestLaunchAngle extends LinearOpMode {

    //Declare driveTrain motors
    DcMotor mBL, mBR, mFL, mFR;

    //boolean for if the apriltag is or isnt in camera view
    boolean targetFound = false;

    //declare variable that holds april tag data
    AprilTagDetection detectedTag;

    //declare variables for distance from targer, height of target, and initial velocity of artifact
    double d, h = .8, v = 8;

    double launchAngle;

    Servo sH;

    int desiredTagID;

    static int imgHeight = 896;
    static int imgWidth = 1600;

    double FB, LR, Rotation;

    @Override
    public void runOpMode() {

        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mFR = hardwareMap.get(DcMotor.class, "mFR");

        sH = hardwareMap.get(Servo.class, "sH");

        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
        mFR.setDirection(DcMotorSimple.Direction.REVERSE);

        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(aTagP)
                .setCameraResolution(new Size(imgWidth, imgHeight))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();


        waitForStart();

        while (opModeIsActive()) {
            portal.setProcessorEnabled(aTagP, true);


            if (gamepad1.dpad_up) {
                desiredTagID = 24;
            } else if (gamepad1.dpad_left) {
                desiredTagID = 20;
            }

            //AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();


            //TODO: Swapped this from getDetections. Must be tested.
            List<AprilTagDetection> currentDetections = aTagP.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                //  if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (/*(desiredTagID < 0) ||*/ (detection.id == desiredTagID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    detectedTag = detection;
                    break;  // don't look any further.
                } else {
                    targetFound = false;
                    // This tag is in the library, but we do not want to track it right now.
//                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }

            }

            if(targetFound) {
                d=Math.sqrt(detectedTag.rawPose.x*detectedTag.rawPose.x+detectedTag.rawPose.y*detectedTag.rawPose.y);

                //BIG BEAUTIFUL EQUATION

                // Theta = arctan( (19.62*d*v^2) - sqrt( (19.62*d*v^2)^2 - 4(9.81*d)^2 ((9.81*d)^2 + 19.6*h*v^2) ) / 2(9.81*d)^2 )

                launchAngle = Math.toDegrees(  Math.atan(( (19.62*d*Math.pow(v,2)) - Math.sqrt( Math.pow((19.62*d*Math.pow(v,2)),2) - 4*Math.pow(9.81*d,2) * (Math.pow(9.81*d,2)+19.6*h*Math.pow(v,2)) ) ) / 2*Math.pow(9.81*d,2) )  );

                sH.setPosition(launchAngle/90);
            }
            else{
                sH.setPosition(0);
            }
        }

    }
}
