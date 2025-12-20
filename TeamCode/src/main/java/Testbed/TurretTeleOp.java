package Testbed;

//import com.arcrobotics.ftclib.hardware.motors.Motor;
import android.util.Size;

import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

@TeleOp(name="Turret_POC_ONLY", group = "Testbed")
@Disabled
public class TurretTeleOp extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(TurretTeleOp.class);
    DcMotor yaw;

    Servo LED;

    DcMotor mBL, mBR, mFL, mFR;

    boolean targetFound = false;

     AprilTagDetection detectedTag;


     int desiredTagID;

    static int imgHeight = 896;
    static int imgWidth = 1600;

    double FB, LR, Rotation;

     PDController YawController;
    
     boolean tooFarLeft, tooFarRight;


    @Override
    public void runOpMode()
    {
         YawController = new PDController(.00004,.0000);

        yaw = hardwareMap.get(DcMotor.class, "yaw");

        LED = hardwareMap.get(Servo.class,"LED");

        mBL = hardwareMap.get(DcMotor.class,"mBL");
        mFL = hardwareMap.get(DcMotor.class,"mFL");
        mBR = hardwareMap.get(DcMotor.class,"mBR");
        mFR = hardwareMap.get(DcMotor.class,"mFR");

        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
        mFR.setDirection(DcMotorSimple.Direction.REVERSE);

        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        yaw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        yaw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(aTagP)
                .setCameraResolution(new Size(imgWidth, imgHeight))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();


        waitForStart();

        while(opModeIsActive()||opModeInInit()){

            portal.setProcessorEnabled(aTagP, true);


            if(gamepad1.dpad_up){
                desiredTagID = 24;
            }
            else if(gamepad1.dpad_left){
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
                //  } else {

                // This tag is NOT in the library, so we don't have enough information to track to it.
//                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                //   }
            }

            
//            if(yaw.getCurrentPosition()<=-200){
//                tooFarLeft = true;
//            }
//            else if(yaw.getCurrentPosition()>=200){
//                tooFarRight = true;
//            }

            if(targetFound) {
                //based on turret motor ticks
//                if (tooFarLeft) {
//                    yaw.setPower(-YawController.calculate(yaw.getCurrentPosition(), 150));
//                    if (yaw.getCurrentPosition() >= 150) {
//                        tooFarLeft = false;
//                    }
//                } else if (tooFarRight) {
//                    //based on turret motor ticks
//                    yaw.setPower(-YawController.calculate(yaw.getCurrentPosition(), -150));
//                    if (yaw.getCurrentPosition() <= -150) {
//                        tooFarRight = false;
//                    }
//                } else {
                    //based on camera pixels
                    yaw.setPower(-YawController.calculate(detectedTag.center.x, 800));
                    telemetry.addData("AprilTagCenter", detectedTag.center.x);
//                }

            }

            //yaw.setPower(gamepad2.left_stick_x);


            Rotation = -gamepad1.right_stick_x * 0.5;
            FB = gamepad1.left_stick_y;
            LR = -gamepad1.left_stick_x * 1.2;

            double mFLPower = FB - LR + Rotation;
            double mFRPower = FB - LR - Rotation;
            double mBLPower = FB + LR + Rotation;
            double mBRPower = FB + LR - Rotation;

            mFL.setPower(mFLPower);
            mFR.setPower(mFRPower);
            mBL.setPower(mBLPower);
            mBR.setPower(mBRPower);

            telemetry.addData("Yaw_Position", yaw.getCurrentPosition());
            telemetry.addData("Yaw_Power", yaw.getPower());
            telemetry.addData("Left Stick X", gamepad2.left_stick_x);
         //
            telemetry.update();

        }
    }
}
