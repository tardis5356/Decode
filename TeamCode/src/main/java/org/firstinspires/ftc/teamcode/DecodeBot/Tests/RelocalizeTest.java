package org.firstinspires.ftc.teamcode.DecodeBot.Tests;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name = "RelocalizeTest", group = "Test")
public class RelocalizeTest extends OpMode {

    private DcMotorEx mFL, mFR, mBL, mBR;
    private MecanumDrive drive;
    private RRSubsystem rrSubsystem;
    private Turret turret ;

    public double fx = 903.5, fy =903.5, cx =  640, cy = 360;

    private AprilTagProcessor aTagP = new AprilTagProcessor.Builder().setLensIntrinsics(fx, fy, cx, cy).build();
    private VisionPortal portal;

    @Override
    public void init() {
        // Hardware
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");


        // Reverse motors if needed
        mFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mBL.setDirection(DcMotorSimple.Direction.REVERSE);

        rrSubsystem = new RRSubsystem(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
turret = new Turret(hardwareMap);

        // Vision
        portal = new VisionPortal.Builder()
                .addProcessors(aTagP)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();



        portal.setProcessorEnabled(aTagP, true);

        telemetry.addLine("Init complete. Ready to run!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Simple mecanum drive
        double FB = -gamepad1.left_stick_y;
        double LR = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x * 0.5;

        mFL.setPower(FB + LR + rot);
        mFR.setPower(FB - LR - rot);
        mBL.setPower(FB - LR + rot);
        mBR.setPower(FB + LR - rot);

     //   drive.updatePoseEstimate();

        // Get detections
        List<AprilTagDetection> detections = aTagP.getDetections();

        // Relocalize
        Pose2d relocalizedPose = FieldTurretTest.relocalize(
                detections,
                Math.toRadians(rrSubsystem.getYawDegrees()),
                telemetry
        );

        // Show results
        telemetry.addData("Detections", detections.size());
        if (relocalizedPose != null) {
            telemetry.addData("Relocalized X", relocalizedPose.position.x);
            telemetry.addData("Relocalized Y", relocalizedPose.position.y);
            telemetry.addData("Relocalized Heading (deg)", Math.toDegrees(relocalizedPose.heading.toDouble()));
        } else {
            telemetry.addLine("No tags detected");
        }

        telemetry.addData("Raw IMU yaw", rrSubsystem.getYawDegrees());
        telemetry.update();
    }
}