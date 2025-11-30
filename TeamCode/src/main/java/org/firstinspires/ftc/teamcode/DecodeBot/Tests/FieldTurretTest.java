package org.firstinspires.ftc.teamcode.DecodeBot.Tests;

import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.PenfieldAuto.DecodeAuto.savedPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_RADIANS_PER_TICK;
//import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Camera.manualExposure;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//TODO AprilTag Acquisition Time optimizations
//Run the AprilTag Optimize Exposure Teleop for more fps and preventing whitewashing for camera.
//Lower Resolution
//Add Decimation for shorter processing time (skip undistortion of image if needed)
// close the camera stream
// Make code To Manually Turn on the camera stream only when needed by disabling and enabling the RC preview (called LiveView)
//https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html

@Config
@TeleOp(name = "FieldTurretTest", group = "AGen1")
public class FieldTurretTest extends CommandOpMode {

    private GamepadEx driver1, driver2;
    private DcMotorEx mFL, mFR, mBL, mBR;

    private Turret turret;
    private MecanumDrive drive;
    private Camera camera;

    private ElapsedTime relocalizeTimer = new ElapsedTime();  // timer for relocalization


    // How fast the robot is allowed to be moving (in/s) to relocalize
    public static double MAX_RELOCALIZE_SPEED_IN_PER_SEC = 3;



    // Separate timer for speed deltas
    private ElapsedTime speedTimer = new ElapsedTime();

    // relocalize every 5 seconds
    private static final double RELOCALIZE_INTERVAL_SEC = 1;

    // Dashboard telemetry
    private final MultipleTelemetry dashboardTelemetry =
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void initialize() {
        telemetry = dashboardTelemetry;
        CommandScheduler.getInstance().reset();


            savedPos = new Pose2d(0,0,0);




        // === Hardware init ===
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        mBL.setDirection(DcMotorSimple.Direction.REVERSE);
        mFL.setDirection(DcMotorSimple.Direction.REVERSE);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        turret = new Turret(hardwareMap);
//        turret.mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        turret.mT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        drive = new MecanumDrive(hardwareMap, savedPos);
        camera = new Camera(hardwareMap);



        GlobalVariables.aColor = "red";

        // === Alliance switch controls ===
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B))
                .whenActive(() -> GlobalVariables.aColor = "red");
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.A))
                .whenActive(() -> GlobalVariables.aColor = "blue");

        //relocalize heading with robot facing audience
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenActive(() -> drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y,0)));




        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(() -> camera.setObeliskMotif());
        telemetry.addData("Turret Heading(DEG)", Math.toDegrees(turret.getCurrentPosition() * TURRET_RADIANS_PER_TICK));
        telemetry.addData("DetectAprilTag?", !camera.getCurrentAprilTagDetections().isEmpty());
        telemetry.addLine("Initialized — ready to start!");
        telemetry.update();




        relocalizeTimer.reset();

    }

    @Override
    public void run() {
        super.run();
       drive.localizer.update();

        // === Turret control ===
       turret.updateTurretTracking(drive, telemetry, 200);


        // === Driving Control ===
        double rotation = cubicScaling(-gamepad1.right_stick_x) * 0.5;
        double forward = cubicScaling(gamepad1.left_stick_y);
        double strafe = cubicScaling(-gamepad1.left_stick_x) * 1.2;

        double mFLPower = forward - strafe + rotation;
        double mFRPower = forward - strafe - rotation;
        double mBLPower = forward + strafe + rotation;
        double mBRPower = forward + strafe - rotation;

        mFL.setPower(mFLPower);
        mFR.setPower(mFRPower);
        mBL.setPower(mBLPower);
        mBR.setPower(mBRPower);



        // === Estimate robot speed (in/s) ===
        Pose2d pose = drive.localizer.getPose();
        double dt = speedTimer.seconds();
        double speedInPerSec = 0.0;

        if (dt > 0 && savedPos != null) {
            double dx = pose.position.x - savedPos.position.x;
            double dy = pose.position.y - savedPos.position.y;
            speedInPerSec = Math.hypot(dx, dy) / dt;
        }

        savedPos = pose;
        speedTimer.reset();


        // === Conditional relocalization ===

        boolean slowEnough = speedInPerSec < MAX_RELOCALIZE_SPEED_IN_PER_SEC;
        boolean timeElapsed = relocalizeTimer.seconds() > RELOCALIZE_INTERVAL_SEC;
        boolean seesTag = !camera.getCurrentAprilTagDetections().isEmpty();

//        if (slowEnough && timeElapsed && seesTag) {
//            Pose2d relocalizedPose = camera.getRelocalizedPose(drive, telemetry);
//            drive.localizer.setPose(relocalizedPose);
//            relocalizeTimer.reset();
//        }


        // === Telemetry ===
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("X", pose.position.x);
        telemetry.addData("Y", pose.position.y);

        for (AprilTagDetection detection : camera.getCurrentAprilTagDetections()){
            telemetry.addData("ATagYaw", detection.ftcPose.yaw);
            telemetry.addData("SensedATagPos",detection.ftcPose.x);
            telemetry.addData("SensedATagPos",detection.ftcPose.y);
            telemetry.addLine();
        }


       // telemetry.addData("ActualTurretPos (DEG)", Math.toDegrees(mT.getCurrentPosition() * TURRET_RADIANS_PER_TICK));
        telemetry.addData("targetPosition (DEG)", Math.toDegrees(turret.getTargetPosition() * TURRET_RADIANS_PER_TICK));

        telemetry.addData("Alliance", GlobalVariables.aColor);
        telemetry.addData("FPS", camera.visionPortal.getFps());
        telemetry.addData("Turret Motor Power", turret.getCurrentMotorPower());
//        telemetry.addData("Target Turret Angle (deg)", Math.toDegrees(de
//        siredTurretAngleRobot));

        telemetry.addData("Relocalize Timer", relocalizeTimer.seconds());
telemetry.addData("manualExposure", manualExposure);

        telemetry.update();
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