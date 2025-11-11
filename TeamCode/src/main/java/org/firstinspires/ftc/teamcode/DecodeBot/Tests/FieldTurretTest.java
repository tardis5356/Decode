package org.firstinspires.ftc.teamcode.DecodeBot.Tests;

import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.DecodeAuto.savedPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.getCurrentPosition;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.mT;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.turretFlipping;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Commands.TurretFlipCommand;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;


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


    // === Field variables ===



    // relocalize every 5 seconds
    private static final double RELOCALIZE_INTERVAL_SEC = 1;

    // Dashboard telemetry
    private final MultipleTelemetry dashboardTelemetry =
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void initialize() {
        telemetry = dashboardTelemetry;
        CommandScheduler.getInstance().reset();

        if (savedPos == null){
            savedPos = new Pose2d(0,0,0);
        }



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


        double maxAngleDeg = 200;
//        new Trigger(() -> Math.abs(Math.toDegrees(getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER)
//) > maxAngleDeg && !turretFlipping)
//                .whenActive(new TurretFlipCommand(10, turret));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(() -> camera.setObeliskMotif());
        telemetry.addData("Turret Heading(DEG)", Math.toDegrees(mT.getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER));
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



          //  drive.localizer.setPose(camera.getRelocalizedPose(drive));


        // === Telemetry ===
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("X", pose.position.x);
        telemetry.addData("Y", pose.position.y);
        telemetry.addData("ActualTurretPos (DEG)", Math.toDegrees(mT.getCurrentPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER));
        telemetry.addData("targetPosition (DEG)", Math.toDegrees(turret.getTargetPosition() * TURRET_TICK_TO_RADIAN_MULTIPLIER));
        telemetry.addData("Alliance", GlobalVariables.aColor);
        telemetry.addData("Flip Active", turret.turretFlipping);
//        telemetry.addData("Target Turret Angle (deg)", Math.toDegrees(desiredTurretAngleRobot));

        telemetry.addData("Relocalize Timer", relocalizeTimer.seconds());


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