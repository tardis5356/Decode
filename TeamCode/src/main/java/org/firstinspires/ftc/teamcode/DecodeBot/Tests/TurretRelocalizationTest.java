package org.firstinspires.ftc.teamcode.DecodeBot.Tests;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_DEGREE_TO_TICK_MULTIPLIER;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.PIDDisabled;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.getCurrentPosition;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.tracking;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Commands.RelocalizationCommand;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Config
@TeleOp(name = "TurretRelocalizationTest", group = "AGen1")

public class TurretRelocalizationTest extends CommandOpMode {
    private GamepadEx driver1, driver2;

    int desiredTagID;

    public static Boolean targetFound = false;
    public double turretBearing;
    AprilTagDetection detectedTag = null;

    Turret turret;

    private long lastDetectionTime = 0; // nanoseconds

    //private IntakeInCommand intakeInCommand;

    //drivetrain motors and variables
    //DcMotorEx is an expanded version of the DcMotor variable that gives us more methods.
    //For example, stop and reset encoder.


    //Forward and back power, Left and right power, rotation power.
    //All are then added and subtracted in different ways for each drive motor


    //multipliers applied to the sum of the above variables to evenly change the speed of the drivetrain


    //CURRENT_SPEED_MULTIPLIER is the actual multiplier applied to the drive train power. It is set to either the fast or slow multipliers


    static int imgHeight = 600;//480
    static int imgWidth = 800;//640
    //below we create a new object instance of all the subsystem classes
    //gripper


    //wrist


    private DcMotorEx mFL, mFR, mBL, mBR;
    double FB, LR, Rotation;
    private MecanumDrive drive;
    private IMU imu;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static double fps;

    public static VisionPortal portal;
    int visionOutputPosition = 1;


    FtcDashboard dashboard = FtcDashboard.getInstance();



    @Override
    //stuff that is ran when you click init at the start of teleop.
    public void initialize() {



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
               CommandScheduler.getInstance().reset();

               mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        imu = hardwareMap.get(IMU.class, "imu");

        mBL.setDirection(DcMotorSimple.Direction.REVERSE);
        mFL.setDirection(DcMotorSimple.Direction.REVERSE);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        GlobalVariables.aColor = "red";

        turret = new Turret(hardwareMap);



        telemetry.setMsTransmissionInterval(1);




        portal = new VisionPortal.Builder()
                .addProcessors(aTagP)
                .setCameraResolution(new Size(imgWidth, imgHeight))

                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        portal.setProcessorEnabled(aTagP, true);




        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B))
                .whenActive(new InstantCommand(() -> GlobalVariables.aColor = "red"));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.A))
                .whenActive(new InstantCommand(() -> GlobalVariables.aColor = "blue"));


    }


    @Override
    public void run() {
        super.run();

        if (GlobalVariables.aColor == "red") {
            desiredTagID = 24;
        }
        if (GlobalVariables.aColor == "blue") {
            desiredTagID = 20;
        }

        List<AprilTagDetection> currentDetections = aTagP.getDetections();
        boolean tagThisFrame = false;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == desiredTagID) {
                detectedTag = detection;
                targetFound = true;
                tagThisFrame = true;
                PIDDisabled = false;
                lastDetectionTime = System.nanoTime();
                break;
            }
        }


        if (!tagThisFrame) {
            long timeSinceLast = System.nanoTime() - lastDetectionTime;
            if (timeSinceLast > 200_000_000) { // 200ms timeout
                targetFound = false;
                detectedTag = null;
            }
        }

        if (targetFound && detectedTag != null) {
            double bearingError = detectedTag.ftcPose.bearing;
            double tickOffset = bearingError * TURRET_DEGREE_TO_TICK_MULTIPLIER;
            turret.setTargetPosition(turret.getCurrentPosition() - tickOffset);

            telemetry.addData("Bearing error", bearingError);
            telemetry.addData("Target pos", turret.getTargetPosition());
            telemetry.addData("Turret encoder", turret.getCurrentPosition());
            telemetry.addData("PID output", turret.getCurrentMotorPower());
        } else {

            PIDDisabled = true;
        }


        Rotation = cubicScaling(-gamepad1.right_stick_x) * 0.5;
        FB = cubicScaling(gamepad1.left_stick_y);
        LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;

        double mFLPower = FB - LR + Rotation;
        double mFRPower = FB - LR - Rotation;
        double mBLPower = FB + LR + Rotation;
        double mBRPower = FB + LR - Rotation;


        mFL.setPower(mFLPower);
        mFR.setPower(mFRPower);
        mBL.setPower(mBLPower);
        mBR.setPower(mBRPower);


        telemetry.addData("Target Found", targetFound);
        telemetry.addData("Time Since Last (ms)", (System.nanoTime() - lastDetectionTime) / 1e6);
        telemetry.addData("FPS", portal.getFps());
        telemetry.update();
    }

    private double cubicScaling(float joystickValue) {
        //store 5% of the joystick value + 95% of the joystick value to the 3rd power
        double v = 0.05 * joystickValue + 0.95 * Math.pow(joystickValue, 3);
        if (joystickValue > 0.02)
            //if the joystick is positive, return positive .1 + the stored value
            return 0.1 + v;
        else if (joystickValue < -0.02)
            //if the joystick is negative, return -.1 plus the stored value
            return -0.1 + v;
            // theres a range where this won't do either, which is a good counter against stick drift (because you can never escape stick drift)
        else
            return 0;
    }

    private double driftLock(float stickValue) {
        if (stickValue > 0.02 || stickValue < -.02) {
            return stickValue;
        } else {
            return 0;
        }
    }


}
