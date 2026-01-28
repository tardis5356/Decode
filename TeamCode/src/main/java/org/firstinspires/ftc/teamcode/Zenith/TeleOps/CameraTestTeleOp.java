package org.firstinspires.ftc.teamcode.Zenith.TeleOps;


//import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.tracking;

import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.DecodeAuto.savedPos;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.aColor;

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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Zenith.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Zenith.Commands.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BellyPan;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BrakePad;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "CameraTestTeleOp", group = "AGen1")

public class CameraTestTeleOp extends CommandOpMode {
    public static boolean flyMode = true;

    double mFLPower;
    double mFRPower;
    double mBLPower;
    double mBRPower;
    //Forward and back power, Left and right power, rotation power.
    //All are then added and subtracted in different ways for each drive motor
    double FB, LR, Rotation;

    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    //below we create a new object instance of all the subsystem classes
    int visionOutputPosition = 1;
    ElapsedTime Timer = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    //gamepads
    //GamepadEx is an extended object version of gamepads that has more organized input checks that we use in triggers.
    private GamepadEx driver1, driver2;
    //drivetrain motors and variables
    //DcMotorEx is an expanded version of the DcMotor variable that gives us more methods.
    //For example, stop and reset encoder.
    private DcMotorEx mFL, mFR, mBL, mBR;
    //intake

    //lift
    private BellyPan bellyPan;
    //turret
    private Turret turret;
    //storage
    private Storage storage;
    //shooter
//    private Shooter shooter;
    //breakpad
    private BrakePad brakePad;
    //Cameras
    private Camera camera;
    //Roadrunner
    private RRSubsystem rrSubsystem;

    private MecanumDrive drive;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    public static boolean intaketoggle = true;


    public GoBildaPinpointDriver driver;

    @Override
    //stuff that is ran when you click init at the start of teleop.
    public void initialize() {


        {
            GlobalVariables.inAuto = false;
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            //Removes previous Commands from scheduler
            //We call it at the start of TeleOp as it clears lingering autocommands that make the intake freak out
            //Make sure it is not called in a loop since it will clear all the triggers every frame. Be very careful. It is a kill switch.
            CommandScheduler.getInstance().reset();

            //sets the digital position of the robot to intake for the deposit to state command

Timer.reset();
            //init controllers
            driver1 = new GamepadEx(gamepad1);
            driver2 = new GamepadEx(gamepad2);

            turret = new Turret(hardwareMap);

            if (savedPos == null) {
                savedPos = new Pose2d(0, 0, Math.toRadians(0));
                turret.mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }

            turret.mT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            storage = new Storage(hardwareMap);

            // intake = new Intake(hardwareMap);

            bellyPan = new BellyPan(hardwareMap);

//            shooter = new Shooter(hardwareMap);
//            shooter.spinning = true;

            brakePad = new BrakePad(hardwareMap);

            camera = new Camera(hardwareMap);

            rrSubsystem = new RRSubsystem(hardwareMap);

            drive = new MecanumDrive(hardwareMap, savedPos);

            //map motors
            mFL = hardwareMap.get(DcMotorEx.class, "mFL");
            mFR = hardwareMap.get(DcMotorEx.class, "mFR");
            mBL = hardwareMap.get(DcMotorEx.class, "mBL");
            mBR = hardwareMap.get(DcMotorEx.class, "mBR");

            driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




            telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.


        }



        //Swapper
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenInactive(() -> turret.manualOffset -= 350);

        //Back Gate
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenInactive(() -> turret.manualOffset += 350);


        //Engage/Disengage PTO
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.BACK))
                .toggleWhenActive(bellyPan::engagePTO, bellyPan::disEngagePTO);

        //BreakPad
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.START))
                .toggleWhenActive(new InstantCommand(brakePad::deploy), new InstantCommand(brakePad::retract));


//        Shooter mode
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.START))
                .toggleWhenActive(new InstantCommand(() -> flyMode = true), new InstantCommand(() -> flyMode = false));


        //automated targetting on/off
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.BACK))
                //TODO: swap this back to true after testing
                .toggleWhenActive(new InstantCommand(turret::disablePID), new InstantCommand(() -> turret.enablePID()));



        {



            new Trigger(() -> gamepad2.touchpad)
                    .whenActive(new InstantCommand(()->drive.localizer.setPose(camera.getRelocalizedPose(drive, telemetry))));


            new Trigger(() -> gamepad2.y)
                    .toggleWhenActive(new InstantCommand(() -> aColor = "red"), new InstantCommand(() -> aColor = "blue"));




        }
    }

    //this is the main run loop
    public void run() {
        super.run();


        drive.localizer.update();


camera.getRelocalizedPose(drive, telemetry);

        turret.updateTurretTracking(drive, telemetry);

        Rotation = -cubicScaling(gamepad1.left_trigger - gamepad1.right_trigger) * 0.75;
        FB = -cubicScaling(gamepad1.left_stick_y);
        LR = -cubicScaling(-gamepad1.left_stick_x) * 1.2;

        //defines the powers for the motors based on the stick inputs (trust i've written this so many times)


        //actually sets the motor powers
        mFLPower = FB + LR + Rotation;
        mFRPower = FB - LR - Rotation;
        mBLPower = FB - LR + Rotation;
        mBRPower = FB + LR - Rotation;

        mFL.setPower(mFLPower);
        mFR.setPower(mFRPower);
        mBL.setPower(mBLPower);
        mBR.setPower(mBRPower);

        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("time", Timer.seconds());

 camera.getATagRobotHeading(turret, telemetry);
 telemetry.addData("Odometry        X(in)       Y(in)       Heading(deg)\n            ","%.3f\t\t%.3f\t\t%.3f", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
        telemetry.addLine();
        telemetry.addData("FPS", camera.visionPortal.getFps());
//        telemetry.addData("AprilTag#Seen", camera.getCurrentAprilTagDetections().size());
        telemetry.addLine();


        //telemetry.addData("pinpointYawScalar", driver.getYawScalar());


        // telemetry.addData("position",drive.localizer.getPose());


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

    public static enum shootModes {
        FLY,
        STORE_MIDDLE,
        STORE_ONE_FOR_LAST,
        STORE_ONE_FOR_SECOND,
        MANUAL
    }


}

