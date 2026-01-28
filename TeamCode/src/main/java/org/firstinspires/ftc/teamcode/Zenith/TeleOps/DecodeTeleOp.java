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
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Zenith.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Zenith.Commands.IntakeToggleCommand;
import org.firstinspires.ftc.teamcode.Zenith.Commands.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BrakePad;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BellyPan;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "Decode Teleop", group = "AGen1")

public class DecodeTeleOp extends CommandOpMode {
    public static boolean flyMode = true;
    //private DcMotorEx liftEncoder;

    //This is just a boolean used for telemetry to see if we took in the incorrect sample color

    //TODO:
   
    //multipliers applied to the sum of the above variables to evenly change the speed of the drivetrain
    static double FAST_SPEED_MULTIPLIER = 1;
    static double SLOW_SPEED_MULTIPLIER = 0.2;
    //resolution of camera view
    static int imgHeight = 896;
    static int imgWidth = 1600;
    public double turretBearing;
    //
    int desiredTagID;
    //String shootMode;
    PDController controller;
    boolean targetFound = false;
    boolean autoTarget = true;
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    //private IntakeInCommand intakeInCommand;
    public static boolean firing;
    AprilTagDetection detectedTag;
    double mFLPower;
    double mFRPower;
    double mBLPower;
    double mBRPower;
    //Forward and back power, Left and right power, rotation power.
    //All are then added and subtracted in different ways for each drive motor
    double FB, LR, Rotation;
    //CURRENT_SPEED_MULTIPLIER is the actual multiplier applied to the drive train power. It is set to either the fast or slow multipliers
    double CURRENT_SPEED_MULTIPLIER;
    double hoodPos;
    double LeftTrigger;
    double RightTrigger;
    public static IMU imu;
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    //below we create a new object instance of all the subsystem classes
    int visionOutputPosition = 1;
    ElapsedTime intakeTimer = new ElapsedTime();
    LaunchSequenceCommand fly, storeMiddle, storeOneForLast, storeOneForSecond, pullIn, pullInAgain, launch, store, unStore, scram;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    //gamepads
    //GamepadEx is an extended object version of gamepads that has more organized input checks that we use in triggers.
    private GamepadEx driver1, driver2;
    //drivetrain motors and variables
    //DcMotorEx is an expanded version of the DcMotor variable that gives us more methods.
    //For example, stop and reset encoder.
    private DcMotorEx mFL, mFR, mBL, mBR;
    //intake
    private Intake intake;
    //lift
    private BellyPan bellyPan;
    //turret
    private Turret turret;
    //storage
    private Storage storage;
    //shooter
    private Shooter shooter;
    //breakpad
    private BrakePad brakePad;
    //Cameras
     private Camera camera;
    //Roadrunner

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
            imu = hardwareMap.get(IMU.class, "imu");

            imu.resetYaw();


            //Removes previous Commands from scheduler
            //We call it at the start of TeleOp as it clears lingering autocommands that make the intake freak out
            //Make sure it is not called in a loop since it will clear all the triggers every frame. Be very careful. It is a kill switch.
            CommandScheduler.getInstance().reset();

            //sets the digital position of the robot to intake for the deposit to state command


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

            intake = new Intake(hardwareMap);

            bellyPan = new BellyPan(hardwareMap);

            shooter = new Shooter(hardwareMap);
            shooter.spinning = true;

            brakePad = new BrakePad(hardwareMap);

             camera = new Camera(hardwareMap);



            drive = new MecanumDrive(hardwareMap, savedPos);

            //map motors
            mFL = hardwareMap.get(DcMotorEx.class, "mFL");
            mFR = hardwareMap.get(DcMotorEx.class, "mFR");
            mBL = hardwareMap.get(DcMotorEx.class, "mBL");
            mBR = hardwareMap.get(DcMotorEx.class, "mBR");

            driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            shooter.hoodOffset = 0;
            shooter.speedOffset = 0;


            //makes the motors brake when power = zero. Is better for driver precision
            mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER;


            telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.


        }

        //Granny Mode
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .toggleWhenActive(() -> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER, () -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER);

        new Trigger(() -> driver2.getRightY() > .1)
                .whileActiveOnce(new InstantCommand(() -> shooter.speedOffset -= 25));

        new Trigger(() -> driver2.getRightY() < -.1)
                .whileActiveOnce(new InstantCommand(() -> shooter.speedOffset += 25));




        //Intake

        new Trigger(()->driver1.getButton(GamepadKeys.Button.X))
                .whenActive(new IntakeToggleCommand(intake, Intake.Direction.IN));

        new Trigger(()->driver1.getButton(GamepadKeys.Button.Y))
                .whenActive(new IntakeToggleCommand(intake, Intake.Direction.OUT));




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

            {


                new Trigger(() -> driver2.getButton(GamepadKeys.Button.B) || driver1.getButton(GamepadKeys.Button.B))
                        .whenActive(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> firing = true),
                                        new LaunchSequenceCommand(intake, storage, "Fly"),
                                        new InstantCommand(() -> driver2.gamepad.rumble(100)),
                                        new InstantCommand(() -> driver1.gamepad.rumble(100)),
                                        new InstantCommand(() -> firing = false)


                                )
                        );


            }




                new Trigger(() -> driver2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                        .toggleWhenActive(new InstantCommand(() -> turret.disablePID()), new InstantCommand(() -> turret.enablePID()));




            new Trigger(() -> driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.A))
                    .whenActive(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> firing = true),
                                    new LaunchSequenceCommand(intake, storage, "Launch"),
                                    new InstantCommand(() -> firing = false)
                            )
                    );



            new Trigger(() -> gamepad2.touchpad)
                    .whenActive(new InstantCommand(()->drive.localizer.setPose(camera.getRelocalizedPose(drive, telemetry))));


            new Trigger(() -> gamepad2.ps)
                    .toggleWhenActive(new InstantCommand(() -> aColor = "red"), new InstantCommand(() -> aColor = "blue"));

//            new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
//                    .whenActive(() -> drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, 0)));


            //Relocalize in Alliance Corner
            //In red LZ Corner
            new Trigger(() -> gamepad1.touchpad && aColor == "red")
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(() -> drive.localizer.setPose(new Pose2d(62, -62, Math.toRadians(0)))),
                            new InstantCommand(()->turret.cwORccw = 1),
                            new InstantCommand(() -> turret.turretLocalized = false)
                    ));
            //In blue LZ Corner
            new Trigger(() -> gamepad1.touchpad && aColor == "blue")
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(() -> drive.localizer.setPose(new Pose2d(62, 62, Math.toRadians(0)))),
                            new InstantCommand(()->turret.cwORccw = 1),
                            new InstantCommand(() -> turret.turretLocalized = false)
                    ));

            new Trigger(() -> gamepad1.ps)
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(() -> drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, Math.toRadians(0))))
                    ));


            new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP))
                    .whenInactive(() ->
                            shooter.hoodOffset += .025);

            new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN))
                    .whenInactive(() ->
                            shooter.hoodOffset -= .025);


        }
    }

    //this is the main run loop
    public void run() {
        super.run();

        shooter.setTargetDistance(GlobalVariables.distanceFromTarget);


        drive.localizer.update();

        intake.setCurrentArtifacts();




        if (gamepad1.dpad_left) {
            shooter.spinning = true;
            shooter.targeting = true;
        } else if (gamepad1.dpad_right) {
            shooter.spinning = false;
        }


       turret.updateTurretTracking(drive, telemetry);



        if (!turret.turretLocalized) {
            if (turret.lT.isPressed()) {
                turret.mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                turret.mT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                turret.turretLocalized = true;
                //turret.cwORccw = 1;
            }
        }





        telemetry.addData("preview on/off", "... Camera Stream\n");


        Rotation = -cubicScaling(gamepad1.left_trigger - gamepad1.right_trigger) * 0.75;
        FB = -cubicScaling(gamepad1.left_stick_y);
        LR = -cubicScaling(-gamepad1.left_stick_x) * 1.2;

        //defines the powers for the motors based on the stick inputs (trust i've written this so many times)


        if (!bellyPan.PTO_Engaged) {
            mFLPower = FB + LR + Rotation;
            mFRPower = FB - LR - Rotation;
            mBLPower = FB - LR + Rotation;
            mBRPower = FB + LR - Rotation;
        } else {
            CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER;
            brakePad.deploy();
            mFLPower = -Math.abs(FB);
            mFRPower = -Math.abs(FB);
            mBLPower = -Math.abs(FB);
            mBRPower = -Math.abs(FB);
            shooter.spinning = false;
            shooter.targeting = false;
            shooter.hoodOffset = .6;
            autoTarget = false;
        }


        //actually sets the motor powers


        mFL.setPower(mFLPower * CURRENT_SPEED_MULTIPLIER);
        mFR.setPower(mFRPower * CURRENT_SPEED_MULTIPLIER);
        mBL.setPower(mBLPower * CURRENT_SPEED_MULTIPLIER);
        mBR.setPower(mBRPower * CURRENT_SPEED_MULTIPLIER);

        Pose2d pose = drive.localizer.getPose();



        telemetry.addData("Pinpoint Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("Control Hub Heading (deg)", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw()));
        telemetry.addData("X", pose.position.x);
        telemetry.addData("Y", pose.position.y);

        telemetry.addLine();
        telemetry.addData("hoodPos", shooter.sH.getPosition());
        telemetry.addData("flyWheelSpeed", shooter.getFlyWheelSpeed());
        telemetry.addData("targetSpeed", shooter.WheelRegression.get(GlobalVariables.distanceFromTarget) + shooter.speedOffset);
        telemetry.addData("Distance",GlobalVariables.distanceFromTarget);
//        telemetry.addData("cwORccw relocRotation", turret.cwORccw);
        telemetry.addData("ApriltagsSeen", camera.getCurrentAprilTagDetections().size());
//        telemetry.addData("motorPower", shooter.mST.getPower());
//        telemetry.addLine();
//        telemetry.addData("turretOffset",turret.manualOffset);
//        telemetry.addData("WheelOffset", shooter.speedOffset);
//        telemetry.addLine();
//        telemetry.addData("aColor", aColor);


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




}

