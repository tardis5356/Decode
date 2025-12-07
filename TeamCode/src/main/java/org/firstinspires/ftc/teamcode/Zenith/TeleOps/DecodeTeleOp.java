package org.firstinspires.ftc.teamcode.Zenith.TeleOps;


//import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.tracking;

import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.DecodeAuto.savedPos;


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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Zenith.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Zenith.Commands.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BreakPad;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BellyPan;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.RecoveryBuilder;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "DecodeTeleop", group = "AGen1")

public class DecodeTeleOp extends CommandOpMode {
    //gamepads
    //GamepadEx is an extended object version of gamepads that has more organized input checks that we use in triggers.
    private GamepadEx driver1, driver2;
    //private DcMotorEx liftEncoder;

    //This is just a boolean used for telemetry to see if we took in the incorrect sample color

    //TODO:

    //
    int desiredTagID;


    public double turretBearing;


    PDController controller;


    boolean targetFound = false;
    public static boolean flyMode = true;

    public static enum shootModes {
        FLY,
        STORE_MIDDLE,
        STORE_ONE_FOR_LAST,
        STORE_ONE_FOR_SECOND,
        MANUAL
    }

    public static shootModes currentShootMode;
    //String shootMode;

    boolean autoTarget = true;

    boolean firing;

    AprilTagDetection detectedTag;

    //private IntakeInCommand intakeInCommand;

    //drivetrain motors and variables
    //DcMotorEx is an expanded version of the DcMotor variable that gives us more methods.
    //For example, stop and reset encoder.
    private DcMotorEx mFL, mFR, mBL, mBR;

    double mFLPower;
    double mFRPower;
    double mBLPower;
    double mBRPower;

    //Forward and back power, Left and right power, rotation power.
    //All are then added and subtracted in different ways for each drive motor
    double FB, LR, Rotation;

    //multipliers applied to the sum of the above variables to evenly change the speed of the drivetrain
    static double FAST_SPEED_MULTIPLIER = 1;
    static double SLOW_SPEED_MULTIPLIER = 0.4;

    //CURRENT_SPEED_MULTIPLIER is the actual multiplier applied to the drive train power. It is set to either the fast or slow multipliers
    double CURRENT_SPEED_MULTIPLIER;

    //resolution of camera view
    static int imgHeight = 896;
    static int imgWidth = 1600;

    double hoodPos;

    //below we create a new object instance of all the subsystem classes

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
    private BreakPad breakPad;

    //Cameras
    private Camera camera;

    //Roadrunner
    private RRSubsystem rrSubsystem;

    private RecoveryBuilder recoveryBuilder;

    private MecanumDrive drive;

    double LeftTrigger;
    double RightTrigger;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    int visionOutputPosition = 1;


    LaunchSequenceCommand fly, storeMiddle, storeOneForLast, storeOneForSecond, pullIn, pullInAgain, launch, store, unStore, scram;


    FtcDashboard dashboard = FtcDashboard.getInstance();


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

//            if (savedPos == null) {
            savedPos = new Pose2d(0, 0, Math.toRadians(0));
//            }

            //init controllers
            driver1 = new GamepadEx(gamepad1);
            driver2 = new GamepadEx(gamepad2);

            turret = new Turret(hardwareMap);
            turret.mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.mT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            storage = new Storage(hardwareMap);

            intake = new Intake(hardwareMap);

            bellyPan = new BellyPan(hardwareMap);

            shooter = new Shooter(hardwareMap);
            shooter.spinning = false;

            breakPad = new BreakPad(hardwareMap);

            camera = new Camera(hardwareMap);

            rrSubsystem = new RRSubsystem(hardwareMap);

            drive = new MecanumDrive(hardwareMap, savedPos);

            //map motors
            mFL = hardwareMap.get(DcMotorEx.class, "mFL");
            mFR = hardwareMap.get(DcMotorEx.class, "mFR");
            mBL = hardwareMap.get(DcMotorEx.class, "mBL");
            mBR = hardwareMap.get(DcMotorEx.class, "mBR");


            //this motor physically runs opposite. For convenience, reverse direction.
            //mBL.setDirection(DcMotorSimple.Direction.REVERSE);
            //mFL.setDirection(DcMotorSimple.Direction.REVERSE);
            //mBR.setDirection(DcMotorSimple.Direction.REVERSE);
            //mFR.setDirection(DcMotorSimple.Direction.REVERSE);

            //makes the motors brake when power = zero. Is better for driver precision
            mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER;


            telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.

            storage.returnSlot();
            //shooter.sH.setPosition(.05);


//            AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();
//
//
//            VisionPortal portal = new VisionPortal.Builder()
//                    .addProcessors(aTagP)
//                    .setCameraResolution(new Size(imgWidth, imgHeight))
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .build();


            //LaunchSequences

            //fly = new LaunchSequenceCommand(intake, storage, "Fly");
            //storeMiddle = new LaunchSequenceCommand(intake, storage, "StoreMiddle");
            //storeOneForLast = new LaunchSequenceCommand(intake, storage, "StoreOneForLast");
            //storeOneForSecond = new LaunchSequenceCommand(intake, storage, "StoreOneForSecond");

            pullIn = new LaunchSequenceCommand(intake, storage, "PullIn");

            //pullInAgain = new LaunchSequenceCommand(intake, storage, "PullIn");
            //launch = new LaunchSequenceCommand(intake, storage, "Launch");
            //store = new LaunchSequenceCommand(intake, storage, "Store");
            //unStore = new LaunchSequenceCommand(intake, storage, "UnStore");
            //scram = new LaunchSequenceCommand(intake, storage, "Scram");
        }

        //Granny Mode
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .toggleWhenActive(() -> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER, () -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER);

        //red vs blue alliance
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(new InstantCommand(() -> GlobalVariables.aColor = "red"));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenActive(new InstantCommand(() -> GlobalVariables.aColor = "blue"));

        //Intake
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.X))
                .toggleWhenActive(new InstantCommand(intake::in), new InstantCommand(intake::stop));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.Y))
                .toggleWhenActive(new InstantCommand(intake::out), new InstantCommand(intake::stop));

        //new Trigger(() -> (driver1.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.X)) && intake.intakePower != 0)
        //        .whenActive(new InstantCommand(intake::stop));

        //move all in by 1
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A))
                .whenActive(pullIn);

        //only spit out the one in the intake
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.X))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(intake::out),
                        new WaitCommand(2000),
                        new InstantCommand(intake::stop)
                ));

        //Swapper
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .toggleWhenActive(storage::storeSlot, storage::returnSlot);

        //Back Gate
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .toggleWhenActive(storage::closeBack, storage::openBack);


        //Engage/Disengage PTO
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.BACK))
                .toggleWhenActive(bellyPan::engagePTO, bellyPan::disEngagePTO);

        //BreakPad
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.A))
                .toggleWhenActive(new InstantCommand(breakPad::deployBreakPad), new InstantCommand(breakPad::retractBreakPad));


        //Shooter mode
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.START))
                .toggleWhenActive(new InstantCommand(() -> flyMode = true), new InstantCommand(() -> flyMode = false));


        //automated targetting on/off
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.BACK))
                .whenActive(() -> autoTarget = true);

        //if driver 2 stickY's or triggers are used the autotarget is turned off
        new Trigger(() -> driftLock((float) driver2.getLeftY()) != 0 ||
                driftLock((float) driver2.getRightY()) != 0 ||
                driftLock((float) driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) != 0 ||
                driftLock((float) driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) != 0 ||
                driver2.getButton(GamepadKeys.Button.LEFT_BUMPER) ||
                driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(() -> autoTarget = false);


        //shoot

        {

            {
                //Fly first and second shot, store one for second second shot
                new Trigger(() -> ((currentShootMode == shootModes.FLY) && (GlobalVariables.ballsShot == 0 || GlobalVariables.ballsShot == 1) && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.B))) ||
                        (currentShootMode == shootModes.STORE_ONE_FOR_SECOND && GlobalVariables.ballsShot == 1 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.B)))
                )
                        //.whileActiveOnce(fly)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(() -> firing = true),
                                new LaunchSequenceCommand(intake, storage, "Launch"),
                                new LaunchSequenceCommand(intake, storage, "PullIn"),
                                //  new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(() -> driver2.gamepad.rumble(300)),
                                new InstantCommand(() -> driver1.gamepad.rumble(300)),
                                new InstantCommand(() -> firing = false)
                        ));


            }


            {
                // Store Middle first shot
                new Trigger(() -> currentShootMode == shootModes.STORE_MIDDLE && GlobalVariables.ballsShot == 0 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.B)))
                        //.whileActiveOnce(storeMiddle)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(() -> firing = true),
                                new LaunchSequenceCommand(intake, storage, "Launch"),
                                new LaunchSequenceCommand(intake, storage, "PullIn"),
                                new LaunchSequenceCommand(intake, storage, "Store"),
                                new LaunchSequenceCommand(intake, storage, "PullIn"),
                                //   new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(() -> driver2.gamepad.rumble(300)),
                                new InstantCommand(() -> driver1.gamepad.rumble(300)),
                                new InstantCommand(() -> firing = false)
                        ))
                ;

                //Store Middle and Store One for Last second shots
                new Trigger(() -> (currentShootMode == shootModes.STORE_MIDDLE && GlobalVariables.ballsShot == 1 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.B))) ||
                        (currentShootMode == shootModes.STORE_ONE_FOR_LAST && GlobalVariables.ballsShot == 1 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.B))))
                        //.whileActiveOnce(storeMiddle)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(() -> firing = true),
                                new LaunchSequenceCommand(intake, storage, "Launch"),
                                new LaunchSequenceCommand(intake, storage, "UnStore"),
                                //  new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(() -> driver2.gamepad.rumble(300)),
                                new InstantCommand(() -> driver1.gamepad.rumble(300)),
                                new InstantCommand(() -> firing = false)
                        ))
                ;
            }

            // Store One For Last first shot
            {
                new Trigger(() -> currentShootMode == shootModes.STORE_ONE_FOR_LAST && GlobalVariables.ballsShot == 0 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.B)))
                        //.whileActiveOnce(storeOneForLast)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(() -> firing = true),
                                new LaunchSequenceCommand(intake, storage, "Store"),
                                new LaunchSequenceCommand(intake, storage, "PullIn"),
                                new LaunchSequenceCommand(intake, storage, "Launch"),
                                new LaunchSequenceCommand(intake, storage, "PullIn"),
                                //     new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(() -> driver2.gamepad.rumble(300)),
                                new InstantCommand(() -> driver1.gamepad.rumble(300)),
                                new InstantCommand(() -> firing = false)
                        ))
                ;


            }

            // Store One For Second first shot
            {
                new Trigger(() -> currentShootMode == shootModes.STORE_ONE_FOR_SECOND && GlobalVariables.ballsShot == 0 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.B)))
                        //.whileActiveOnce(storeOneForSecond)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(() -> firing = true),
                                new LaunchSequenceCommand(intake, storage, "Store"),
                                new LaunchSequenceCommand(intake, storage, "PullIn"),
                                new LaunchSequenceCommand(intake, storage, "Launch"),
                                new LaunchSequenceCommand(intake, storage, "UnStore"),
                                //    new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(() -> driver2.gamepad.rumble(300)),
                                new InstantCommand(() -> driver1.gamepad.rumble(300)),
                                new InstantCommand(() -> firing = false)
                        ))
                ;

            }

            // All shoot modes end with just launching
            new Trigger(() -> GlobalVariables.ballsShot == 2 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.B)))
                    //.whileActiveOnce(storeOneForSecond)
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(() -> firing = true),
                            new LaunchSequenceCommand(intake, storage, "Launch"),
                            new InstantCommand(() -> GlobalVariables.ballsShot = 0),
                            new InstantCommand(() -> driver2.gamepad.rumble(300)),
                            new InstantCommand(() -> driver1.gamepad.rumble(300)),
                            new InstantCommand(() -> firing = false),
                            new InstantCommand(() -> intake.currentArtifactsEstablished = false)
                    ))
            ;

            new Trigger(() -> driver2.getButton(GamepadKeys.Button.B))
                    .whenActive(
                            new SequentialCommandGroup(
                                    new LaunchSequenceCommand(intake, storage, "Fly"),
                                    new InstantCommand(() -> driver2.gamepad.rumble(300)),
                                    new InstantCommand(() -> driver1.gamepad.rumble(300))
                            )
                    );

            new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                    .whenActive(() -> drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, 0)));


            new Trigger(()-> driver2.getButton(GamepadKeys.Button.DPAD_UP))
                    .whenInactive(new InstantCommand(()->
                            hoodPos += .05)
                    );

            new Trigger(()-> driver2.getButton(GamepadKeys.Button.DPAD_DOWN))
                    .whenInactive(new InstantCommand(()->
                            hoodPos -= .05)
                    );

            // Triple shot modes
            // TODO: Check that these actually launch all 3 or if it just stops at one
//            {
//                new Trigger(() -> currentShootMode == shootModes.FLY && GlobalVariables.ballsShot == 0 && driver2.getButton(GamepadKeys.Button.B))
//                        .whileActiveOnce(
//                                new SequentialCommandGroup(
//                                        new InstantCommand(()->firing = true),
//                                        fly,//new LaunchSequenceCommand(intake, storage, "Fly"),
//                                        new InstantCommand(()->firing = false),
//                                        new InstantCommand(()->driver2.gamepad.rumble(.5,.5, 500)),
//                                        new InstantCommand(()->intake.currentArtifactsEstablished = false)
//                                )
//                        );
//
//                new Trigger(() -> currentShootMode == shootModes.STORE_MIDDLE && GlobalVariables.ballsShot == 0 && driver2.getButton(GamepadKeys.Button.B))
//                        .whileActiveOnce(
//                                new SequentialCommandGroup(
//                                        new InstantCommand(()->firing = true),
//                                        storeMiddle,//new LaunchSequenceCommand(intake, storage, "StoreMiddle"),
//                                        new InstantCommand(()->firing = false),
//                                        new InstantCommand(()->driver2.gamepad.rumble(.5,.5,500)),
//                                        new InstantCommand(()->intake.currentArtifactsEstablished = false)
//                                )
//                        );
//
//                new Trigger(() -> currentShootMode == shootModes.STORE_ONE_FOR_LAST && GlobalVariables.ballsShot == 0 && driver2.getButton(GamepadKeys.Button.B))
//                        .whileActiveOnce(
//                                new SequentialCommandGroup(
//                                        new InstantCommand(()->firing = true),
//                                        storeOneForLast,//new LaunchSequenceCommand(intake, storage, "StoreOneForLast"),
//                                        new InstantCommand(()->firing = false),
//                                        new InstantCommand(()->driver2.gamepad.rumble(.5,.5,500)),
//                                        new InstantCommand(()->intake.currentArtifactsEstablished = false)
//                                )
//                        );
//
//                new Trigger(() -> currentShootMode == shootModes.STORE_ONE_FOR_SECOND && GlobalVariables.ballsShot == 0 && driver2.getButton(GamepadKeys.Button.B))
//                        .whileActiveOnce(
//                                new SequentialCommandGroup(
//                                        new InstantCommand(()->firing = true),
//                                        storeOneForSecond,//new LaunchSequenceCommand(intake, storage, "StoreOneForSecond"),
//                                        new InstantCommand(()->firing = false),
//                                        new InstantCommand(()->driver2.gamepad.rumble(.5,.5,500)),
//                                        new InstantCommand(()->intake.currentArtifactsEstablished = false)
//                                )
//                        );
//
//            }
//
//            new Trigger(() -> firing && driver2.wasJustReleased(GamepadKeys.Button.B))
//                    .whenActive(
//                            new SequentialCommandGroup(
//                                    new InstantCommand(()-> fly.cancel()),
//                                    new InstantCommand(()-> storeMiddle.cancel()),
//                                    new InstantCommand(()-> storeOneForSecond.cancel()),
//                                    new InstantCommand(()-> storeOneForLast.cancel()),
//                                    new InstantCommand(()-> intake.setCurrentArtifacts()),
//                                    new InstantCommand(()->driver2.gamepad.rumble(1,1,1000)),
//                                    RecoveryBuilder.BuildRecSequence(currentShootMode,storage, intake)
//                                    //new InstantCommand((Runnable) scram),
//
//                            )
//
//                    );
        }
    }

    //this is the main run loop
    public void run() {
        super.run();

        drive.localizer.update();

        intake.setCurrentArtifacts();

        shooter.sH.setPosition(hoodPos);

        if (gamepad1.dpad_left) {
            shooter.spinning = true;
        } else if (gamepad1.dpad_right) {
            shooter.spinning = false;
        }

        if (autoTarget) {
            turret.updateTurretTracking(drive, telemetry);
        }


        if (flyMode || GlobalVariables.currentArtifacts.substring(1) == GlobalVariables.motif) {
            currentShootMode = shootModes.FLY;
        } else if ((GlobalVariables.currentArtifacts.substring(1) == "PPG" && GlobalVariables.motif == "PGP") || (GlobalVariables.currentArtifacts.substring(1) == "PGP" && GlobalVariables.motif == "PPG")) {
            currentShootMode = shootModes.STORE_MIDDLE;
        } else if ((GlobalVariables.currentArtifacts.substring(1) == "PGP" && GlobalVariables.motif == "GPP") || (GlobalVariables.currentArtifacts.substring(1) == "GPP" && GlobalVariables.motif == "PPG")) {
            currentShootMode = shootModes.STORE_ONE_FOR_LAST;
        } else if ((GlobalVariables.currentArtifacts.substring(1) == "GPP" && GlobalVariables.motif == "PGP")) {
            currentShootMode = shootModes.STORE_ONE_FOR_SECOND;
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
            mFLPower = -Math.abs(FB);
            mFRPower = -Math.abs(FB);
            mBLPower = -Math.abs(FB);
            mBRPower = -Math.abs(FB);
            shooter.spinning = false;
        }


        //actually sets the motor powers


        mFL.setPower(mFLPower * CURRENT_SPEED_MULTIPLIER);
        mFR.setPower(mFRPower * CURRENT_SPEED_MULTIPLIER);
        mBL.setPower(mBLPower * CURRENT_SPEED_MULTIPLIER);
        mBR.setPower(mBRPower * CURRENT_SPEED_MULTIPLIER);

        Pose2d pose = drive.localizer.getPose();

        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("X", pose.position.x);
        telemetry.addData("Y", pose.position.y);

        telemetry.addData("PTO_Engaged", bellyPan.PTO_Engaged);

        telemetry.addData("currentArtifacts", GlobalVariables.currentArtifacts);
        telemetry.addData("currentShootmode", currentShootMode);
        telemetry.addData("hoodPos", shooter.sH.getPosition());
        telemetry.addData("flyWheelSpeed", shooter.getFlyWheelSpeed());
        telemetry.addData("targetSpeed",shooter.flyWheelSpeed);
        telemetry.addData("motorPower",shooter.mST.getPower());


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

