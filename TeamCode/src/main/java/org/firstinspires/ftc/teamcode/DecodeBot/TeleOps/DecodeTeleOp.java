package org.firstinspires.ftc.teamcode.DecodeBot.TeleOps;


//import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret.tracking;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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

import org.firstinspires.ftc.teamcode.DecodeBot.Commands.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BreakPad;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BellyPan;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.IntakeCamera;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name = "DecodeTeleop", group = "AGen1")

public class DecodeTeleOp extends CommandOpMode {
    //gamepads
    //GamepadEx is an extended object version of gamepads that has more organized input checks that we use in triggers.
    private GamepadEx driver1, driver2;
    //private DcMotorEx liftEncoder;

    //This is just a boolean used for telemetry to see if we took in the incorrect sample color


    //
    int desiredTagID;



    public double turretBearing;


    PDController controller;


    boolean targetFound = false;
    boolean flyMode = true;

    enum shootModes {
        FLY,
        STORE_MIDDLE,
        STORE_ONE_FOR_LAST,
        STORE_ONE_FOR_SECOND
    }

    shootModes currentShootMode;
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
    private IntakeCamera camera;

    //Roadrunner
    private RRSubsystem rrSubsystem;

    double LeftTrigger;
    double RightTrigger;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    int visionOutputPosition = 1;


    LaunchSequenceCommand fly, storeMiddle, storeOneForLast, storeOneForSecond, pullIn, launch, store, unStore, scram;


    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    //stuff that is ran when you click init at the start of teleop.
    public void initialize() {

        {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            //Removes previous Commands from scheduler
            //We call it at the start of TeleOp as it clears lingering autocommands that make the intake freak out
            //Make sure it is not called in a loop since it will clear all the triggers every frame. Be very careful. It is a kill switch.
            CommandScheduler.getInstance().reset();

            //sets the digital position of the robot to intake for the deposit to state command


            //init controllers
            driver1 = new GamepadEx(gamepad1);
            driver2 = new GamepadEx(gamepad2);

            turret = new Turret(hardwareMap);

            storage = new Storage(hardwareMap);

            intake = new Intake(hardwareMap);

            bellyPan = new BellyPan(hardwareMap);

            turret = new Turret(hardwareMap);

            shooter = new Shooter(hardwareMap);

            breakPad = new BreakPad(hardwareMap);

            camera = new IntakeCamera(hardwareMap);

            rrSubsystem = new RRSubsystem(hardwareMap);

            //map motors
            mFL = hardwareMap.get(DcMotorEx.class, "mFL");
            mFR = hardwareMap.get(DcMotorEx.class, "mFR");
            mBL = hardwareMap.get(DcMotorEx.class, "mBL");
            mBR = hardwareMap.get(DcMotorEx.class, "mBR");


            //this motor physically runs opposite. For convenience, reverse direction.
            mBR.setDirection(DcMotorSimple.Direction.REVERSE);
            mFR.setDirection(DcMotorSimple.Direction.REVERSE);

            //makes the motors brake when power = zero. Is better for driver precision
            mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER;


            telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.


//            AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();
//
//
//            VisionPortal portal = new VisionPortal.Builder()
//                    .addProcessors(aTagP)
//                    .setCameraResolution(new Size(imgWidth, imgHeight))
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .build();


            //LaunchSequences

            fly = new LaunchSequenceCommand(intake, storage, "Fly");
            storeMiddle = new LaunchSequenceCommand(intake, storage, "StoreMiddle");
            storeOneForLast = new LaunchSequenceCommand(intake, storage, "StoreOneForLast");
            storeOneForSecond = new LaunchSequenceCommand(intake, storage, "StoreOneForSecond");
            pullIn = new LaunchSequenceCommand(intake,storage,"PullIn");
            launch = new LaunchSequenceCommand(intake,storage, "Launch");
            store = new LaunchSequenceCommand(intake,storage,"Store");
            unStore = new LaunchSequenceCommand(intake,storage,"UnStore");
            scram = new LaunchSequenceCommand(intake,storage,"Scram");
        }

        //Granny Mode
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .toggleWhenActive(() -> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER, () -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER);



        //Intake
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(new InstantCommand(intake::in));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(new InstantCommand(intake::allOut));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.Y))
                .whenActive(new InstantCommand(intake::stop));

        //move all in by 1
        new Trigger(()-> driver2.getButton(GamepadKeys.Button.A))
                .whenActive(pullIn);

        //only spit out the one in the intake
        new Trigger(()-> driver2.getButton(GamepadKeys.Button.X))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(intake::oneOut),
                        new WaitCommand(2000),
                        new InstantCommand(intake::stop)
                ));


        //Engage PTO
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.BACK))
                .whenActive(new ParallelCommandGroup(
                        new InstantCommand(bellyPan::engagePTO)
                ));

        //BreakPad
        new Trigger(()->driver1.getButton(GamepadKeys.Button.A))
                .toggleWhenActive(new InstantCommand(breakPad::deployBreakPad), new InstantCommand(breakPad::retractBreakPad));


        //Shooter mode
        new Trigger(()->driver2.getButton(GamepadKeys.Button.START))
                .toggleWhenActive(new InstantCommand(()->flyMode = true), new InstantCommand(()->flyMode = false));


        //automated targetting on/off
        new Trigger(()->driver2.getButton(GamepadKeys.Button.B))
                .whenActive(()-> autoTarget = true);

            //if driver 2 stickY's or triggers are used the autotarget is turned off
        new Trigger(()-> driftLock((float) driver2.getLeftY()) != 0 || driftLock((float) driver2.getRightY()) != 0 || driftLock((float) driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) != 0 || driftLock((float) driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) != 0)
                .whenActive(()-> autoTarget = false);


        //shoot

        {
            // Fly mode manual shots
            {
                new Trigger(() -> (currentShootMode == shootModes.FLY) && GlobalVariables.ballsShot == 0 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.X)))
                        //.whileActiveOnce(fly)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()->firing = true),
                                launch,
                                pullIn,
                                new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(()->firing = false)
                        ))
                ;

                new Trigger(() -> (currentShootMode == shootModes.FLY) && GlobalVariables.ballsShot == 1 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.X)))
                        //.whileActiveOnce(fly)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()->firing = true),
                                launch,
                                pullIn,
                                new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(()->firing = false)
                        ))
                ;

            }

            // Store Middle manual shots
            {
                new Trigger(() -> currentShootMode == shootModes.STORE_MIDDLE && GlobalVariables.ballsShot == 0 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.X)))
                        //.whileActiveOnce(storeMiddle)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()->firing = true),
                                launch,
                                pullIn,
                                store,
                                pullIn,
                                new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(()->firing = false)
                        ))
                ;

                new Trigger(() -> currentShootMode == shootModes.STORE_MIDDLE && GlobalVariables.ballsShot == 1 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.X)))
                        //.whileActiveOnce(storeMiddle)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()->firing = true),
                                launch,
                                unStore,
                                new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(()->firing = false)
                        ))
                ;
            }

            // Store One For Last manual shots
            {
                new Trigger(() -> currentShootMode == shootModes.STORE_ONE_FOR_LAST && GlobalVariables.ballsShot == 0 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.X)))
                        //.whileActiveOnce(storeOneForLast)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()->firing = true),
                                store,
                                pullIn,
                                launch,
                                pullIn,
                                new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(()->firing = false)
                        ))
                ;

                new Trigger(() -> currentShootMode == shootModes.STORE_ONE_FOR_LAST && GlobalVariables.ballsShot == 1 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.X)))
                        //.whileActiveOnce(storeOneForLast)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()->firing = true),
                                launch,
                                unStore,
                                new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(()->firing = false)
                        ))
                ;
            }

            // Store One For Second manual shots
            {
                new Trigger(() -> currentShootMode == shootModes.STORE_ONE_FOR_SECOND && GlobalVariables.ballsShot == 0 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.X)))
                        //.whileActiveOnce(storeOneForSecond)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()->firing = true),
                                store,
                                pullIn,
                                launch,
                                unStore,
                                new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(()->firing = false)
                        ))
                ;

                new Trigger(() -> currentShootMode == shootModes.STORE_ONE_FOR_SECOND && GlobalVariables.ballsShot == 1 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.X)))
                        //.whileActiveOnce(storeOneForSecond)
                        .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()->firing = true),
                                launch,
                                pullIn,
                                new InstantCommand(() -> GlobalVariables.ballsShot += 1),
                                new InstantCommand(()->firing = false)
                        ))
                ;
            }

            // All shoot modes end with just launching
            new Trigger(() -> GlobalVariables.ballsShot == 2 && (driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.X)))
                    //.whileActiveOnce(storeOneForSecond)
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(()->firing = true),
                            launch,
                            new InstantCommand(() -> GlobalVariables.ballsShot = 0),
                            new InstantCommand(()->firing = false)
                    ))
            ;

            // Triple shot modes
            // TODO: Check that these actually launch all 3 or if it just stops at one
            {
                new Trigger(() -> currentShootMode == shootModes.FLY && GlobalVariables.ballsShot == 0 && driver2.getButton(GamepadKeys.Button.B))
                        .whileActiveOnce(
                                new SequentialCommandGroup(
                                        new InstantCommand(()->firing = true),
                                        fly,
                                        new InstantCommand(()->firing = false),
                                        new InstantCommand(()->driver2.gamepad.rumble(500))
                                )
                        );

                new Trigger(() -> currentShootMode == shootModes.STORE_MIDDLE && GlobalVariables.ballsShot == 0 && driver2.getButton(GamepadKeys.Button.B))
                        .whileActiveOnce(
                                new SequentialCommandGroup(
                                        new InstantCommand(()->firing = true),
                                        storeMiddle,
                                        new InstantCommand(()->firing = false),
                                        new InstantCommand(()->driver2.gamepad.rumble(500))
                                )
                        );

                new Trigger(() -> currentShootMode == shootModes.STORE_ONE_FOR_LAST && GlobalVariables.ballsShot == 0 && driver2.getButton(GamepadKeys.Button.B))
                        .whileActiveOnce(
                                new SequentialCommandGroup(
                                        new InstantCommand(()->firing = true),
                                        storeOneForLast,
                                        new InstantCommand(()->firing = false),
                                        new InstantCommand(()->driver2.gamepad.rumble(500))
                                )
                        );

                new Trigger(() -> currentShootMode == shootModes.STORE_ONE_FOR_SECOND && GlobalVariables.ballsShot == 0 && driver2.getButton(GamepadKeys.Button.B))
                        .whileActiveOnce(
                                new SequentialCommandGroup(
                                        new InstantCommand(()->firing = true),
                                        storeOneForSecond,
                                        new InstantCommand(()->firing = false),
                                        new InstantCommand(()->driver2.gamepad.rumble(500))
                                )
                        );

            }
            
            new Trigger(() -> firing && driver2.wasJustReleased(GamepadKeys.Button.B))
                    .whenActive(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> fly.cancel()),
                                    new InstantCommand(()-> storeMiddle.cancel()),
                                    new InstantCommand(()-> storeOneForSecond.cancel()),
                                    new InstantCommand(()-> storeOneForLast.cancel()),
                                    new InstantCommand((Runnable) scram),
                                    new InstantCommand(()->driver2.gamepad.rumble(1500))
                            )
                            
                    );
        }
    }

    //this is the main run loop
    public void run() {
        super.run();

        if (GlobalVariables.aColor == "red") {
            desiredTagID = 24;
        }
        if (GlobalVariables.aColor == "blue") {
            desiredTagID = 20;
        }


        
        if(flyMode  || GlobalVariables.currentArtifacts == GlobalVariables.motif){
            currentShootMode = shootModes.FLY;
        }
        else if( (GlobalVariables.currentArtifacts == "PPG" && GlobalVariables.motif == "PGP") || (GlobalVariables.currentArtifacts == "PGP" && GlobalVariables.motif == "PPG") ){
            currentShootMode = shootModes.STORE_MIDDLE;
        }
        else if( (GlobalVariables.currentArtifacts == "PGP" && GlobalVariables.motif == "GPP") || (GlobalVariables.currentArtifacts == "GPP" && GlobalVariables.motif == "PPG") ){
            currentShootMode = shootModes.STORE_ONE_FOR_LAST;
        }
        else if( (GlobalVariables.currentArtifacts == "GPP" && GlobalVariables.motif == "PGP") ){
            currentShootMode = shootModes.STORE_ONE_FOR_SECOND;
        }


        telemetry.addData("preview on/off", "... Camera Stream\n");

        AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();

        List<AprilTagDetection> currentDetections = aTagP.getDetections();

        for (AprilTagDetection detection : currentDetections) {

            //  Check to see if we want to track towards this tag.
            if ((detection.id == desiredTagID)) {
                // Yes, we want to use this tag.
                targetFound = true;

                detectedTag = detection;
                break;  // don't look any further.
            } else {
                targetFound = false;
            }
        }

        if (detectedTag.id == desiredTagID) {

            turretBearing = detectedTag.ftcPose.bearing;


        }



        //AprilTag converter equation from bearing to encoder ticks
        //

//        if (tracking == true) {
//            Turret.targetPosition = Turret.getCurrentPosition() - turretBearing * TURRET_DEGREE_TO_TICK_MULTIPLIER;
//        }


        Rotation = cubicScaling(-gamepad1.right_stick_x) * 0.5;
        FB = cubicScaling(gamepad1.left_stick_y);
        LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;

        //defines the powers for the motors based on the stick inputs (trust i've written this so many times)


        if(!BellyPan.PTO_Engaged){
            mFLPower = FB + LR + Rotation;
            mFRPower = FB - LR - Rotation;
            mBLPower = FB - LR + Rotation;
            mBRPower = FB + LR - Rotation;
        }
        else{
            mFLPower = FB;
            mFRPower = FB;
            mBLPower = FB;
            mBRPower = FB;
        }
        //actually sets the motor powers


            mFL.setPower(mFLPower * CURRENT_SPEED_MULTIPLIER);
            mFR.setPower(mFRPower * CURRENT_SPEED_MULTIPLIER);
            mBL.setPower(mBLPower * CURRENT_SPEED_MULTIPLIER);
            mBR.setPower(mBRPower * CURRENT_SPEED_MULTIPLIER);



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

