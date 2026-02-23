package org.firstinspires.ftc.teamcode.Zenith.TeleOps;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Zenith.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Zenith.Commands.IntakeToggleCommand;
import org.firstinspires.ftc.teamcode.Zenith.Commands.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BellyPan;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BrakePad;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "PinpointDebug", group = "AGen1")

public class PinpointDebug extends CommandOpMode {

    double mFLPower;
    double mFRPower;
    double mBLPower;
    double mBRPower;

    public static IMU imu;
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    int visionOutputPosition = 1;
    ElapsedTime intakeTimer = new ElapsedTime();
    LaunchSequenceCommand fly, storeMiddle, storeOneForLast, storeOneForSecond, pullIn, pullInAgain, launch, store, unStore, scram;
    FtcDashboard dashboard = FtcDashboard.getInstance();
      private GamepadEx driver1, driver2;
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
    public GoBildaPinpointDriver driver;
    double FB, LR, Rotation;
    public GoBildaPinpointDriver.EncoderDirection initialParDirection;
    public GoBildaPinpointDriver.EncoderDirection initialPerpDirection;


    @Override
    //stuff that is ran when you click init at the start of teleop.
    public void initialize() {


        {
            GlobalVariables.inAuto = false;
//            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


            //Removes previous Commands from scheduler
            //We call it at the start of TeleOp as it clears lingering autocommands that make the intake freak out
            //Make sure it is not called in a loop since it will clear all the triggers every frame. Be very careful. It is a kill switch.
            CommandScheduler.getInstance().reset();

            //sets the digital position of the robot to intake for the deposit to state command


            //init controllers
            driver1 = new GamepadEx(gamepad1);
            driver2 = new GamepadEx(gamepad2);

            turret = new Turret(hardwareMap);

//            if (savedPos == null) {
//                savedPos = new Pose2d(0, 0, Math.toRadians(0));
                turret.mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            }

            turret.mT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            storage = new Storage(hardwareMap);

            intake = new Intake(hardwareMap);

            bellyPan = new BellyPan(hardwareMap);

            shooter = new Shooter(hardwareMap);
            shooter.spinning = true;

            brakePad = new BrakePad(hardwareMap);

            camera = new Camera(hardwareMap);
            telemetry.addLine("NOT READY");
            telemetry.update();
            sleep(500);

//            drive = new MecanumDrive(hardwareMap, savedPos);
             double parYTicks = -2040.015;
             double perpXTicks = -3480.1;
            driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//driver.setOffsets(-3.34, -5.5, DistanceUnit.INCH);
            double mmPerTick = MecanumDrive.PARAMS.inPerTick * 25.4;
            driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
            driver.setOffsets(mmPerTick * parYTicks, mmPerTick * perpXTicks, DistanceUnit.MM);

            driver.setYawScalar(0.99933711*(1/0.99986111));

            // TODO: reverse encoder directions if needed
            initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
            initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

            driver.setEncoderDirections(initialParDirection, initialPerpDirection);

            driver.resetPosAndIMU();
            imu = hardwareMap.get(IMU.class, "imu");

            imu.initialize(
                    new IMU.Parameters(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                            )
                    )
            );

            imu.resetYaw();

            //map motors
            mFL = hardwareMap.get(DcMotorEx.class, "mFL");
            mFR = hardwareMap.get(DcMotorEx.class, "mFR");
            mBL = hardwareMap.get(DcMotorEx.class, "mBL");
            mBR = hardwareMap.get(DcMotorEx.class, "mBR");

            //driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            shooter.hoodOffset = 0;
            shooter.speedOffset = 0;


            //makes the motors brake when power = zero. Is better for driver precision
            mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




            telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.


        }





        //Intake

        new Trigger(()->driver1.getButton(GamepadKeys.Button.X))
                .whenActive(new IntakeToggleCommand(intake, Intake.Direction.IN));

        new Trigger(()->driver1.getButton(GamepadKeys.Button.Y))
                .whenActive(new IntakeToggleCommand(intake, Intake.Direction.OUT));

        new Trigger(()->driver1.getButton(GamepadKeys.Button.A))
                .whenActive( new InstantCommand(()->driver.resetPosAndIMU()));








        //automated targetting on/off
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.BACK))
                //TODO: swap this back to true after testing
                .toggleWhenActive(new InstantCommand(turret::disablePID), new InstantCommand(() -> turret.enablePID()));

        {

            {


                new Trigger(() -> driver2.getButton(GamepadKeys.Button.B) || driver1.getButton(GamepadKeys.Button.B))
                        .whenActive(
                                new SequentialCommandGroup(

                                        new LaunchSequenceCommand(intake, storage, "Fly"),
                                        new InstantCommand(() -> driver2.gamepad.rumble(100)),
                                        new InstantCommand(() -> driver1.gamepad.rumble(100))



                                )
                        );


            }




            new Trigger(() -> driver2.getButton(GamepadKeys.Button.Y) || driver1.getButton(GamepadKeys.Button.A))
                    .whenActive(
                            new SequentialCommandGroup(
                                    new InstantCommand(storage::openGate),
                                    new InstantCommand(intake::out),
                                    new WaitCommand(100),
                                    new InstantCommand(storage::closeGate),
                                    new InstantCommand(intake::stop)
                            )
                    );



            new Trigger(() -> gamepad2.touchpad)
                    .whenActive(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> shooter.targeting = false),
                                    new InstantCommand(()-> shooter.hoodOffset = .95),
                                    new WaitCommand(400),
                                    new InstantCommand(() -> drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, Math.toRadians(camera.getATagRobotHeading(turret, telemetry))))),
                                    new InstantCommand(()->drive.localizer.setPose(camera.getRelocalizedPose(drive, telemetry))),
                                    new InstantCommand(()-> shooter.hoodOffset = 0.0),
                                    new InstantCommand(()-> turret.manualOffset = 0),
                                    new InstantCommand(()-> shooter.targeting = true)

                            )
                    );


            new Trigger(() -> gamepad2.ps)
                    .toggleWhenActive(new InstantCommand(() -> aColor = "red"), new InstantCommand(() -> aColor = "blue"));




            new Trigger(()-> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT))
                    .toggleWhenActive(storage::closeGate, storage::openGate);




        }
        telemetry.clearAll();
        telemetry.addLine("Ready To Go");
        telemetry.update();
    }

    //this is the main run loop
    public void run() {
        driver.update();
        super.run();



        drive.localizer.setPose(new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(AngleUnit.RADIANS)));

        shooter.setTargetDistance(GlobalVariables.distanceFromTarget);

        intake.setCurrentArtifacts();

        if (gamepad1.dpad_left) {
            shooter.spinning = true;
            shooter.targeting = true;
        } else if (gamepad1.dpad_right) {
            shooter.spinning = false;
        }

        turret.updateTurretTracking(drive, telemetry);
        telemetry.addData("preview on/off", "... Camera Stream\n");
        Rotation = -cubicScaling(gamepad1.left_trigger - gamepad1.right_trigger) * 0.75;
        FB = -cubicScaling(gamepad1.left_stick_y);
        LR = -cubicScaling(-gamepad1.left_stick_x) * 1.2;
        //defines the powers for the motors based on the stick inputs (trust i've written this so many times)
            mFLPower = FB + LR + Rotation;
            mFRPower = FB - LR - Rotation;
            mBLPower = FB - LR + Rotation;
            mBRPower = FB + LR - Rotation;
        //actually sets the motor powers
        mFL.setPower(mFLPower );
        mFR.setPower(mFRPower );
        mBL.setPower(mBLPower);
        mBR.setPower(mBRPower);
        telemetry.addData("Pinpoint Heading (deg)", driver.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Control Hub Heading (deg)",imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("X", driver.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", driver.getPosY(DistanceUnit.INCH));





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