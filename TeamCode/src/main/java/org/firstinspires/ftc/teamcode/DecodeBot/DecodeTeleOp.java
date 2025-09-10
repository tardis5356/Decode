package org.firstinspires.ftc.teamcode.DecodeBot;


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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
@TeleOp(name = "MVCC_TeleOp", group = "AGen1")

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

    AprilTagDetection detectedTag;

    //private IntakeInCommand intakeInCommand;

    //drivetrain motors and variables
    //DcMotorEx is an expanded version of the DcMotor variable that gives us more methods.
    //For example, stop and reset encoder.
    private DcMotorEx mFL, mFR, mBL, mBR;
    private DcMotorEx mS;


    //Forward and back power, Left and right power, rotation power.
    //All are then added and subtracted in different ways for each drive motor
    double FB, LR, Rotation;

    //multipliers applied to the sum of the above variables to evenly change the speed of the drivetrain
    static double FAST_SPEED_MULTIPLIER = 1;
    static double SLOW_SPEED_MULTIPLIER = 0.4;

    //CURRENT_SPEED_MULTIPLIER is the actual multiplier applied to the drive train power. It is set to either the fast or slow multipliers
    double CURRENT_SPEED_MULTIPLIER;

    static int imgHeight = 896;
    static int imgWidth = 1600;
    //below we create a new object instance of all the subsystem classes
    //gripper


    //wrist
    private Intake intake;

    private Spindex spindex;
    private Lift lift;


    double LeftTrigger;
    double RightTrigger;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    int visionOutputPosition = 1;


    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    //stuff that is ran when you click init at the start of teleop.
    public void initialize() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //Removes previous Commands from scheduler
        //We call it at the start of TeleOp as it clears lingering autocommands that make the intake freak out
        //Make sure it is not called in a loop since it will clear all the triggers every frame. Be very careful. It is a kill switch.
        CommandScheduler.getInstance().reset();

        //sets the digital position of the robot to intake for the deposit to state command


        //init controllers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);


        spindex = new Spindex(hardwareMap);

        intake = new Intake(hardwareMap);

        lift = new Lift(hardwareMap);


        //map motors
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");


        mS = hardwareMap.get(DcMotorEx.class, "mS");


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


        AprilTagProcessor aTagP = new AprilTagProcessor.Builder().build();


        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(aTagP)
                .setCameraResolution(new Size(imgWidth, imgHeight))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();


        if (GlobalVariables.aColor == "red") {
            desiredTagID = 24;
        }
        if (GlobalVariables.aColor == "blue") {
            desiredTagID = 20;
        }

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .toggleWhenActive(() -> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER, () -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER);

        new Trigger (() -> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER) && Intake.intakeState == "stop" ||  Intake.intakeState == "out")
                .whenActive(new InstantCommand(intake::in));

        new Trigger (() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && Intake.intakeState == "stop" ||  Intake.intakeState == "in")
                .whenActive(new InstantCommand(intake::out));

        new Trigger (() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && Intake.intakeState == "in" ||  Intake.intakeState == "out")
                .whenActive(new InstantCommand(intake::stop));


        new Trigger(()-> driver2.getButton(GamepadKeys.Button.START))
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(lift::engagePTO),
                            new WaitCommand(250),
                            new InstantCommand(() -> Lift.PTO_State = "engaged")
                    ));


    }

    //this is the main run loop
    public void run() {
        super.run();

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

        //Shooter Brrrrrrrrrrr
        mS.setPower(9999999);

        //AprilTag converter equation from bearing to encoder ticks
        //

        if (tracking == "true") {
            Turret.targetPosition = Turret.getCurrentPosition() - turretBearing * BotPositions.TURRET_DEGREE_TO_TICK_MULTIPLIER;
        }


        Rotation = cubicScaling(-gamepad1.right_stick_x) * 0.5;
        FB = cubicScaling(gamepad1.left_stick_y);
        LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;

        //defines the powers for the motors based on the stick inputs (trust i've written this so many times)


        double mFLPower = FB + LR + Rotation;
        double mFRPower = FB - LR - Rotation;
        double mBLPower = FB - LR + Rotation;
        double mBRPower = FB + LR - Rotation;
        //actually sets the motor powers

        if (Lift.PTO_State == "disengaged"){
            mFL.setPower(mFLPower * CURRENT_SPEED_MULTIPLIER);
            mFR.setPower(mFRPower * CURRENT_SPEED_MULTIPLIER);
            mBL.setPower(mBLPower * CURRENT_SPEED_MULTIPLIER);
            mBR.setPower(mBRPower * CURRENT_SPEED_MULTIPLIER);
        } else if (Lift.PTO_State == "engaged" && !Lift.limitLift.isPressed()){
            mFL.setPower(1);
            mFR.setPower(1);
            mBL.setPower(1);
            mBR.setPower(1);
            Lift.mL.setPower(1);
        }else if (Lift.limitLift.isPressed()){
            mFL.setPower(0);
            mFR.setPower(0);
            mBL.setPower(0);
            mBR.setPower(0);
            Lift.mL.setPower(0);
        }



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

