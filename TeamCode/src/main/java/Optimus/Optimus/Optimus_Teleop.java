package Optimus.Optimus;

//import static org.firstinspires.ftc.teamcode.Optimus.Subsystems.OptimusBotPositions.ARM_INDEX_POSITION;
//import static org.firstinspires.ftc.teamcode.Optimus.Subsystems.OptimusBotPositions.ARM_LOWER_LIMIT;
//import static org.firstinspires.ftc.teamcode.Optimus.Subsystems.OptimusBotPositions.ARM_UPPER_LIMIT;
//import static org.firstinspires.ftc.teamcode.Optimus.Subsystems.OptimusBotPositions.WRIST_RESET_POSITION;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import Optimus.Optimus.Subsystems.Arm;
import Optimus.Optimus.Subsystems.Gripper;
import Optimus.Optimus.Subsystems.OptimusBotPositions;

//import org.firstinspires.ftc.teamcode.Optimus.Optimus.Subsystems.Arm;
//import org.firstinspires.ftc.teamcode.Optimus.Subsystems.Gripper;
//import org.firstinspires.ftc.teamcode.Optimus.Subsystems.OptimusBotPositions;

@TeleOp(name = "Optimus_Teleop")

public class Optimus_Teleop extends CommandOpMode {
    //declaration of parts
    DcMotor mR, mL, mA;

    Servo sW;

    Boolean tooFar;

    TouchSensor armLimit;

    GamepadEx driver1, driver2;

    Gripper gripper;

    Arm arm;

    public double ForwardBackward, Rotation, armPosition, positionDiff;

    public boolean wristDown;

    @Override
    public void initialize() {
        //hardware map
        mR = hardwareMap.get(DcMotor.class, "mR");
        mL = hardwareMap.get(DcMotor.class, "mL");

        mL.setDirection(DcMotorSimple.Direction.REVERSE);

        gripper = new Gripper(hardwareMap);

        mA = hardwareMap.get(DcMotor.class, "mA");
        armLimit = hardwareMap.get(TouchSensor.class, "armLimit");

        sW = hardwareMap.get(Servo.class, "sW");

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        //triggers
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER) || driver2.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .toggleWhenActive(gripper::open, gripper::close);

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::up),
                                new InstantCommand(() -> wristDown = false)
                        )
                );

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT) || driver2.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive(
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::middle),
                                new InstantCommand(() -> wristDown = false)
                        )
                );

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenActive(
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::down),
                                new InstantCommand(() -> wristDown = true)
                        )
                );

    }

    @Override
    public void run() {
        //teleop specific logic
        super.run();

        ForwardBackward = antiDrift(driver1.getLeftY());
        Rotation = antiDrift(driver1.getRightX());

        mR.setPower(ForwardBackward - Rotation);
        mL.setPower(ForwardBackward + Rotation);

        if (armLimit.isPressed()) {
            armPosition = OptimusBotPositions.ARM_INDEX_POSITION;
            positionDiff = mA.getCurrentPosition() - armPosition;
//            if (driver2.getLeftY()<0){
//                mA.setPower(0);
//            }
        }

        armPosition = mA.getCurrentPosition() - positionDiff;

        if (armPosition > OptimusBotPositions.ARM_LOWER_LIMIT && driver2.getLeftY() > 0) {
            mA.setPower(0);
            tooFar = true;

        } else if (armPosition < OptimusBotPositions.ARM_UPPER_LIMIT && driver2.getLeftY() < 0) {
            mA.setPower(0);

        } else {
            mA.setPower(driver2.getLeftY());
            tooFar = false;
        }

        if (armPosition > OptimusBotPositions.WRIST_RESET_POSITION && wristDown) {

            sW.setPosition(OptimusBotPositions.GRIPPER_MIDDLE);

        }

        telemetry.addData("armPower", mA.getPower());
        telemetry.addData("armPosition", armPosition);
        telemetry.addData("FB", ForwardBackward);
        telemetry.addData("Rotation", Rotation);
        telemetry.addData("mR_Power", mR.getPower());
        telemetry.addData("mL_Power", mL.getPower());
        telemetry.addData("LeftStickY", driver1.getLeftY());
        telemetry.addData("RightStickX", driver1.getRightX());
        telemetry.addData("TooFarForward", tooFar);
        telemetry.update();
    }

    private double antiDrift(double stickVal) {

        if (stickVal < -.02 || stickVal > .02) {
            return stickVal;
        } else {
            return 0;
        }
    }
}
