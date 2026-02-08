package org.firstinspires.ftc.teamcode.Zenith.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Shooter;

@TeleOp(name="TuneFlyWheel")
@Disabled
public class TuneFlyWheel extends CommandOpMode {

    private Shooter shooter;
    private Intake intake;
    private GamepadEx driver1;

    double MP;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooter(hardwareMap);
        driver1 = new GamepadEx(gamepad1);

        intake = new Intake(hardwareMap);


        shooter.sH.setPosition(0);

        new Trigger(()->driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenInactive(()->MP+=.05);

        new Trigger(()->driver1.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenInactive(()->MP-=.05);

        new Trigger(()->driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .whenActive(new InstantCommand(intake::in));

        new Trigger(()->driver1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .whenActive(new InstantCommand(intake::stop));


    }

    @Override
    public void run() {
        super.run();

        shooter.mSR.setPower(MP);
        shooter.mSL.setPower(MP);

        telemetry.addData("TPS", shooter.getFlyWheelSpeed());
        telemetry.addData("motorPower", shooter.mSR.getPower());
        telemetry.update();
    }
}
