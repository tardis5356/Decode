package org.firstinspires.ftc.teamcode.DecodeBot.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Shooter;

@TeleOp(name="TuneFlyWheel")
public class TuneFlyWheel extends CommandOpMode {

    private Shooter shooter;
    private GamepadEx driver1;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooter(hardwareMap);
        driver1 = new GamepadEx(gamepad1);

        shooter.sH.setPosition(0);

        new Trigger(()->driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenInactive(()->shooter.setFlyWheelSpeed(1000));

        new Trigger(()->driver1.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenInactive(()->shooter.setFlyWheelSpeed(1500));

    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("TPS", shooter.getFlyWheelSpeed());
        telemetry.addData("motorPower", shooter.mST.getPower());
        telemetry.update();
    }
}
