package Testbed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Shooter;

public class FlyWheelTuner extends CommandOpMode{

    Shooter s;
    GamepadEx d1;
    double tps;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void initialize() {
        s = new Shooter(hardwareMap);
        d1 = new GamepadEx(gamepad1);

        new Trigger(()->d1.getButton(GamepadKeys.Button.A))
                .whenActive(()->tps=1000);

        new Trigger(()->d1.getButton(GamepadKeys.Button.B))
                .whenActive(()->tps=1600);
    }

    @Override
    public void run() {
        super.run();
        s.setFlyWheelSpeed(tps);

        telemetry.addData("tps",s.getFlyWheelSpeed());
        telemetry.update();
    }
}
