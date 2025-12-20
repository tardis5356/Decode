package org.firstinspires.ftc.teamcode.Zenith.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Turret;

@TeleOp(name="TurretTuner")
@Disabled
@Config
public class TuneTurretPID extends CommandOpMode {

    double targetPos;
    Turret turret;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        turret = new Turret(hardwareMap);
    }
    @Override
    public void run(){
        super.run();
        dashboard.updateConfig();
    }

}
