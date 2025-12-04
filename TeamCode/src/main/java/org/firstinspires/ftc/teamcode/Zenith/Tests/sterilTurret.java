package org.firstinspires.ftc.teamcode.Zenith.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "steril_Turret")

public class sterilTurret extends CommandOpMode {

    double motorPower;
    DcMotor mT;
    @Override
    public void initialize() {
        mT = hardwareMap.get(DcMotor.class,"mT");

    }
    @Override
    public void run() {
        super.run();
        if(gamepad1.x){
            motorPower = .1;
        }
        else if(gamepad1.y){
            motorPower = -.1;
        }

        mT.setPower(motorPower);

        telemetry.addData("MotorDetectedPower", mT.getPower());
        telemetry.addData("CommandedPower",motorPower);
        telemetry.update();

    }

}
