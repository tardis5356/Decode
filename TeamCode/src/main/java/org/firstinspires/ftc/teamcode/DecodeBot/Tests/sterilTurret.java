package org.firstinspires.ftc.teamcode.DecodeBot.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

    }

}
