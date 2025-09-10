package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {





    DcMotorEx mI;


    public static String intakeState = new String();

    double motorPower = 0;

    public Intake(HardwareMap hardwareMap){

        mI = hardwareMap.get(DcMotorEx.class, "mI");



    }

    @Override
    public void periodic(){
     mI.setPower(motorPower);
    }

    public void in(){
       motorPower = 1;
       intakeState = "in";
    }

    public void out(){
        motorPower = -1;
        intakeState = "out";
    }

    public void stop(){
        motorPower = 0;
        intakeState = "stop";
    }

}