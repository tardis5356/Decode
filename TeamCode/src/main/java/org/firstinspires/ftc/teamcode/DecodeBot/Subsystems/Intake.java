package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {





    DcMotorEx mI;





    double power = 0;

    public Intake(HardwareMap hardwareMap){

        mI = hardwareMap.get(DcMotorEx.class, "mI");



    }

    @Override
    public void periodic(){
     mI.setPower(power);
    }

    public void in(){
       power = 1;
    }

    public void out(){
        power = -1;
    }

    public void stop(){
        power = 0;
    }

}