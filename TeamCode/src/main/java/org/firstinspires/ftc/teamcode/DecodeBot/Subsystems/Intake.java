package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    DcMotorEx mI, mP;

    ColorSensor cSI, cSM, cSS;

    public static String intakeState = new String();

    boolean currentArtifactsEstablished;

    double intakePower = 0, pathPower = 0;

    public Intake(HardwareMap hardwareMap){

        mI = hardwareMap.get(DcMotorEx.class, "mI");
        mP = hardwareMap.get(DcMotorEx.class, "mP");

        cSI = hardwareMap.get(ColorSensor.class, "cSI");
        cSM = hardwareMap.get(ColorSensor.class, "cSM");
        cSS = hardwareMap.get(ColorSensor.class, "cSS");


    }

    @Override
    public void periodic(){
        mI.setPower(intakePower);
        mP.setPower(pathPower);

        if(GlobalVariables.currentArtifacts == "PPG" || GlobalVariables.currentArtifacts == "PGP" || GlobalVariables.currentArtifacts == "GPP"){
            currentArtifactsEstablished = true;
        }

        if(!currentArtifactsEstablished){
            setCurrentArtifacts();
        }

    }

    public void in(){
       intakePower = 1;
       pathPower = 1;
       intakeState = "in";
    }

    public void oneOut(){
        intakePower = -1;
        intakeState = "out";
    }

    public void allOut(){
        intakePower = -1;
        pathPower = -1;
        intakeState = "out";
    }
    public void stop(){
        intakePower = 0;
        pathPower = 0;
        intakeState = "stop";
    }

    public String greenOrPurple(ColorSensor cs){
        if (cs.red()>10){
            return "P";
        }
        else if(cs.red()<=10){
            return "G";
        }
        else{
            return "V";
        }
    }

    public void setCurrentArtifacts(){
        GlobalVariables.currentArtifacts = greenOrPurple(cSS) + greenOrPurple(cSM) + greenOrPurple(cSI);
    }

}