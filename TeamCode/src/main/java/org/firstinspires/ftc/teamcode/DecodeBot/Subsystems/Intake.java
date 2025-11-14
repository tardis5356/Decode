package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    DcMotorEx mI;

    ColorSensor cSI, cSM, cSSh, cSSt;

    public static String intakeState = new String();

    boolean currentArtifactsEstablished;

    double intakePower = 0, pathPower = 0;

    public Intake(HardwareMap hardwareMap){

        mI = hardwareMap.get(DcMotorEx.class, "mI");


        cSI = hardwareMap.get(ColorSensor.class, "cSI");
        cSM = hardwareMap.get(ColorSensor.class, "cSM");
        cSSh = hardwareMap.get(ColorSensor.class, "cSSh");
        cSSt = hardwareMap.get(ColorSensor.class,"cSSt");

    }

    @Override
    public void periodic(){
        mI.setPower(intakePower);
        //mP.setPower(pathPower);

//        if(GlobalVariables.currentArtifacts.substring(1) == "PPG" || GlobalVariables.currentArtifacts.substring(1) == "PGP" || GlobalVariables.currentArtifacts.substring(1) == "GPP"){
//            currentArtifactsEstablished = true;
//        }

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
        //TODO: actually set the rgb ranges
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
        GlobalVariables.currentArtifacts = greenOrPurple(cSSt) + greenOrPurple(cSSh) + greenOrPurple(cSM) + greenOrPurple(cSI);
    }

}