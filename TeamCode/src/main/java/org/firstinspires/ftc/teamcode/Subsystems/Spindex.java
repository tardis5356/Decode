package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Spindex extends SubsystemBase {

    //Axon servos in continuous rotation mode for the spindex gearbox
    CRServo sSL, sSR;

    //this is just for the encoder of the spindex
    DcMotorEx eS;

    ColorSensor cS1, cS2, cS3;

    PDController spindexControl;

    double spindexTarget, power;

    public Spindex(HardwareMap hardwareMap){
        sSL = hardwareMap.get(CRServo.class,"sSL");
        sSR = hardwareMap.get(CRServo.class, "sSR");
        eS = hardwareMap.get(DcMotorEx.class, "mFR");
        cS1 = hardwareMap.get(ColorSensor.class, "cS1");
        cS2 = hardwareMap.get(ColorSensor.class, "cS2");
        cS3 = hardwareMap.get(ColorSensor.class, "cS3");

        spindexControl = new PDController(.1,.01);

        //NOTE TO SELF: This logic regarding when you set the spindexTarget to equal the current position during initialization will likely change
        //Porobably have an indipendent method for it
        spindexTarget = eS.getCurrentPosition();
    }

    @Override
    public void periodic(){
        power = spindexControl.calculate(eS.getCurrentPosition(), spindexTarget);
        sSR.setPower(power);
        sSL.setPower(power);
    }

    //only call this via trigger from driver input or not continuously during auto
    //should be .whenActiveOnce
    public void spinMotif(Slot s1, Slot s2, Slot s3){
        if(GlobalVariables.motif == "ppg"){
            if(s1.artifactColor == "purple" && s2.artifactColor == "purple" && s3.artifactColor == "green"){
                //set target to be x amount more clockwise
                spindexTarget += 60;
            }
            else if(s1.artifactColor == "purple" && s2.artifactColor == "green" && s3.artifactColor == "purple"){
                //set target to be x amount more counterclockwise
                spindexTarget -= 60;
            }
            else if(s1.artifactColor == "green" && s2.artifactColor == "purple" && s3.artifactColor == "purple"){
                //either way doesn't matter but there needs to be a flag or something that is set active here to tell the boot not to kick for a sed
                spindexTarget -= 60;
            }
        }
        else if(GlobalVariables.motif == "pgp"){

        }

    }

}
