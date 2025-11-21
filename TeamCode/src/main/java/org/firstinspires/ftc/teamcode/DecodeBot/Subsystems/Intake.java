package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.currentArtifacts;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Storage.slotFly;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    DcMotorEx mI;

    public ColorRangeSensor cSI;
    public ColorRangeSensor cSM;
    public ColorRangeSensor cSSh;
    public ColorRangeSensor cSSt;


    public static String intakeState = new String();

    public boolean currentArtifactsEstablished;

    double intakePower = 0;


    public Intake(HardwareMap hardwareMap) {

        mI = hardwareMap.get(DcMotorEx.class, "mI");


        cSI = hardwareMap.get(ColorRangeSensor.class, "cSI");
        cSM = hardwareMap.get(ColorRangeSensor.class, "cSM");
        cSSh = hardwareMap.get(ColorRangeSensor.class, "cSSh");
        cSSt = hardwareMap.get(ColorRangeSensor.class, "cSSt");


    }

    @Override
    public void periodic() {
        mI.setPower(intakePower);
        //mP.setPower(pathPower);
        if (GlobalVariables.currentArtifacts.substring(1) == "PPG" || GlobalVariables.currentArtifacts.substring(1) == "PGP" || GlobalVariables.currentArtifacts.substring(1) == "GPP") {
            currentArtifactsEstablished = true;
        } else {
            currentArtifactsEstablished = false;
        }

        if (!currentArtifactsEstablished) {
            setCurrentArtifacts();
        }


//        long emptySlots = GlobalVariables.currentArtifacts.chars()
//                .filter(c -> c == '_')
//                .count();
//
//        // If less than 2 empty slots → STOP the intake
//        if (emptySlots < 2) {
//            stop();
//        }


    }

    public void in() {
        intakePower = 1;
        intakeState = "in";
    }


    public void out() {
        intakePower = -1;
        intakeState = "out";
    }

    public void stop() {
        intakePower = 0;
        intakeState = "stop";
    }



    public String greenOrPurple(ColorRangeSensor cs) {
//Normalized colors return values from 1 to 0

        if (cs == cSSt && slotFly) {
            return "_";
        }
        if (cs.getNormalizedColors().red < 3) {
            return "G";
        } else if (cs.getNormalizedColors().red > 10) {
            return "P";
        } else {
            return "_";
        }
    }

    public void setCurrentArtifacts() {
        currentArtifacts = greenOrPurple(cSSt) + greenOrPurple(cSSh) + greenOrPurple(cSM) + greenOrPurple(cSI);
    }

}