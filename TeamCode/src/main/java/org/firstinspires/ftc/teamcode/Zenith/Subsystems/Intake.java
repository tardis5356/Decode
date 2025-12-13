package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.currentArtifacts;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage.slotFly;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {

    public DcMotorEx mI;

    public DigitalChannel redLED;
    public DigitalChannel greenLED;

    public ColorRangeSensor cSI;
    public ColorRangeSensor cSM;
    public ColorRangeSensor cSSh;
    public ColorRangeSensor cSSt;


    public static String intakeState = new String();

    public boolean currentArtifactsEstablished;

    public double intakePower = 0;


    public Intake(HardwareMap hardwareMap) {

        mI = hardwareMap.get(DcMotorEx.class, "mI");


        cSI = hardwareMap.get(ColorRangeSensor.class, "cSI");
        cSM = hardwareMap.get(ColorRangeSensor.class, "cSM");
        cSSh = hardwareMap.get(ColorRangeSensor.class, "cSSh");
        cSSt = hardwareMap.get(ColorRangeSensor.class, "cSSt");

        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class,"green");


    }

    @Override
    public void periodic() {
        mI.setPower(intakePower);
        //mP.setPower(pathPower);

        setCurrentArtifacts();


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
        greenLED.setState(true);
        redLED.setState(false);
    }


    public void out() {
        intakePower = -1;
        intakeState = "out";
        greenLED.setState(false);
        redLED.setState(true);
    }

    public void stop() {
        intakePower = 0;
        intakeState = "stop";
        greenLED.setState(false);
        redLED.setState(false);
    }


    public String greenOrPurple(ColorRangeSensor cs) {
        //Normalized colors return values from 1 to 0
        if (cs.getDistance(DistanceUnit.CM) < 7) {
            if (cs == cSSt && slotFly) {
                return "_";
            }
            if (cs.getNormalizedColors().red < 3) {
                return "P";
            } else if (cs.getNormalizedColors().red > 10) {
                return "G";
            } else {
                return "_";
            }
        } else return "_";

    }

    public void setCurrentArtifacts() {
        currentArtifacts = greenOrPurple(cSSt) + greenOrPurple(cSSh) + greenOrPurple(cSM) + greenOrPurple(cSI);
    }

}