package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.currentArtifacts;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage.slotFly;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {

    public DcMotorEx mI;

    public Servo liT;

    public DigitalChannel redIntakeLED;
    public DigitalChannel greenIntakeLED;

    public ColorRangeSensor cSI;
    public ColorRangeSensor cSM;
    public ColorRangeSensor cSSh;
    public ColorRangeSensor cSSt;


    public static String intakeState = new String();

    public boolean currentArtifactsEstablished;

    public double intakePower = 0;


    public Intake(HardwareMap hardwareMap) {

        mI = hardwareMap.get(DcMotorEx.class, "mI");
        liT = hardwareMap.get(Servo.class, "liT");

        cSI = hardwareMap.get(ColorRangeSensor.class, "cSI");
        cSM = hardwareMap.get(ColorRangeSensor.class, "cSM");
        cSSh = hardwareMap.get(ColorRangeSensor.class, "cSSh");
        cSSt = hardwareMap.get(ColorRangeSensor.class, "cSSt");

        redIntakeLED = hardwareMap.get(DigitalChannel.class, "iR");
        greenIntakeLED = hardwareMap.get(DigitalChannel.class,"iG");

        redIntakeLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenIntakeLED.setMode(DigitalChannel.Mode.OUTPUT);


    }

    @Override
    public void periodic() {
        mI.setPower(intakePower);
        //mP.setPower(pathPower);

        setCurrentArtifacts();


        long emptySlots = GlobalVariables.currentArtifacts.chars()
                .filter(c -> c == '_')
                .count();

        switch ((int) emptySlots){
            case 4:
                liT.setPosition(0);
                break;

            case 3:
                liT.setPosition(0.333);
                break;

            case 2:
                liT.setPosition(0.388);
                break;

            case 1:
                liT.setPosition(0.5);
                break;

            default:
                liT.setPosition(0.277);
                break;
        }

        // If less than 2 empty slots → STOP the intake
//        if (emptySlots == 1 ) {
//            stop();
//        }


    }

    public void in() {
        intakePower = 1;
        intakeState = "in";
        greenIntakeLED.setState(true);
        redIntakeLED.setState(false);
    }


    public void out() {
        intakePower = -1;
        intakeState = "out";
        greenIntakeLED.setState(false);
        redIntakeLED.setState(true);
    }

    public void stop() {
        intakePower = 0;
        intakeState = "stop";
        greenIntakeLED.setState(false);
        redIntakeLED.setState(false);
    }


    public String greenOrPurple(ColorRangeSensor cs) {
        //Normalized colors return values from 1 to 0
        if (cs.getDistance(DistanceUnit.CM) < 5) {
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