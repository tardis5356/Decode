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

    public enum Direction {
        OFF,
        IN,
        OUT
    }

    public Direction currentDirection;


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

        currentDirection = Direction.OFF;


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
//        switch ((int) emptySlots){
//            case 4:
//                liT.setPosition(0);
//                break;
//
//            case 3:
//                liT.setPosition(0.333);
//                break;
//
//            case 2:
//                liT.setPosition(0.388);
//                break;
//
//            case 1:
//                liT.setPosition(0.5);
//                break;
//
//            default:
//                liT.setPosition(0.277);
//                break;
//        }

        if(GlobalVariables.currentArtifacts == "____"){
            liT.setPosition(0);
        }
        else if(currentArtifacts == "P___"){
            liT.setPosition(.227);
        }
        else if(currentArtifacts == "P_P_"){
            liT.setPosition(.388);
        }
        else if(currentArtifacts == "P_PP"){
            liT.setPosition(.5);
        }
        else{
            liT.setPosition(.72);
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
        currentDirection = Direction.IN;
    }


    public void out() {
        intakePower = -1;
        intakeState = "out";
        greenIntakeLED.setState(false);
        redIntakeLED.setState(true);
        currentDirection = Direction.OUT;
    }

    public void stop() {
        intakePower = 0;
        intakeState = "stop";
        greenIntakeLED.setState(false);
        redIntakeLED.setState(false);
        currentDirection = Direction.OFF;
    }


    public String gPST(ColorRangeSensor cs) {
        //Normalized colors return values from 1 to 0
        if(cs == cSSt){
            if (cs.getDistance(DistanceUnit.CM) < 1) {
                return "P";
            } else return "_";
        }
        else if(cs == cSSh){
            if (cs.getDistance(DistanceUnit.CM) < 5) {
                if (slotFly) {
                    return "_";
                }
                return "P";
            } else return "_";
        }
        else if(cs == cSM){
            if (cs.getDistance(DistanceUnit.CM) < 2) {
                return "P";
            } else return "_";
        }
        else if(cs == cSI){
            if (cs.getDistance(DistanceUnit.CM) < 12) {
                return "P";
            } else return "_";
        }
        else{
            return "_";
        }


    }

    public void setCurrentArtifacts() {
        currentArtifacts = gPST(cSSt) + gPST(cSSh) + gPST(cSM) + gPST(cSI);
    }

}