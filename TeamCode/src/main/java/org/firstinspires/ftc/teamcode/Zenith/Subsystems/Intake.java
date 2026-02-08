package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.currentArtifacts;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Zenith.TeleOps.DecodeTeleOp;

public class Intake extends SubsystemBase {

    public DcMotorEx mI;

    public Servo liT;

    public boolean intakeFull;

    public DigitalChannel redIntakeLED;
    public DigitalChannel greenIntakeLED;

    public DigitalChannel bbF;
    public DigitalChannel bbM;
    public DigitalChannel bbSh;

    private boolean shLatched = false;
    private boolean mLatched  = false;
    private boolean fLatched = false;

    private long shLastBroken = 0;
    private long mLastBroken  = 0;
    private long fLastBroken = 0;

    private ElapsedTime timeIntakeFull = new ElapsedTime();

    private static final long CLEAR_TIME_MS = 75; // tweak if needed


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

        bbF = hardwareMap.get(DigitalChannel.class, "bbF");
        bbM = hardwareMap.get(DigitalChannel.class, "bbM");
        bbSh = hardwareMap.get(DigitalChannel.class, "bbSh");


        redIntakeLED = hardwareMap.get(DigitalChannel.class, "iR");
        greenIntakeLED = hardwareMap.get(DigitalChannel.class,"iG");

        bbF.setMode(DigitalChannel.Mode.INPUT);
        bbM.setMode(DigitalChannel.Mode.INPUT);
        bbSh.setMode(DigitalChannel.Mode.INPUT);


        redIntakeLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenIntakeLED.setMode(DigitalChannel.Mode.OUTPUT);

        currentDirection = Direction.OFF;
timeIntakeFull.reset();

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
            case 3:
                liT.setPosition(0);
                break;

            case 2:
                liT.setPosition(0.333);
                break;

            case 1:
                liT.setPosition(0.388);
                break;

            case 0:
                liT.setPosition(0.5);
                break;

            default:
                liT.setPosition(0.277);
                break;
        }

//        if(Objects.equals(currentArtifacts, "____")){
//            liT.setPosition(0);
//        }
//        else if(Objects.equals(currentArtifacts, "P___")){
//            liT.setPosition(.227);
//        }
//        else if(Objects.equals(currentArtifacts, "P_P_")){
//            liT.setPosition(.388);
//        }
//        else if(Objects.equals(currentArtifacts, "P_PP")){
//            liT.setPosition(.5);
//        }
//        else{
//            liT.setPosition(.72);
//        }

        if (emptySlots != 0){
            timeIntakeFull.reset();
        }

        if (timeIntakeFull.seconds() >0.5 && currentDirection != Direction.OFF && !DecodeTeleOp.firing) {
            stop();
        }


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


    public String getIntakeState(DigitalChannel bb) {
        long now = System.currentTimeMillis();
        boolean beamBroken = bb.getState(); // true = broken

        if (bb == bbSh) {
            if (!beamBroken) {
                shLatched = true;
                shLastBroken = now;
            } else if (shLatched && now - shLastBroken > CLEAR_TIME_MS) {
                shLatched = false;
            }
            return shLatched ? "P" : "_";
        }

        if (bb == bbM) {
            if (!beamBroken) {
                mLatched = true;
                mLastBroken = now;
            } else if (mLatched && now - mLastBroken > CLEAR_TIME_MS) {
                mLatched = false;
            }
            return mLatched ? "P" : "_";
        }

        if (bb == bbF) {
            if (!beamBroken) {
                fLatched = true;
                fLastBroken = now;
            } else if (fLatched && now - fLastBroken > CLEAR_TIME_MS) {
                fLatched = false;
            }
            return fLatched ? "P" : "_";
        }

        return "_";
    }

    public void setCurrentArtifacts() {
        currentArtifacts = getIntakeState(bbSh) + getIntakeState(bbM) + getIntakeState(bbF);
    }

}