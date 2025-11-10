package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Storage extends SubsystemBase {
    Servo sG, sS, sK, sB;
    public static boolean gateOpen, slotFly, kickerDown, backOpen;

    public Storage(HardwareMap hardwareMap){
        sG = hardwareMap.get(Servo.class,"sG");
        sS = hardwareMap.get(Servo.class,"sS");
        sK = hardwareMap.get(Servo.class,"sK");
        sB = hardwareMap.get(Servo.class, "sB");

        closeBack();
    }

    @Override
    public void periodic(){

    }

    public void openGate(){
        sG.setPosition(BotPositions.GATE_OPEN);
        gateOpen = true;
    }

    public void closeGate(){
        sG.setPosition(BotPositions.GATE_CLOSED);
        gateOpen = false;
    }

    public void openBack(){
        sB.setPosition(BotPositions.BACK_OPEN);
        backOpen = true;
    }
    public void closeBack(){
        sB.setPosition(BotPositions.BACK_CLOSE);
        backOpen = false;
    }

    public void raiseKicker(){
        sK.setPosition(BotPositions.KICKER_UP);
        kickerDown = false;
    }

    public void lowerKicker(){
        sK.setPosition(BotPositions.KICKER_DOWN);
        kickerDown = true;
    }

    public void storeSlot(){
        sS.setPosition(BotPositions.SLOT_STORED);
        slotFly = false;
    }

    public void returnSlot(){
        sS.setPosition(BotPositions.SLOT_FLY);
        slotFly = true;
    }

}
