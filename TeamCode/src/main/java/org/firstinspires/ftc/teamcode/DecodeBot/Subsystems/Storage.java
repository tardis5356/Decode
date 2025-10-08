package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Storage extends SubsystemBase {
    Servo sG, sS, sK;

    public Storage(HardwareMap hardwareMap){
        sG = hardwareMap.get(Servo.class,"sG");
        sS = hardwareMap.get(Servo.class,"sS");
        sK = hardwareMap.get(Servo.class,"sK");
    }

    @Override
    public void periodic(){

    }

    public void openGate(){
        sG.setPosition(BotPositions.GATE_OPEN);
    }

    public void closeGate(){
        sG.setPosition(BotPositions.GATE_CLOSED);
    }

    public void raiseKicker(){
        sK.setPosition(BotPositions.KICKER_UP);
    }

    public void lowerKicker(){
        sK.setPosition(BotPositions.KICKER_DOWN);
    }

    public void storeSlot(){
        sS.setPosition(BotPositions.SLOT_STORED);
    }

    public void returnSlot(){
        sS.setPosition(BotPositions.SLOT_FLY);
    }

}
