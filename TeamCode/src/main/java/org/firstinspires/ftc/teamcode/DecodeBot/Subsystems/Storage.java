package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Storage extends SubsystemBase {
    public Servo sG;
    public Servo sS;
    public Servo sK;
    public Servo sBG;
    public static boolean gateOpen = true, slotFly = true, kickerDown = true, backOpen = false;

    public Storage(HardwareMap hardwareMap){
        sG = hardwareMap.get(Servo.class,"sG");
        sS = hardwareMap.get(Servo.class,"sS");
        sK = hardwareMap.get(Servo.class,"sK");
        sBG = hardwareMap.get(Servo.class, "sBG");
        openGate();
returnSlot();
        closeBack();
    }

    @Override
    public void periodic(){
//if (GlobalVariables.currentArtifacts.charAt(1) != '_'){
//    closeGate();
//}
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
        sBG.setPosition(BotPositions.BACK_OPEN);
        backOpen = true;
    }
    public void closeBack(){
        sBG.setPosition(BotPositions.BACK_CLOSE);
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
