package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Storage extends SubsystemBase {
    public Servo sG;
//    public Servo sS;
//    public Servo sK;
//    public Servo sBG;

    public boolean gateManualMode;


    //We have booleans for the states of each actuator in the storage system for the recovery builder
    public static boolean gateOpen = true, slotFly = true, kickerDown = true, backOpen = false, ballShot = true;



    //The constructor method of the class if you will. you'd write storage = new Storage(hardwaremap);
    //This will then map all of the variables/objects and run methods as innitial setup
    public Storage(HardwareMap hardwareMap){
        sG = hardwareMap.get(Servo.class,"sG");
//        sS = hardwareMap.get(Servo.class,"sS");
//        sK = hardwareMap.get(Servo.class,"sK");
//        sBG = hardwareMap.get(Servo.class, "sBG");
  closeGate();
//raiseKicker();

    }

    @Override
    public void periodic(){
        //if the slot underneath the shooter isn't empty, close the gate, otherwise open it
//        if (!gateManualMode){
//            if ((GlobalVariables.currentArtifacts.charAt(0) != '_'|| GlobalVariables.currentArtifacts.charAt(1) != '_'  || !kickerDown || !ballShot)){
//                closeGate();
//                ballShot = false;
//            }
//            else {
//                openGate();
//            }
//        }else {
//            ballShot = true;
//        }




    }

    public void openGate(){
        sG.setPosition(BotPositions.GATE_OPEN);
        gateOpen = true;
    }

    public void closeGate(){
        sG.setPosition(BotPositions.GATE_CLOSED);
        gateOpen = false;
    }
//    //Open and close Back are referring to the back gate.
////    public void openBack(){
////        sBG.setPosition(BotPositions.BACK_OPEN);
////        backOpen = true;
////    }
////    public void closeBack(){
////        sBG.setPosition(BotPositions.BACK_CLOSE);
////        backOpen = false;
////    }
//
//    public void raiseKicker(){
//        sK.setPosition(BotPositions.KICKER_UP);
//        kickerDown = false;
//    }
//
//    public void lowerKicker(){
//        sK.setPosition(BotPositions.KICKER_DOWN);
//        kickerDown = true;
//    }

//    public void storeSlot(){
//        sS.setPosition(BotPositions.SLOT_STORED);
//        slotFly = false;
//    }
//
//    public void returnSlot(){
//        sS.setPosition(BotPositions.SLOT_FLY);
//        slotFly = true;
//    }

}
