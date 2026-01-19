package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BrakePad extends SubsystemBase {
    Servo sB;
    public static boolean breakPadEngaged;

    public BrakePad(HardwareMap hardwareMap){
        sB = hardwareMap.get(Servo.class,"sB");
                retract();

    }



    public void periodic() {
        // runs every loop



    }


    public void deploy(){
        sB.setPosition(BotPositions.BREAKPAD_ACTIVE);
        breakPadEngaged = true;
    }

    public void retract(){
        sB.setPosition(BotPositions.BREAKPAD_INACTIVE);
        breakPadEngaged = false;
    }

}
