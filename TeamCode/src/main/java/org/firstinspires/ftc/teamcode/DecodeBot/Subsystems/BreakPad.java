package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BreakPad extends SubsystemBase {
    Servo sB;

    public BreakPad (HardwareMap hardwareMap){
        sB = hardwareMap.get(Servo.class,"sB");
    }



    public void periodic() {
        // runs every loop



    }


    public void deployBreakPad(){
        sB.setPosition(BotPositions.BREAKPAD_ACTIVE);
    }

    public void retractBreakPad(){
        sB.setPosition(BotPositions.BREAKPAD_INACTIVE);
    }

}
