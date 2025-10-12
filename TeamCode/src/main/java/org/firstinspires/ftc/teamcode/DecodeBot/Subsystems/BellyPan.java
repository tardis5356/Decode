package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.PTO_DISENGAGED;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.PTO_ENGAGED;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class BellyPan extends SubsystemBase {

    //public static DcMotorEx mL;

    public Servo sP;

    public Servo sRL;
    public Servo sLL;


    //touch sensor near the bottom of the lift slides used to localize the lift encoder
//and prevent the lift from driving into the deck plate
    public static boolean PTO_Engaged;
    public static TouchSensor limitLift;
    public double motorPower;

    //hardwaremap virtual components to configuration
    public BellyPan(HardwareMap hardwareMap) {
        sP = hardwareMap.get(Servo.class, "sP");
        sLL = hardwareMap.get(Servo.class, "sLL");
        sRL = hardwareMap.get(Servo.class, "sRL");
        limitLift = hardwareMap.get(TouchSensor.class, "lL");
        sP.setPosition(PTO_DISENGAGED);
        PTO_Engaged = false;

    }


    public void periodic() {
        // runs every loop




        //}

    }

    //these are a few telemetry methods for trouble shooting

    public void engagePTO() {
        sP.setPosition(PTO_ENGAGED);

        PTO_Engaged = true;
    }



    public double getCurrentMotorPower() {
        return motorPower;
    }
}


//this was for auto with so that we could do a stage one hang by letting the motors sag down to the lower bar.
//it never worked and I still don't know why

