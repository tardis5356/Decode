package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift extends SubsystemBase {

    public static DcMotorEx mL;

    public static Servo sP;
    //touch sensor near the bottom of the lift slides used to localize the lift encoder
//and prevent the lift from driving into the deck plate
    public static String PTO_State = new String();
    public static TouchSensor limitLift;
    public double motorPower;

    //hardwaremap virtual components to configuration
    public Lift(HardwareMap hardwareMap) {

        mL = hardwareMap.get(DcMotorEx.class, "mL");
        sP = hardwareMap.get(Servo.class, "sP");
        limitLift = hardwareMap.get(TouchSensor.class, "lL");
        //mLT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//at the start of teleop reset the encoder value to 0 (localize it)
        mL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sP.setPosition(BotPositions.PTO_DISENGAGED);
        mL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PTO_State = "disengaged";
    }


    public void periodic() {
        // runs every loop




        //}

    }

    //these are a few telemetry methods for trouble shooting

    public void engagePTO() {
        sP.setPosition(BotPositions.PTO_ENGAGED);

        PTO_State = "engaged";
    }



    public double getCurrentMotorPower() {
        return motorPower;
    }
}


//this was for auto with so that we could do a stage one hang by letting the motors sag down to the lower bar.
//it never worked and I still don't know why

