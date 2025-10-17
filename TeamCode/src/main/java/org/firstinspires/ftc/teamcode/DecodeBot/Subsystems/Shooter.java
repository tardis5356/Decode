package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DecodeBot.InterpolatingDoubleTreeMap;

public class Shooter extends SubsystemBase {

    DcMotorEx mS;
    Servo sH;

    InterpolatingDoubleTreeMap regression = new InterpolatingDoubleTreeMap();

    public double flyWheelSpeed;

    boolean targeting;

    public Shooter(HardwareMap hardwareMap){
        //map subsystems
        mS = hardwareMap.get(DcMotorEx.class,"mS");
        sH = hardwareMap.get(Servo.class,"sH");

        // TODO: Actually tune the velocity PID
        mS.setVelocityPIDFCoefficients(.02,.0001,.01,0);

        //prep regression data
        regression.put(1.,1.);
    }

    @Override
    public void periodic(){

        updateFlyWheelSpeed(flyWheelSpeed);

        if (targeting != false){

            sH.setPosition(regression.get(getTargetDistance()));


        }

    }


    private void updateFlyWheelSpeed(double rpm){
        //rotations/minute / 60 seconds/minute * 2pi to get radians/second

        mS.setVelocity(rpm/60*2*3.14159265359, AngleUnit.RADIANS);
    }

    public void setFlyWheelSpeed(double rpm){
        flyWheelSpeed = rpm;
    }

    public double getFlyWheelSpeed(){
        return mS.getVelocity(AngleUnit.RADIANS)/(2*3.14159265359)*60;
    }

    public double getTargetDistance(){
        if(GlobalVariables.aColor == "red"){
            //place holder
            return 1;
        }
        else{
            //place holder
            return 1;
        }
    }


}
