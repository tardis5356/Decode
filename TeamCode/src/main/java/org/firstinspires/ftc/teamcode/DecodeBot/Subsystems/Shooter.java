package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DecodeBot.InterpolatingDoubleTreeMap;

@Config
public class Shooter extends SubsystemBase {

    public static float vP = 4f, vI = 1f, vD = 2f;

    public DcMotorEx mS;
    public Servo sH;

    InterpolatingDoubleTreeMap LDRegression = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap SDRegression = new InterpolatingDoubleTreeMap();

    public double flyWheelSpeed;

    boolean targeting;

    double distanceFromTarget;

    public Shooter(HardwareMap hardwareMap){
        //map subsystems
        mS = hardwareMap.get(DcMotorEx.class,"mS");
        sH = hardwareMap.get(Servo.class,"sH");

        // TODO: Actually tune the velocity PID
        mS.setVelocityPIDFCoefficients(vP,vI,vD,0);

        //prep regression data
        LDRegression.put(1.,1.);

        SDRegression.put(1.,1.);

        targeting = true;
    }

    @Override
    public void periodic(){

        updateFlyWheelSpeed(flyWheelSpeed);

        if (targeting){

            if( mS.getVelocity() < (BotPositions.LONG_DISTANCE_TPS-30) && mS.getVelocity() > (BotPositions.LONG_DISTANCE_TPS+30)){
                sH.setPosition(LDRegression.get(distanceFromTarget));
            }
            else if (mS.getVelocity() < (BotPositions.SHORT_DISTANCE_TPS-30) && mS.getVelocity() > (BotPositions.SHORT_DISTANCE_TPS+30)){
                sH.setPosition(SDRegression.get(distanceFromTarget));
            }

        }


    }


    private void updateFlyWheelSpeed(double tps){
        //rotations/minute / 60 seconds/minute * 2pi to get radians/second

        mS.setVelocity(tps);
    }

    public void setFlyWheelSpeed(double tps){
        flyWheelSpeed = tps;

    }

    public double getFlyWheelSpeed(){
        return mS.getVelocity();
    }

    public void setTargetDistance(double d){
        distanceFromTarget = d;
    }


}
