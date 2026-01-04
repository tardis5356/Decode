package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Zenith.InterpolatingDoubleTreeMap;

import java.util.concurrent.TimeUnit;

@Config
public class Shooter extends SubsystemBase {

    //TODO: Retune these 11.6.25
    //Start by tuning vV so that your tps vs time graph approaches the set point (expect a horizontal asymptote),
    //then tune vP to speed it up and then maybe vD and vI.
    //idk if vS is necessary but that's just there so the motor is at a power always at the brink of surpassing the force of static friction.
    public static float vP = 0.0095f, vI = 0, vD = 0.0005f, vV = 0.007f, vS = 0;

    public static double hardWheelOffset = 0;

    public static double neededVoltage, dutyCycle, batteryVoltage;

    private final VoltageSensor voltageSensor;

    //declare and define controllers
    PIDController velPIDController = new PIDController(vP, vI, vD);
    SimpleMotorFeedforward velFFController = new SimpleMotorFeedforward(vS, vV);

    public DcMotorEx mST, mSB;
    public Servo sH;
    public Servo liH;

    //InterpolatingDoubleTreeMap is a class that draws straight lines between points that you feed it.
    //Then if you ask for a point in between two other ones, it will return the value of your input along the line it drew.
    InterpolatingDoubleTreeMap HoodRegression = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap WheelRegression = new InterpolatingDoubleTreeMap();

    public double targetFlyWheelSpeed;
    public double hoodOffset;

    public int speedOffset;

    public boolean targeting;

    public boolean spinning;

    double distanceFromTarget;

    ElapsedTime time;

    public Shooter(HardwareMap hardwareMap) {
        time = new ElapsedTime();
        time.reset();

        //map subsystems
        mST = hardwareMap.get(DcMotorEx.class, "mST");
        mSB = hardwareMap.get(DcMotorEx.class, "mSB");

        mST.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mSB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sH = hardwareMap.get(Servo.class, "sH");
        liH = hardwareMap.get(Servo.class, "liH");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        mSB.setDirection(DcMotorSimple.Direction.REVERSE);

        sH.setPosition(0.05);

// OLD SHOOTING DATA
        //prep regression data
        //small triangle
//        HoodRegression.put(152., .05);
//        HoodRegression.put(126.7, .078);
//        HoodRegression.put(96.6, .05);
//        HoodRegression.put(75.7, .071);
//        HoodRegression.put(54.5,.1);
//        HoodRegression.put(34.5,.1);
//        HoodRegression.put(18.,.85);
//
//        //MDRegression.put(115., .05);
//        WheelRegression.put(152., 1450.-hardWheelOffset);
//        WheelRegression.put(149.,1450.-hardWheelOffset);
//        //WheelRegression.put(126.7, 1325.-hardWheelOffset);
//        //TODO: CHECK THESE POINTS
//        WheelRegression.put(126.7, 1365.-hardWheelOffset);
//        WheelRegression.put(120.,1365-hardWheelOffset);
//        WheelRegression.put(110.,1100-hardWheelOffset);
//
//        //TODO:Check this point
//        WheelRegression.put(120.,1350-hardWheelOffset);
//        WheelRegression.put(110.,1100-hardWheelOffset);
//
//        WheelRegression.put(96.6,1100.-hardWheelOffset);
//        WheelRegression.put(75.7,1075.-hardWheelOffset);
//        WheelRegression.put(54.5,975.-hardWheelOffset);
//        WheelRegression.put(34.5,975.-hardWheelOffset);
//        WheelRegression.put(18.,850.-hardWheelOffset);




       ////NEW SHOOTING DATA 1/4/26
        HoodRegression.put(139.8, 0.1);
        HoodRegression.put(118.6, 0.1);
        HoodRegression.put(109.0, 0.15);
        HoodRegression.put(100.4, 0.2);
        HoodRegression.put(90.0, 0.2);
        HoodRegression.put(79.7, 0.2);
        HoodRegression.put(62.0, 0.25);
        HoodRegression.put(52.5, 0.4);
        HoodRegression.put(42.5, 0.525);
        HoodRegression.put(32.8, 0.625);


        WheelRegression.put(139.8, 1410.-hardWheelOffset);
        WheelRegression.put(118.6, 1310.-hardWheelOffset);
        WheelRegression.put(109.0, 1230.-hardWheelOffset);
        WheelRegression.put(100.4, 1190.-hardWheelOffset);
        WheelRegression.put(90.0, 1090.-hardWheelOffset);
        WheelRegression.put(79.7, 1010.-hardWheelOffset);
        WheelRegression.put(62.0, 990.-hardWheelOffset);
        WheelRegression.put(52.5, 870.-hardWheelOffset);
        WheelRegression.put(42.5, 810.-hardWheelOffset);
        WheelRegression.put(32.8, 790.-hardWheelOffset);



        targeting = true;
    }

    @Override
    public void periodic() {

        mSB.setPower(calculateFlyWheelPower(targetFlyWheelSpeed + speedOffset));
        mST.setPower(calculateFlyWheelPower(targetFlyWheelSpeed + speedOffset));

            if (targeting) {
                sH.setPosition(HoodRegression.get(distanceFromTarget) + hoodOffset);
            }
            else{
                sH.setPosition(hoodOffset);
            }

            if (spinning) {
                setVel(WheelRegression.get(distanceFromTarget));
            } else {
                setVel(0);
            }

            if (Math.abs(getTargetFlyWheelSpeed()-getFlyWheelSpeed()) < 35) {
                liH.setPosition(0.8);
            } else if (getFlyWheelSpeed()-getTargetFlyWheelSpeed() < -35) {
                liH.setPosition(0);
            } else {
                liH.setPosition(getFastPulse());
            }

    }


    public void setVel(double tps) {
        targetFlyWheelSpeed = tps;
    }

    public double calculateFlyWheelPower(double tps) {
        return (velPIDController.calculate(getFlyWheelSpeed(), tps) + velFFController.calculate(tps)) / voltageSensor.getVoltage();
    }

    public double getFlyWheelSpeed() {
        return mST.getVelocity();
    }

    public double getTargetFlyWheelSpeed() {
        return targetFlyWheelSpeed;
    }

    //place this in the run loop in teleop with the input being some math calc finding the distance between the odo listed coordinate and the coordinate of the target
    public void setTargetDistance(double d) {
        distanceFromTarget = d;
    }

    public double getFastPulse() {
        return (Math.sin(2 * Math.PI * time.time(TimeUnit.SECONDS) - 0.5 * Math.PI) + 1) / 5;
    }

    public void setMidRangeAutoShooterPos() {
        targeting = false;
        sH.setPosition(BotPositions.MID_RANGE_AUTO_HOOD_POS);
        setVel(BotPositions.MID_RANGE_AUTO_FLYWHEEL_TPS);
    }

}