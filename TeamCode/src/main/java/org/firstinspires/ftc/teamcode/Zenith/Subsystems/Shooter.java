package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.inAuto;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage.gateOpen;

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
    public static float vP = 0.007f, vI = 0.000f, vD = 0.000f, vV = 0.000435f, vS = 0.11f;

    public static double hardWheelOffset = 0;

    public static double neededVoltage, dutyCycle, batteryVoltage;

    private final VoltageSensor voltageSensor;

    //declare and define controllers
    PIDController velPIDController = new PIDController(vP, vI, vD);
    SimpleMotorFeedforward velFFController = new SimpleMotorFeedforward(vS, vV);

    public DcMotorEx mSR, mSL;
    public Servo sH;
    public Servo liH;

    public boolean bangBangActive;

    public boolean shooterLock = false;

    public enum ShooterPreset {
        CLOSE,
        MID,
        FAR
    }

    public ShooterPreset shooterPreset = ShooterPreset.MID;

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
        mSR = hardwareMap.get(DcMotorEx.class, "mSR");
        mSL = hardwareMap.get(DcMotorEx.class, "mSL");

        mSR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mSL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sH = hardwareMap.get(Servo.class, "sH");
        liH = hardwareMap.get(Servo.class, "liH");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        mSL.setDirection(DcMotorSimple.Direction.REVERSE);

   

        HoodRegression.put(36., .9);
        WheelRegression.put(36., 600.);//for auto testing

        HoodRegression.put(50.,.83);
        HoodRegression.put(64.,.82);
        HoodRegression.put(86.6,.78);
        HoodRegression.put(92.,.75);
        HoodRegression.put(136.,.72);
        HoodRegression.put(151.,.71);

        WheelRegression.put(50.,790.);
        WheelRegression.put(64.,843.);
        WheelRegression.put(86.6,1048.);
        WheelRegression.put(92.,1113.);
        WheelRegression.put(136.,1514.);
        WheelRegression.put(151.,1637.);



//        HoodRegression.put(158., 0.73);

//        WheelRegression.put(158.0, 1320 - hardWheelOffset);


        targeting = true;
    }

    @Override
    public void periodic() {

        if (gateOpen) {
            bangBangActive = true;
        } else bangBangActive = false;

        setVel(WheelRegression.get(distanceFromTarget));

        if (targeting) {
            if (shooterLock) {
                switch (shooterPreset) {
                    case CLOSE:
                        sH.setPosition(.89 + hoodOffset);
                        break;

                    case MID:
                        sH.setPosition(.77 + hoodOffset);
                        break;

                    case FAR:
                        sH.setPosition(.72 + hoodOffset);
                        break;
                }
            } else {
                sH.setPosition(HoodRegression.get(distanceFromTarget) + hoodOffset);
            }
        } else {
            sH.setPosition(hoodOffset);
        }

        if (spinning) {
            if (shooterLock) {
                switch (shooterPreset) {
                    case CLOSE:
                        mSL.setPower(calculateBangBangFlyWheelPower(670. + speedOffset));
                        mSR.setPower(calculateBangBangFlyWheelPower(670. + speedOffset));
                        break;

                    case MID:
                        mSL.setPower(calculateBangBangFlyWheelPower(1095. + speedOffset));
                        mSR.setPower(calculateBangBangFlyWheelPower(1095. + speedOffset));
                        break;

                    case FAR:
                        mSL.setPower(calculateBangBangFlyWheelPower(1485. + speedOffset));
                        mSR.setPower(calculateBangBangFlyWheelPower(1485. + speedOffset));
                        break;

                }
            } else if (bangBangActive || !inAuto) {
                mSL.setPower(calculateBangBangFlyWheelPower(targetFlyWheelSpeed + speedOffset));
                mSR.setPower(calculateBangBangFlyWheelPower(targetFlyWheelSpeed + speedOffset));
            } else {
                mSL.setPower(calculatePIDFlyWheelPower(targetFlyWheelSpeed + speedOffset));
                mSR.setPower(calculatePIDFlyWheelPower(targetFlyWheelSpeed + speedOffset));
            }
        } else {
            mSL.setPower(0);
            mSR.setPower(0);
        }


        if (!shooterLock) {
            if (Math.abs(getTargetFlyWheelSpeed() - getFlyWheelSpeed()) < 35) {
                liH.setPosition(0.8);
            } else if (getFlyWheelSpeed() - getTargetFlyWheelSpeed() < -35) {
                liH.setPosition(0);
            } else {
                liH.setPosition(getFastPulse());
            }
        } else {
            if (shooterPreset == ShooterPreset.MID) {
                liH.setPosition(0.8);
            } else if (shooterPreset == ShooterPreset.CLOSE) {
                liH.setPosition(0);
            } else {
                liH.setPosition(getFastPulse());
            }
        }

    }


    public void setVel(double tps) {
        targetFlyWheelSpeed = tps;
    }

    public double calculatePIDFlyWheelPower(double tps) {

        double flywheelError = Math.abs(getFlyWheelSpeed() - tps);
        if (flywheelError < 30) {
            return (velPIDController.calculate(getFlyWheelSpeed(), tps) + velFFController.calculate(tps)) * 12.5 / voltageSensor.getVoltage();
        } else {
            return calculateBangBangFlyWheelPower(tps);
        }
    }

    public double calculateBangBangFlyWheelPower(double tps) {
        if (getFlyWheelSpeed() >= tps) {
            return 0;
        } else return 1;
    }


    public double getFlyWheelSpeed() {
        return mSR.getVelocity();
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


}