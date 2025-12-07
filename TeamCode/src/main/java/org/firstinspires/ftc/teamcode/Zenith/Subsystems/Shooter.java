package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.inAuto;

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

import org.firstinspires.ftc.teamcode.Zenith.InterpolatingDoubleTreeMap;

@Config
public class Shooter extends SubsystemBase {

    //TODO: Retune these 11.6.25
    //Start by tuning vV so that your tps vs time graph approaches the set point (expect a horizontal asymptote),
    //then tune vP to speed it up and then maybe vD and vI.
    //idk if vS is necessary but that's just there so the motor is at a power always at the brink of surpassing the force of static friction.
    public static float vP = 0.0095f, vI = 0, vD = 0.0005f, vV = 0.007f, vS = 0;

    public static double neededVoltage, dutyCycle, batteryVoltage;

    private final VoltageSensor voltageSensor;


    PIDController velPIDController = new PIDController(vP, vI, vD);
    SimpleMotorFeedforward velFFController = new SimpleMotorFeedforward(vS, vV);

    public DcMotorEx mST, mSB;
    public Servo sH;

    InterpolatingDoubleTreeMap LDRegression = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap MDRegression = new InterpolatingDoubleTreeMap();

    public double flyWheelSpeed;

    boolean targeting;

    public boolean spinning;

    double distanceFromTarget;

    public Shooter(HardwareMap hardwareMap) {
        //map subsystems
        mST = hardwareMap.get(DcMotorEx.class, "mST");
        mSB = hardwareMap.get(DcMotorEx.class, "mSB");

        mST.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mSB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sH = hardwareMap.get(Servo.class, "sH");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        mSB.setDirection(DcMotorSimple.Direction.REVERSE);

        sH.setPosition(0.05);


        //prep regression data
        LDRegression.put(133., .05);
        LDRegression.put(142., .05);
        LDRegression.put(147., .05);

        MDRegression.put(115., .05);
        MDRegression.put(103., .2);
        MDRegression.put(90., .55);


        targeting = true;
    }

    @Override
    public void periodic() {

        mSB.setPower(calculateFlyWheelPower(flyWheelSpeed));
        mST.setPower(calculateFlyWheelPower(flyWheelSpeed));

        if (!inAuto) {
            if (targeting) {

                if (GlobalVariables.distanceFromTarget > 125) {
                    sH.setPosition(LDRegression.get(distanceFromTarget));
                } else if (GlobalVariables.distanceFromTarget <= 125) {
                    sH.setPosition(MDRegression.get(distanceFromTarget));
                }

            }
        } else {
            if (GlobalVariables.distanceFromTarget > 125) {
                sH.setPosition(LDRegression.get(distanceFromTarget));
            } else if (GlobalVariables.distanceFromTarget <= 100) {
                sH.setPosition(.05);
            }
        }

        if (!inAuto) {
            if (spinning) {
                if (GlobalVariables.distanceFromTarget > 125) {
                    setVel(1300);
                } else if (GlobalVariables.distanceFromTarget <= 125) {
                    setVel(1125);
                }
            } else {
                setVel(0);
            }
        } else {
            if (GlobalVariables.distanceFromTarget > 125) {
                setVel(1300);
            } else if (GlobalVariables.distanceFromTarget <= 100) {
                setVel(1000);
            }
        }


    }


    public void setVel(double tps) {
        flyWheelSpeed = tps;
    }

    public double calculateFlyWheelPower(double tps) {
        return (velPIDController.calculate(getFlyWheelSpeed(), tps) + velFFController.calculate(tps)) / voltageSensor.getVoltage();
    }

    public double getFlyWheelSpeed() {
        return mST.getVelocity();
    }

    //place this in the run loop in teleop with the input being some math calc finding the distance between the odo listed coordinate and the coordinate of the target
    public void setTargetDistance(double d) {
        distanceFromTarget = d;
    }

    public void setMidRangeAutoShooterPos() {
        targeting = false;
        sH.setPosition(BotPositions.MID_RANGE_AUTO_HOOD_POS);
        setVel(BotPositions.MID_RANGE_AUTO_FLYWHEEL_TPS);
    }

}