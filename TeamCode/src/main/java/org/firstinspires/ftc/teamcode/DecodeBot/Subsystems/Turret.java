package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_D;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_I;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_P;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.DecodeBot.DecodeTeleOp;

public class Turret extends SubsystemBase {

    public static DcMotorEx mT;
    //touch sensor near the bottom of the lift slides used to localize the lift encoder
//and prevent the lift from driving into the deck plate
    public static Boolean tracking = true;
    public PIDController controller;
    //PID explanation. The PIDController is a neat thing that drives a motor to a position and holds it there
    //You will use PIDObject.calculate, which takes the encoder value of the motor and the desired value (position)
    //and then spits out a motor power to drive the motor to the desired position. If its in a continual loop,
    //it will continuously change this value based on its tuned P I and D variables

    public static double targetPosition;// stores the desired position of the lift in motor ticks
    public double motorPower;//stores the final desired motor power, which is then fed into the motors
    boolean tooFar;

    public static boolean PIDDisabled = false;


    //hardwaremap virtual components to configuration
    public Turret(HardwareMap hardwareMap) {

        mT = hardwareMap.get(DcMotorEx.class, "yaw");


        //mLT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//at the start of teleop reset the encoder value to 0 (localize it)

        mT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//TODO We need to check motor encoder to see if right goes in positive direction.
        //IMPORTANT NOTES ON TUNING PID. The P value can be seen as how fast the lift moves to reach a position. Start very
        //small and slowly increase it until the lift slightly oscillates around the desired location. Then, increase the D value,
        //also starting very small and slowly increasing. The D value determines how much the lift slows down as it approaches the desired value.
        //The I value is for accuracy. It takes how far off you are from your desired location and adds power to close that gap.
        //Its tuned in the same way as the other two values and should be done last after D.
        //This video is a good explanation:
        //https://www.youtube.com/watch?time_continue=7&v=XfAt6hNV8XM&embeds_referring_euri=https%3A%2F%2Fcdn.iframe.ly%2F&source_ve_path=MTM5MTE3LDEzOTExNywyODY2Ng
        controller = new PIDController(TURRET_P, TURRET_I, TURRET_D);
    }


    public void periodic() {
        // runs every loop
if (PIDDisabled == false){
    motorPower = getCurrentPID();
} else {
    motorPower = 0;
}


        mT.setPower(motorPower);

        //}

    }

    //these are a few telemetry methods for trouble shooting
    public static double getCurrentPosition() {
        return mT.getCurrentPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getCurrentPID() {// this is the method that takes the current position and desired position and returns a motor power
        return controller.calculate(mT.getCurrentPosition(), targetPosition);
    }

    public static void setTargetPosition(double newTargetPosition) {// updates the target position to whatever its set as by the LiftToStateCommand
        targetPosition = newTargetPosition;
    }


    public double getCurrentMotorPower() {
        return motorPower;
    }


    //this was for auto with so that we could do a stage one hang by letting the motors sag down to the lower bar.
    //it never worked and I still don't know why

}