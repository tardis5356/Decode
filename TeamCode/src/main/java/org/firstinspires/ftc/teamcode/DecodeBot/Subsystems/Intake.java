package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.currentArtifacts;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Storage.slotFly;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    DcMotorEx mI;

    public ColorRangeSensor cSI;
    public ColorRangeSensor cSM;
    public ColorRangeSensor cSSh;
    public ColorRangeSensor cSSt;


    public static String intakeState = new String();

    public boolean currentArtifactsEstablished;

    double intakePower = 0;


    public Intake(HardwareMap hardwareMap) {

        mI = hardwareMap.get(DcMotorEx.class, "mI");


        cSI = hardwareMap.get(ColorRangeSensor.class, "cSI");
        cSM = hardwareMap.get(ColorRangeSensor.class, "cSM");
        cSSh = hardwareMap.get(ColorRangeSensor.class, "cSSh");
        cSSt = hardwareMap.get(ColorRangeSensor.class, "cSSt");


    }

    @Override
    public void periodic() {
        mI.setPower(intakePower);
        //mP.setPower(pathPower);
        if (GlobalVariables.currentArtifacts.substring(1) == "PPG" || GlobalVariables.currentArtifacts.substring(1) == "PGP" || GlobalVariables.currentArtifacts.substring(1) == "GPP") {
            currentArtifactsEstablished = true;
        } else {
            currentArtifactsEstablished = false;
        }

        if (!currentArtifactsEstablished) {
            setCurrentArtifacts();
        }


        long emptySlots = GlobalVariables.currentArtifacts.chars()
                .filter(c -> c == '_')
                .count();

        // If less than 2 empty slots → STOP the intake
        if (emptySlots < 2) {
            stop();
        }


    }

    public void in() {
        intakePower = 1;
        intakeState = "in";
    }


    public void out() {
        intakePower = -1;
        intakeState = "out";
    }

    public void stop() {
        intakePower = 0;
        intakeState = "stop";
    }

    //0 = red, 1 = green, 2 = blue
    public double[] normalizeRGB(double r, double g, double b) {
        double total = r + g + b;


        double red = r / total;
        double green = g / total;
        double blue = b / total;
//0 = red, 1 = green, 2 = blue
        return new double[]{red, green, blue};
    }

    public String greenOrPurple(ColorSensor cs) {
        double[] normalizedRGB = normalizeRGB(cs.red(), cs.green(), cs.blue());

        if (cs == cSSt && slotFly) {
            return "_";
        }
        if (normalizedRGB[0] < 3) {
            return "G";
        } else if (normalizedRGB[0] > 10) {
            return "P";
        } else {
            return "_";
        }
    }

    public void setCurrentArtifacts() {
        currentArtifacts = greenOrPurple(cSSt) + greenOrPurple(cSSh) + greenOrPurple(cSM) + greenOrPurple(cSI);
    }

}