package org.firstinspires.ftc.teamcode.Zenith.Tests;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_TICKS_PER_DEGREE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Turret;
@Disabled
@TeleOp(name = "Turret kS Calibration (FTCLib)", group = "Calibration")
public class TurretCalibrationTeleop extends OpMode {

    // ==== HARDWARE ====
    private Turret turret;
    private VoltageSensor voltageSensor;

    // ==== CONSTANTS ====
    private static final double MAX_TURRET_ANGLE_DEG = 200;
    private static final int KS_BIN_DEG = 20;


    private static final double POSITION_POWER = 0.2;
    private static final double POWER_STEP = 0.002;
    private static final int MOTION_THRESHOLD_TICKS = 100;
    private static final double MAX_TEST_POWER = 0.3;
    private static final double POSITION_TOLERANCE_DEG = 2.0;
    private static final double TIMEOUT_SEC = 5.0;

    // ==== BINS ====
    private static final int NUM_BINS =
            (int) ((2 * MAX_TURRET_ANGLE_DEG) / KS_BIN_DEG);

    private final double[][] kSLookup = new double[NUM_BINS][2];
    private final boolean[][] measured = new boolean[NUM_BINS][2];

    // ==== STATE ====
    private enum CalState { MOVE_TO_BIN, MEASURE_CCW, MEASURE_CW, NEXT_BIN, DONE }
    private CalState state = CalState.MOVE_TO_BIN;

    private int currentBin = 0;
    private double testPower = 0;
    private int lastTicks = 0;


    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        turret.mT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        telemetry.addLine("AUTO Turret kS Calibration");
        telemetry.update();
    }

    @Override
    public void loop() {

        double currentAngle = turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        double targetAngle = binToAngle(currentBin);

        switch (state) {

            case MOVE_TO_BIN:
                double error = targetAngle - currentAngle;

                if (Math.abs(error) > POSITION_TOLERANCE_DEG) {
                    turret.mT.setPower(Math.signum(error) * POSITION_POWER);
                } else {
                    turret.mT.setPower(0);
                    state = CalState.MEASURE_CCW;
                }
                break;

            case MEASURE_CCW:
                measureKS(0);
                break;

            case MEASURE_CW:
                measureKS(1);
                break;

            case NEXT_BIN:
                currentBin++;
                state = currentBin >= NUM_BINS
                        ? CalState.DONE
                        : CalState.MOVE_TO_BIN;
                break;

            case DONE:
                turret.mT.setPower(0);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Bin", currentBin + " / " + (NUM_BINS - 1));
        telemetry.addData("Bin Center (deg)", binToAngle(currentBin));
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("Test Power", testPower);
        telemetry.addData("Angle (deg)",
                turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE);
        printKSTableTelemetry();

        telemetry.update();
    }

    // ==== kS MEASUREMENT ====
    private void measureKS(int dir) {

        if (!measured[currentBin][dir]) {

            testPower += POWER_STEP * (dir == 0 ? 1 : -1);
            testPower = Range.clip(testPower, -MAX_TEST_POWER, MAX_TEST_POWER);

            turret.mT.setPower(testPower);

            int ticks = (int) turret.getCurrentPosition();
            int delta = Math.abs(ticks - lastTicks);

            if (delta > MOTION_THRESHOLD_TICKS || timer.seconds() > TIMEOUT_SEC) {

                double ks = Math.abs(testPower) *
                        (12.0 / voltageSensor.getVoltage());

                kSLookup[currentBin][dir] = ks;
                measured[currentBin][dir] = true;

                testPower = 0;
                lastTicks = ticks;
                timer.reset();

                state = (dir == 0)
                        ? CalState.MEASURE_CW
                        : CalState.NEXT_BIN;
            }

            lastTicks = ticks;
        }
    }
    private void printKSTableTelemetry() {
        telemetry.addLine("public static final double[][] TURRET_KS = {");

        for (int i = 0; i < NUM_BINS; i++) {
            telemetry.addLine(String.format(
                    "    {%.4f, %.4f}%s // %.0f°",
                    kSLookup[i][0],     // CCW
                    kSLookup[i][1],     // CW
                    (i == NUM_BINS - 1) ? "" : ",",
                    binToAngle(i)
            ));
        }

        telemetry.addLine("};");

    }

    // ==== BIN UTILS ====
    public int angleToBin(double angleDeg) {
        angleDeg = Range.clip(angleDeg, -MAX_TURRET_ANGLE_DEG, MAX_TURRET_ANGLE_DEG);
        return (int) ((angleDeg + MAX_TURRET_ANGLE_DEG) / KS_BIN_DEG);
    }
    public double binToAngle(int bin) {
        return (bin * KS_BIN_DEG) - MAX_TURRET_ANGLE_DEG;
    }
}
