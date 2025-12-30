package org.firstinspires.ftc.teamcode.Zenith.Tests;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_TICKS_PER_DEGREE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret kS Calibration", group = "Calibration")
public class TurretCalibrationTeleop extends OpMode {

    // ==== HARDWARE ====
    public static DcMotorEx mT;
    private VoltageSensor voltageSensor;
    private boolean measurementStarted = false;
    // ==== CONSTANTS ====
    private static final double MAX_TURRET_ANGLE_DEG = 200;
    private static final int KS_BIN_DEG = 10;


    private static final double POSITION_POWER = 0.40;
    private static final double POWER_STEP = 0.002;
    private static final int MOTION_THRESHOLD_TICKS = 350;
    private static final double MAX_TEST_POWER = 0.3;
    private static final double POSITION_TOLERANCE_DEG = 2.0;
    private static final double TIMEOUT_SEC = 5.0;

    // ==== BINS ====
    private static final int NUM_BINS =
            (int) ((2 * MAX_TURRET_ANGLE_DEG) / KS_BIN_DEG) + 1;

    private final double[][] kSLookup = new double[NUM_BINS][2];
    private final boolean[][] measured = new boolean[NUM_BINS][2];

    // ==== STATE ====
    private enum CalibrateState { MOVE_TO_BIN, MEASURE_CCW, MEASURE_CW, NEXT_BIN, DONE }
    private CalibrateState state = CalibrateState.MOVE_TO_BIN;

    private int currentBin = 0;
    private double testPower = 0;
    private int lastTicks = 0;


    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
      mT = hardwareMap.get(DcMotorEx.class, "mT");

        mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      mT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mT.setDirection(DcMotorSimple.Direction.REVERSE);
        mT.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        telemetry.addLine("AUTO Turret kS Calibration");
        telemetry.update();
    }

    @Override
    public void loop() {

        double currentAngle = mT.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
        double targetAngle = binToAngle(currentBin);

        switch (state) {

            case MOVE_TO_BIN:
                measurementStarted = false;
                testPower = 0;

                double error = targetAngle - currentAngle;

                if (Math.abs(error) > POSITION_TOLERANCE_DEG) {
                  mT.setPower(-Math.signum(error) * POSITION_POWER);
                  telemetry.addLine("Moving to Bin");
                } else {
                 mT.setPower(0);
                    state = CalibrateState.MEASURE_CCW;
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
                        ? CalibrateState.DONE
                        : CalibrateState.MOVE_TO_BIN;
                break;

            case DONE:
              mT.setPower(0);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Bin", currentBin + " / " + (NUM_BINS - 1));
        telemetry.addData("Bin Center (deg)", binToAngle(currentBin));
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("Current Angle (deg)", currentAngle);
        telemetry.addData("Power", mT.getPower());
        telemetry.addData("error", targetAngle - currentAngle);
        telemetry.addData("Angle (deg)",
                mT.getCurrentPosition() / TURRET_TICKS_PER_DEGREE);
        printKSTableTelemetry();

        telemetry.update();
    }

    // ==== kS MEASUREMENT ====
    private void measureKS(int dir) {

        if (!measurementStarted) {
            lastTicks =  mT.getCurrentPosition();
            testPower = 0;
            timer.reset();
            measurementStarted = true;
        }

        testPower += POWER_STEP * (dir == 0 ? 1 : -1);
        testPower = Range.clip(testPower, -MAX_TEST_POWER, MAX_TEST_POWER);

        mT.setPower(testPower);

        int currentTicks = mT.getCurrentPosition();
        int delta = Math.abs(currentTicks - lastTicks);


        if (delta > MOTION_THRESHOLD_TICKS || timer.seconds() > TIMEOUT_SEC) {

            double ks = Math.abs(testPower) *
                    (12.0 / voltageSensor.getVoltage());

            kSLookup[currentBin][dir] = ks;
            measured[currentBin][dir] = true;

            mT.setPower(0);
            measurementStarted = false;

            state = (dir == 0)
                    ? CalibrateState.MEASURE_CW
                    : CalibrateState.NEXT_BIN;

        }
    }
    private void printKSTableTelemetry() {
        telemetry.addLine("public static final double[][] TurretAngle_kSMatrix = {");

        for (int i = 0; i < NUM_BINS; i++) {
            telemetry.addLine(String.format(
                    "    {%.2f, %.4f, %.4f}%s",
                    binToAngle(i),      // Angle
                    kSLookup[i][0],     // CCW
                    kSLookup[i][1],     // CW
                    (i == NUM_BINS - 1) ? "" : ","
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
