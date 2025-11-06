package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;


import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.backStartPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.frontStartPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.generateTrajectories;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.aColor;
import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Turret;

import java.util.Set;

@Autonomous(name = "Decode Auto")
//@Disabled

public class DecodeAuto extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive drive;
    public static RRSubsystem rrSubsystem;
    private FtcDashboard dashboard;
    private MultipleTelemetry telemetry2;

    // --- Cycle selection ---
    public static final int MAX_CYCLES = 3;
    private int cycleCount = 2;   // default 2 cycles
    private int currentCycle = 0;  // row selector
    private int currentColumn = 0; // column selector: 0=shoot, 1=spike

    private boolean dpadUpPressed, dpadDownPressed, dpadLeftPressed, dpadRightPressed, bumperPressed;

    // choices[cycleIndex][0=shootChoice(0 front,1 back), 1=spikeChoice(0 front,1 mid,2 back)]
    private int[][] choices = new int[MAX_CYCLES][2];

    public static Pose2d startPos;
    private String aColor = null;



    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry2 = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        CommandScheduler.getInstance().reset();
        rrSubsystem = new RRSubsystem(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(rrSubsystem);

        telemetry2.addData("Status", "Initialized");
        telemetry2.update();
    }

    @Override
    public void init_loop() {
        // --- Alliance selection ---
        if (gamepad2.a) {
            aColor = "blue";
            GlobalVariables.aColor = "blue";
            AutoTrajectories.updateAlliancePoses();
        } else if (gamepad2.b) {
            aColor = "red";
            GlobalVariables.aColor = "red";
            AutoTrajectories.updateAlliancePoses();
        }

        // --- Start position selection ---
        if (aColor != null) {
            if (gamepad2.dpad_up) startPos = AutoTrajectories.frontStartPos;
            else if (gamepad2.dpad_down) startPos = AutoTrajectories.backStartPos;
        }

        // --- Initialize drive for trajectory preview ---
        if (startPos != null && drive == null && aColor != null) {
            drive = new MecanumDrive(hardwareMap, startPos);
            AutoTrajectories.generateTrajectories(drive, choices, cycleCount, startPos);
        }

        // --- Handle user input for cycles ---
        handleInput();

        // --- Display telemetry table and alliance/start ---
        printTelemetryTable();

        // --- Dashboard visualization (optional) ---
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();
        if (startPos != null) {
            field.setStroke("#00FF00");
            field.strokeCircle(startPos.position.x, startPos.position.y, 8);
            double headingLength = 8;
            double x = startPos.position.x, y = startPos.position.y, heading = startPos.heading.toDouble();
            field.strokeLine(x, y, x + headingLength * Math.cos(heading), y + headingLength * Math.sin(heading));
        }
        dashboard.sendTelemetryPacket(packet);
    }

    private void handleInput() {
        // Navigate cycles (up/down)
        if (gamepad1.dpad_up && !dpadUpPressed) {
            currentCycle = (currentCycle - 1 + cycleCount) % cycleCount;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) dpadUpPressed = false;

        if (gamepad1.dpad_down && !dpadDownPressed) {
            currentCycle = (currentCycle + 1) % cycleCount;
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) dpadDownPressed = false;

        // Navigate columns (left/right)
        if (gamepad1.dpad_left && !dpadLeftPressed) {
            currentColumn = (currentColumn - 1 + 2) % 2;
            dpadLeftPressed = true;
        } else if (!gamepad1.dpad_left) dpadLeftPressed = false;

        if (gamepad1.dpad_right && !dpadRightPressed) {
            currentColumn = (currentColumn + 1) % 2;
            dpadRightPressed = true;
        } else if (!gamepad1.dpad_right) dpadRightPressed = false;

        // Adjust cycle count with bumpers
        if ((gamepad1.left_bumper || gamepad1.right_bumper) && !bumperPressed) {
            if (gamepad1.right_bumper && cycleCount < MAX_CYCLES) cycleCount++;
            if (gamepad1.left_bumper && cycleCount > 1) cycleCount--;
            currentCycle = Math.min(currentCycle, cycleCount - 1);
            bumperPressed = true;
        } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) bumperPressed = false;

        // Select choices
        if (gamepad1.a) choices[currentCycle][currentColumn] = 0; // Shoot front / Spike front
        if (gamepad1.b) choices[currentCycle][currentColumn] = 1; // Shoot back / Spike mid
        if (gamepad1.y && currentColumn == 1) choices[currentCycle][currentColumn] = 2; // Spike back
    }

    private void printTelemetryTable() {
        telemetry2.clearAll();

        // --- Alliance + Start position ---
        String startName = "Not chosen";
        if (startPos != null) {
            if (startPos.equals(AutoTrajectories.frontStartPos)) startName = "Front Start";
            else if (startPos.equals(AutoTrajectories.backStartPos)) startName = "Back Start";
        }
        String allianceDisplay = (aColor != null) ? aColor : "None";
        telemetry2.addData("Alliance Start", allianceDisplay + " - " + startName);

        telemetry2.addLine("=== Cycle Selection ===");
        telemetry2.addData("Cycle Count", cycleCount);
        telemetry2.addLine("Use D-Pad to navigate, bumpers to change cycle count");
        telemetry2.addLine("");

        // --- Table header ---
        telemetry2.addLine("Cycle | Shoot  | Spike");
        telemetry2.addLine("-------------------------");

        String[] shootNames = {"Front", "Back"};
        String[] spikeNames = {"Front", "Mid", "Back"};

        for (int i = 0; i < cycleCount; i++) {
            String shoot = shootNames[choices[i][0]];
            String spike = spikeNames[choices[i][1]];

            // Single marker for the selected cell
            String shootCell = (i == currentCycle && currentColumn == 0) ? "*" + shoot : " " + shoot;
            String spikeCell = (i == currentCycle && currentColumn == 1) ? "*" + spike : " " + spike;

            telemetry2.addLine(String.format(" %d    | %s   | %s", i + 1, shootCell, spikeCell));
        }

        telemetry2.update();
    }

    @Override
    public void start() {
        runtime.reset();
        if (drive == null) drive = new MecanumDrive(hardwareMap, startPos);
        AutoTrajectories.generateTrajectories(drive, choices, cycleCount, startPos);

        Set<Subsystem> requirements = Set.of(rrSubsystem);
        CommandScheduler.getInstance().schedule(
                AutoGenerator.buildAuto(requirements, cycleCount)
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        if (drive != null) drive.updatePoseEstimate();
        startPos = drive.localizer.getPose();
    }
}
