package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;


import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_RADIANS_PER_TICK;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.currentArtifacts;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.motif;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Storage;
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
    private Turret turret;
    private Storage storage;

    // --- Cycle selection ---
    public static int gateCycleIndex = 0;
    public static final int MAX_CYCLES = 5;
    private int cycleCount = 2;   // default 2 cycles
    private int currentCycle = 0;  // row selector
    private int currentColumn = 0; // column selector: 0=shoot, 1=spike

    private boolean dpadUpPressed, dpadDownPressed, dpadLeftPressed, dpadRightPressed, bumperPressed;

    // choices[cycleIndex][0=shootChoice(0 goal,1 audience), 1=spikeChoice(0 goal,1 mid,2 audience)]
    private int[][] choices = new int[MAX_CYCLES][2];

    private boolean gateCyclePressed;

    public Pose2d startPos;

    private Camera camera;
    private Intake intake;

    private String aColor = null;

    public static Pose2d savedPos;

    private Command auto;


    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry2 = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        CommandScheduler.getInstance().reset();
        rrSubsystem = new RRSubsystem(hardwareMap);
        turret = new Turret(hardwareMap);
        // camera = new Camera(hardwareMap);
        intake = new Intake(hardwareMap);
        storage = new Storage(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(rrSubsystem);
        turret.mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.mT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


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
            if (gamepad2.dpad_up) startPos = AutoTrajectories.goalStartPos;
            else if (gamepad2.dpad_down) startPos = AutoTrajectories.audienceStartPos;
        }

        // --- Handle user input for cycles ---
        handleInput();
//        if (aColor != null){
//            camera.setObeliskMotif();
//        }


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


        if ((gamepad2.right_bumper || gamepad2.left_bumper) && !gateCyclePressed) {

            if (gamepad2.right_bumper) {
                gateCycleIndex = (gateCycleIndex + 1) % (cycleCount + 1);
            }

            if (gamepad2.left_bumper) {
                gateCycleIndex = (gateCycleIndex - 1 + (cycleCount + 1)) % (cycleCount + 1);
            }

            gateCyclePressed = true;

        } else if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
            gateCyclePressed = false;
        }


        // Select choices
        if (gamepad1.a) choices[currentCycle][currentColumn] = 0; // Shoot goal / Spike goal
        if (gamepad1.b) choices[currentCycle][currentColumn] = 1; // Shoot audience / Spike mid
        if (gamepad1.y && currentColumn == 1)
            choices[currentCycle][currentColumn] = 2; // Spike audience
    }

    private void printTelemetryTable() {
        telemetry2.addData("Turret Heading(DEG)", Math.toDegrees(turret.getCurrentPosition() * TURRET_RADIANS_PER_TICK));
        if (motif != null) {
            telemetry2.addData("Motif", motif);
        }


        String gateTxt = (gateCycleIndex == cycleCount)
                ? "After ALL cycles"
                : "After cycle " + (gateCycleIndex + 1);
        telemetry2.addData("Gate Action", gateTxt);
//

        // --- Alliance + Start position ---
        String startName = "Not chosen";
        if (startPos != null) {
            if (startPos.equals(AutoTrajectories.goalStartPos)) startName = "Goal Start";
            else if (startPos.equals(AutoTrajectories.audienceStartPos))
                startName = "Audience Start";
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

        String[] shootNames = {"Goal", "Audience"};
        String[] spikeNames = {"Goal", "Mid", "Audience"};

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
        auto = AutoGenerator.buildAuto(requirements, cycleCount, intake, storage);
        CommandScheduler.getInstance().schedule(
                auto
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        if (runtime.seconds() > 29.5 && auto != null) {
            CommandScheduler.getInstance().cancel(auto);
            auto = null;
        }


        if (drive != null) drive.updatePoseEstimate();
        savedPos = drive.localizer.getPose();
        telemetry2.addData("artifactLocation", currentArtifacts);
        telemetry2.addData("Turret Heading(DEG)", Math.toDegrees(turret.getTargetPosition() * TURRET_RADIANS_PER_TICK));

    }
}
