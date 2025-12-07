package org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto;


import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_RADIANS_PER_TICK;
//import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.currentArtifacts;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.motif;

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

import org.firstinspires.ftc.teamcode.Zenith.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BellyPan;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Zenith.TeleOps.DecodeTeleOp;

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
    public static int gateCycleIndex = 1; //default gate cycle after cycle 2
    public static final int MAX_CYCLES = 5;
    private int cycleCount = 3;   //5 // default 5 cycles
    private int currentCycle = 0;  // row selector
    private int currentColumn = 0; // column selector: 0=shoot, 1=intake

    private boolean dpadUpPressed, dpadDownPressed, dpadLeftPressed, dpadRightPressed, bumperPressed;

    // choices[cycleIndex][0=shootChoice(0 goal,1 audience), 1=intakeChoice(0 goal,1 mid,2 audience, 3 LZ preset, 4 LZ random)]
    private int[][] choices = new int[MAX_CYCLES][2];

    private boolean gateCyclePressed;

    public Pose2d startPos;

    // private Camera camera;
//    private MecanumDrive localizer;
    private Intake intake;
    private BellyPan bellyPan;

    private Camera camera;

    private Shooter shooter;
    private String aColor = null;

    public static Pose2d savedPos;

    private Command auto;


    @Override
    public void init() {
        GlobalVariables.inAuto = true;
        dashboard = FtcDashboard.getInstance();
        telemetry2 = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        CommandScheduler.getInstance().reset();
        rrSubsystem = new RRSubsystem(hardwareMap);
        turret = new Turret(hardwareMap);
        bellyPan = new BellyPan(hardwareMap);
         camera = new Camera(hardwareMap);
        intake = new Intake(hardwareMap);
        storage = new Storage(hardwareMap);
        shooter = new Shooter(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(rrSubsystem);
        turret.mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.mT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        bellyPan.disEngagePTO(); // be SURE the bellyPan is latched at the start of the auto
       // turret.updateTurretTracking(drive, telemetry);

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
            if (gamepad2.dpad_up) {
                startPos = AutoTrajectories.goalStartPos;

                //default config
                // choices[cycleIndex][0=shootChoice(0 goal,1 audience),
                // 1=intakeChoice(0 goal,1 mid,2 audience, 3 LZ preset, 4 LZ random)]
                cycleCount = 5;
                gateCycleIndex = 1; //default gate cycle after cycle 2
                choices = new int[][]{
                        {1, 2}, //shoot: audience, intake: audience
                        {0, 0}, //shoot: goal, intake: goal
                        //gate
                        {0, 1}, //shoot: goal, intake: mid
                        {1, 4}, //shoot: audience, intake: LZ random
                        {1, 4} //shoot: audience, intake: LZ random
                };
            } else if (gamepad2.dpad_down) {
                startPos = AutoTrajectories.audienceStartPos;

                //default config
                // choices[cycleIndex][0=shootChoice(0 goal,1 audience),
                gateCycleIndex = 1; //default gate cycle after cycle 2
                // 1=intakeChoice(0 goal,1 mid,2 audience, 3 LZ preset, 4 LZ random)]
                cycleCount = 5;
                choices = new int[][]{
                        {1, 3}, //shoot: audience, intake: LZ preset
                        {0, 0}, //shoot: goal, intake: goal
                        //gate
                        {0, 1}, //shoot: goal, intake: mid
                        {0, 2}, //shoot: audience, intake: audience
                        {1, 4} //shoot: audience, intake: LZ random
                };

            }
        }

        // --- Handle user input for cycles ---
        handleInput();
        if (aColor != null){
            camera.setObeliskMotif();
        }


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
        if (gamepad1.a) choices[currentCycle][currentColumn] = 0; // Shoot goal / intake goal
        if (gamepad1.b) choices[currentCycle][currentColumn] = 1; // Shoot audience / intake mid
        if (gamepad1.y && currentColumn == 1)
            choices[currentCycle][currentColumn] = 2; // intake audience
        if (gamepad1.right_stick_button && currentColumn == 1)
            choices[currentCycle][currentColumn] = 3; // intake LZ preset
        if (gamepad1.left_stick_button && currentColumn == 1)
            choices[currentCycle][currentColumn] = 4; // intake LZ random
    }
    private static final String HEADER_FORMAT = "%-5s | %-12s | %-12s";
    private static final String ROW_FORMAT    = "%-5d | %-12s | %-12s";
    private void printTelemetryTable() {
        telemetry2.addData("Turret Heading(DEG)", Math.toDegrees(turret.getCurrentPosition() * TURRET_RADIANS_PER_TICK));
        if (motif != null) {
            telemetry2.addData("Motif", motif);
        }


        String gateTxt = (gateCycleIndex == cycleCount)
                ? "No Gate Cycle"
                : "After cycle " + (gateCycleIndex + 1);
        telemetry2.addLine("Gamepad 2⬇️");
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
        telemetry2.addLine("=== Alliance Selection ===");

        if (aColor == null){
            telemetry2.addLine("️✖️ - Blue, ⭕ - Red");
        } else {
            telemetry2.addLine("Up - Goal Start, Down - Audience Start");
        }
        telemetry2.addData("Alliance Start", allianceDisplay + " - " + startName);

        telemetry2.addLine("=== Cycle Selection ===");
        telemetry2.addLine("Gamepad 1⬇️");
        telemetry2.addData("Cycle Count", cycleCount);


        telemetry2.addLine("Use D-Pad to navigate, bumpers to change cycle count");

        telemetry2.addLine("");
        if (currentColumn == 0) {
            telemetry2.addLine("✖️ - Goal, ⭕ - Audience");
        } else if (currentColumn == 1) {
            telemetry2.addLine("✖️ - Goal, ⭕ - Mid,    ⃤ ️ - Audience");
            telemetry2.addLine("Right Stick - LZ Preset, Left Stick - LZ Random");
        }
        telemetry2.addLine("");
        // --- Table header ---
        telemetry2.addLine("Cycle | Shoot  | Intake");
        telemetry2.addLine("-------------------------");

        String[] shootNames = {"Goal", "Audience"};
        String[] intakeNames = {"Goal", "Mid", "Audience", "LZ Preset", "LZ Random"};


        telemetry2.addLine(String.format(HEADER_FORMAT, "Cycle", "Shoot", "Intake"));
        telemetry2.addLine("------------------------------------------");

        for (int i = 0; i < cycleCount; i++) {
            String shoot = shootNames[choices[i][0]];
            String intake = intakeNames[choices[i][1]];

            if (i == currentCycle && currentColumn == 0) shoot = "*" + shoot;
            if (i == currentCycle && currentColumn == 1) intake = "*" + intake;

            telemetry2.addLine(String.format(ROW_FORMAT, i + 1, shoot, intake));
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
intake.setCurrentArtifacts();

        if (GlobalVariables.currentArtifacts.substring(1) == GlobalVariables.motif) {
            DecodeTeleOp.currentShootMode = DecodeTeleOp.shootModes.FLY;
        } else if ((GlobalVariables.currentArtifacts.substring(1) == "PPG" && GlobalVariables.motif == "PGP") || (GlobalVariables.currentArtifacts.substring(1) == "PGP" && GlobalVariables.motif == "PPG")) {
            DecodeTeleOp.currentShootMode = DecodeTeleOp.shootModes.STORE_MIDDLE;
        } else if ((GlobalVariables.currentArtifacts.substring(1) == "PGP" && GlobalVariables.motif == "GPP") || (GlobalVariables.currentArtifacts.substring(1) == "GPP" && GlobalVariables.motif == "PPG")) {
            DecodeTeleOp.currentShootMode = DecodeTeleOp.shootModes.STORE_ONE_FOR_LAST;
        } else if ((GlobalVariables.currentArtifacts.substring(1) == "GPP" && GlobalVariables.motif == "PGP")) {
            DecodeTeleOp.currentShootMode = DecodeTeleOp.shootModes.STORE_ONE_FOR_SECOND;
        } else {
            DecodeTeleOp.currentShootMode = DecodeTeleOp.shootModes.FLY;
        }

        CommandScheduler.getInstance().run();


        turret.updateTurretTracking(drive, telemetry2);


        if (drive != null) drive.updatePoseEstimate();
        savedPos = drive.localizer.getPose();
        telemetry2.addData("artifactLocation", currentArtifacts);
        telemetry2.addData("Turret Heading(DEG)", Math.toDegrees(turret.getTargetPosition() * TURRET_RADIANS_PER_TICK));

    }
}
