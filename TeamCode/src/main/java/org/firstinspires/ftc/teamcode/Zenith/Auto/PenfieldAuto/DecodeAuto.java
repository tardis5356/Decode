package org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto;


import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.AutoTrajectories.allianceValue;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_RADIANS_PER_TICK;
//import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions.TURRET_TICK_TO_RADIAN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions.TURRET_TICKS_PER_DEGREE;
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
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.RRSubsystem;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Zenith.TeleOps.DecodeTeleOp;

import java.util.Set;

@Autonomous(name = "Decode Auto"
        , group = "Autonomous"
        , preselectTeleOp = "Decode Teleop" // TODO: readd preselect
)

//@Disabled

public class DecodeAuto extends OpMode {

    public static final int MAX_CYCLES = 5;
    private static final String HEADER_FORMAT = "%-5s | %-12s | %-12s";
    private static final String ROW_FORMAT = "%-5d | %-12s | %-12s";
    public static RRSubsystem rrSubsystem;
    // --- Cycle selection ---
    public static int gateCycleIndex = 1; //default gate cycle after cycle 2
    public static Pose2d savedPos;
    public static Pose2d startPos;
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive drive;
    private FtcDashboard dashboard;
    private MultipleTelemetry telemetry2;
    private Turret turret;
    private Storage storage;
    private int cycleCount = 2;   //5 // default 2 cycles
    private int currentCycle = 0;  // row selector
    private int currentColumn = 0; // column selector: 0=shoot, 1=intake
    private boolean dpadUpPressed, dpadDownPressed, dpadLeftPressed, dpadRightPressed, bumperPressed;
    // choices[cycleIndex][0=shootChoice(0 goal,1 audience), 1=intakeChoice(0 goal,1 mid,2 audience, 3 LZ preset, 4 LZ random)]
    private int[][] choices = new int[MAX_CYCLES][2];
    private boolean gateCyclePressed;
    // private Camera camera;
//    private MecanumDrive localizer;
    private Intake intake;
    private BellyPan bellyPan;
    //private Camera camera;
    private Shooter shooter;
    private String aColor = null;
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
       // camera = new Camera(hardwareMap);
        intake = new Intake(hardwareMap);
        storage = new Storage(hardwareMap);
        shooter = new Shooter(hardwareMap);


        CommandScheduler.getInstance().registerSubsystem(rrSubsystem);
      //  CommandScheduler.getInstance().registerSubsystem(turret); //TODO: comment out
        turret.mT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.mT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        bellyPan.disEngagePTO(); // be SURE the bellyPan is latched at the start of the auto
        startPos = null;
        turret.setTargetPosition(allianceValue(0));

        telemetry2.addData("Status", "Initialized");
        telemetry2.update();
    }

    @Override
    public void init_loop() {
        CommandScheduler.getInstance().run();

        shooter.spinning = false;
        shooter.targeting = false;


        if (startPos == AutoTrajectories.audienceStartPos) {
            turret.setTargetPosition(allianceValue(-12300));
            //sometimes turn before setting configs
        } else if (startPos == AutoTrajectories.goalStartPos) {
            turret.setTargetPosition(allianceValue(-90 * TURRET_TICKS_PER_DEGREE));

        }
        /* else {
            turret.setTargetPosition(0);
        } */ //TODO: test this

        //  turret.periodic();

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
                cycleCount = 3;
                gateCycleIndex = 1; //default gate cycle after cycle 2
                choices = new int[][]{
                        {1, 2}, //shoot: goal, intake: Goal
                        {0, 1}, //shoot: goal, intake: mid
                        {1, 0}, //shoot: gate ready to push, intake: audience
                        {1, 4}, //shoot: audience, intake: LZ random
                        {1, 3} //shoot: audience, intake: preset pose
                };
            } else if (gamepad2.dpad_down) {
                startPos = AutoTrajectories.audienceStartPos;

                //default config
                // choices[cycleIndex][0=shootChoice(0 goal,1 audience),
                gateCycleIndex = 1; //default gate cycle after cycle 2
                // 1=intakeChoice(0 goal,1 mid,2 audience, 3 LZ preset, 4 LZ random)]
                cycleCount = 3;
                choices = new int[][]{
                        {1, 2}, //shoot: audience, intake: audience
                        {0, 1}, //shoot: goal, intake: mid
                        {1, 0}, //shoot: goal ready to push, intake: Goal
                        {1, 4}, //shoot: audience, intake: LZ random
                        {1, 3} //shoot: audience, intake: LZ random
                };

            }

        }

        // --- Handle user input for cycles ---
        handleInput();


//        if (aColor != null) {
//            camera.setObeliskMotif();
//        }


        // --- Display telemetry table and alliance/start ---
        printEASITelemetry();

        //EASI - Easy Autonomous Selectable Interface
         //


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


        if ((gamepad2.right_bumper || gamepad2.left_bumper) && !(gateCyclePressed)) {

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

    private void printEASITelemetry() {
        telemetry2.addData("Turret Heading(DEG)", Math.toDegrees(turret.getCurrentPosition() * TURRET_RADIANS_PER_TICK));
        if (motif != null) {
            telemetry2.addData("Motif", motif);
        }

        telemetry2.addLine("");
        telemetry2.addLine("=== EASI Selection ===");
        telemetry2.addLine("");

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

        if (aColor == null) {
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


        //telemetry2.addLine(String.format(HEADER_FORMAT, "Cycle", "Shoot", "Intake"));
//        telemetry2.addLine("------------------------------------------");

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

        shooter.spinning = true;
        shooter.targeting = true;

        // TODO: potentially add a command scheduler reset here

//        CommandScheduler.getInstance().cancelAll();
//        CommandScheduler.getInstance().reset();

        runtime.reset();
        if (drive == null) drive = new MecanumDrive(hardwareMap, startPos);
        AutoTrajectories.generateTrajectories(drive, choices, cycleCount, startPos);

        Set<Subsystem> requirements = Set.of(rrSubsystem);
        auto = AutoGenerator.buildAuto(requirements, cycleCount, intake, storage, turret);
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
//        if (runtime.seconds() > 29 && auto != null) {
//            CommandScheduler.getInstance().cancel(auto);
//
//            drive.leftBack.setPower(0);
//            drive.leftFront.setPower(0);
//            drive.rightBack.setPower(0);
//            drive.rightFront.setPower(0);
//
//            auto = null;
//        }

        turret.updateTurretTracking(drive, telemetry2);
        shooter.setTargetDistance(GlobalVariables.distanceFromTarget);


        if (drive != null) drive.updatePoseEstimate();
        savedPos = drive.localizer.getPose();
        telemetry2.addData("artifactLocation", currentArtifacts);
        telemetry2.addData("Turret Heading(DEG)", Math.toDegrees(turret.getTargetPosition() * TURRET_RADIANS_PER_TICK));
        telemetry.addData("flyWheelSpeed", shooter.getFlyWheelSpeed());
        telemetry.addData("targetSpeed", shooter.targetFlyWheelSpeed + shooter.speedOffset);
    }

//    @Override
//    public void stop() {// TODO Test
//        CommandScheduler.getInstance().cancelAll();
//        CommandScheduler.getInstance().reset();
//    }
}
