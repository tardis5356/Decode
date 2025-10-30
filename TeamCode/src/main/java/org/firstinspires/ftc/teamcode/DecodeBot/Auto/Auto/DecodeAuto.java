package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;


import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.backSpikeToShoot1;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.backStartPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.frontSpikeIntake;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.frontSpikeToGate;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.frontStartPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.generateTrajectories;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.midSpikeToFrontSpike;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.backStartToBackSpike;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.midSpikeToShoot2;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.shootPos1ToMidSpike;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.shootPos2ToGate;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.aColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.RRSubsystem;

import java.util.Set;

@Autonomous(name = "Decode Auto")
//@Disabled

public class DecodeAuto extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private MecanumDrive drive;
    private RRSubsystem rrSubsystem;
    public static Pose2d startPos;
    int visionOutputPosition = 1;
    private DcMotorEx mFL;
    private DcMotorEx mFR;
    private DcMotorEx mBL;
    private DcMotorEx mBR;

    private FtcDashboard dashboard;
    private MultipleTelemetry telemetry2;

    // Selection system
    private int currentSet = 0; // which set you’re editing (0–2)
    private final int[] choices = {0, 0}; // one choice for each set
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    private static ActionCommand StartToBackSpike;
    private static ActionCommand BackSpikeToShoot1;
    private static ActionCommand Shoot1ToMidSpike;
    private static ActionCommand MidSpikeToShoot2;
    private static ActionCommand Shoot2ToGate;
    private static ActionCommand MidSpikeIntake;
    private static ActionCommand FrontSpikeIntake;
    private static ActionCommand MidSpikeToFrontSpike;
    private static ActionCommand FrontSpikeToGate;

    private final String[][] autoNames = {
            {"Set1-FrontShoot", "Set1-BackShoot"}, //PPG, PGP, GPP
            {"Set2-FrontShoot", "Set2-BackShoot",}
    };


    @Override
    public void init() {
        // ✅ Initialize dashboard & telemetry safely
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
        // ----- Alliance color selection -----
        if (gamepad2.a) aColor = "blue";
        else if (gamepad2.b) aColor = "red";

        // ----- Starting position selection -----
        if (aColor != null){
            if (gamepad2.dpad_up) startPos = frontStartPos;
            else if (gamepad2.dpad_down) startPos = backStartPos;

        }

        // ----- Scroll between sets -----
        if (gamepad1.dpad_up && !dpadUpPressed) {
            currentSet = (currentSet + 1) % 2;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            currentSet = (currentSet - 1 + 2) % 2;
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // ----- Choose A/B -----
        if (gamepad1.a) choices[currentSet] = 0;
        if (gamepad1.b) choices[currentSet] = 1;

        // ----- Initialize drive once startPos chosen -----
        if (startPos != null && drive == null && aColor != null) {
            drive = new MecanumDrive(hardwareMap, startPos);
        }

        // ----- Generate trajectories dynamically -----
        if (drive != null && startPos != null && aColor != null) {
            generateTrajectories(drive, choices[0], choices[1]);
        }

        // ----- Telemetry -----
        telemetry2.addLine("Use D-Pad to switch sets");
        telemetry2.addLine("Pick A=1, B=2 inside the current set");
        for (int i = 0; i < 2; i++) {
            telemetry2.addData("Set " + (i + 1),
                    autoNames[i][choices[i]] + (i == currentSet ? "  <==" : ""));
        }
        telemetry2.addData("startPos", startPos);
        telemetry2.addData("Alliance Color", aColor);
        telemetry2.update();

        // ----- Dashboard visualization (optional) -----
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();
        if (startPos != null) {
            field.setStroke("#00FF00");
            field.strokeCircle(startPos.position.x, startPos.position.y, 5);
        }
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void start() {
        runtime.reset();

        if (drive == null) drive = new MecanumDrive(hardwareMap, startPos);
        generateTrajectories(drive, choices[0], choices[1]);

        Set<Subsystem> requirements = Set.of(rrSubsystem);
        StartToBackSpike = new ActionCommand(backStartToBackSpike, requirements);
        BackSpikeToShoot1 = new ActionCommand(backSpikeToShoot1, requirements);
        Shoot1ToMidSpike = new ActionCommand(shootPos1ToMidSpike, requirements);
        MidSpikeToShoot2 = new ActionCommand(midSpikeToShoot2, requirements);
        Shoot2ToGate = new ActionCommand(shootPos2ToGate, requirements);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        StartToBackSpike
//                        BackSpikeToShoot1,
//                        Shoot1ToMidSpike,
//                        MidSpikeToShoot2,
//                        Shoot2ToGate
                )
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        telemetry2.addData("Heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry2.addData("X", drive.localizer.getPose().position.x);
        telemetry2.addData("Y", drive.localizer.getPose().position.y);
        telemetry2.update();

        drive.updatePoseEstimate();
    }
}
