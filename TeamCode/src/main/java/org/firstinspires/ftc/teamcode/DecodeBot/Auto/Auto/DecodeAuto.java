package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;


import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.bigStartPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.generateTrajectories;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.smallStartPos;

import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.aColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.RRSubsystem;

import java.util.Set;

@Autonomous(name = "Decode Auto")
//@Disabled

public class DecodeAuto extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive drive;
    private RRSubsystem rrSubsystem;
    public static Pose2d startPos;

    // Selection system
    private int currentSet = 0; // which set you’re editing (0–2)
    private final int[] choices = {0, 0, 0}; // one choice for each set
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    private final String[][] autoNames = {
            {"Set1-Mark1", "Set1-Mark2", "Set1-Mark3"},
            {"Set2-A", "Set2-B", "Set2-C"},
            {"Set3-A", "Set3-B", "Set3-C"}
    };

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        rrSubsystem = new RRSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

        if (gamepad2.a) {
            aColor = "blue";
        } else if (gamepad2.b) {
            aColor = "red";
        }

        if (gamepad2.dpad_up) {
           startPos = bigStartPos;
        } else if (gamepad2.dpad_down) {
            startPos = smallStartPos;
        }



        // Scroll between sets
        if (gamepad1.dpad_up && !dpadUpPressed) {
            currentSet++;
            if (currentSet > 2) currentSet = 0;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            currentSet--;
            if (currentSet < 0) currentSet = 2;
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // Pick with A/B/X
        if (gamepad1.a) choices[currentSet] = 0;
        if (gamepad1.b) choices[currentSet] = 1;
        if (gamepad1.x) choices[currentSet] = 2;

        // Telemetry
        telemetry.addLine("Use D-Pad Up/Down to switch sets");
        telemetry.addLine("Pick A=1, B=2, X=3 inside the current set");
        for (int i = 0; i < 3; i++) {
            telemetry.addData("Set " + (i + 1),
                    autoNames[i][choices[i]] + (i == currentSet ? "  <==" : ""));
        }
        telemetry.update();
    }

    @Override
    public void start() {

        drive = new MecanumDrive(hardwareMap, startPos);

        generateTrajectories(drive, choices[0]);

        runtime.reset();

        // Build one combined sequential auto
        SequentialCommandGroup auto = AutoGenerator.buildAuto(
                choices[0], choices[1], choices[2],
                rrSubsystem, startPos
        );

        CommandScheduler.getInstance().schedule(auto);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        telemetry.addData("Heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.addData("X", drive.localizer.getPose().position.x);
        telemetry.addData("Y", drive.localizer.getPose().position.y);
        telemetry.update();

        drive.updatePoseEstimate();
    }
}
