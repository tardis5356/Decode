package org.firstinspires.ftc.teamcode.Zenith.Auto.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Zenith.Auto.Drawing;
import org.firstinspires.ftc.teamcode.Zenith.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Zenith.Auto.TankDrive;

public class LocalizationTest extends LinearOpMode {

    double lastTime = 0;

    double lastX = 0;
    double lastY = 0;
    double lastHeading = 0;

    double lastLinearVel = 0;
    double lastAngularVel = 0;

    double maxLinearVel = 0;
    double maxAngularVel = 0;

    double maxLinearAccel = 0;
    double minLinearAccel = 0;

    double maxAngularAccel = 0;
    double minAngularAccel = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                if (gamepad1.aWasPressed()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    1.0,
                                    0.0
                            ),
                            0.0
                    ));
                } else if (gamepad1.aWasReleased()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    0.0,
                                    0.0
                            ),
                            0.0
                    ));
                }

                if (gamepad1.bWasPressed()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    0.0,
                                    1.0
                            ),
                            0.0
                    ));
                } else if (gamepad1.bWasReleased()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    0.0,
                                    0.0
                            ),
                            0.0
                    ));
                }

                if (gamepad1.xWasPressed()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    0.0,
                                    0.0
                            ),
                            1.0
                    ));
                } else if (gamepad1.xWasReleased()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    0.0,
                                    0.0
                            ),
                            0.0
                    ));
                }


                drive.updatePoseEstimate();


                Pose2d pose = drive.localizer.getPose();

                double currentTime = getRuntime();
                double dt = currentTime - lastTime;

                if (dt > 0) {
                    // --- Compute linear velocity ---
                    double dx = pose.position.x - lastX;
                    double dy = pose.position.y - lastY;
                    double linearVel = Math.hypot(dx, dy) / dt;

                    // --- Compute angular velocity ---
                    double dHeading = pose.heading.toDouble() - lastHeading;
                    double angularVel = dHeading / dt;

                    // --- Compute accelerations ---
                    double linearAccel = (linearVel - lastLinearVel) / dt;
                    double angularAccel = (angularVel - lastAngularVel) / dt;

                    // --- Track maximums ---
                    maxLinearVel = Math.max(maxLinearVel, linearVel);
                    maxAngularVel = Math.max(maxAngularVel, Math.abs(angularVel));

                    maxLinearAccel = Math.max(maxLinearAccel, linearAccel);
                    minLinearAccel = Math.min(minLinearAccel, linearAccel);

                    maxAngularAccel = Math.max(maxAngularAccel, angularAccel);
                    minAngularAccel = Math.min(minAngularAccel, angularAccel);

                    // --- Telemetry ---
                    telemetry.addData("Linear Vel", linearVel);
                    telemetry.addData("Angular Vel", angularVel);
                    telemetry.addData("Linear Accel", linearAccel);
                    telemetry.addData("Angular Accel", angularAccel);
                    telemetry.addLine();
                    telemetry.addData("Max Linear Vel", maxLinearVel);
                    telemetry.addData("Max Angular Vel", maxAngularVel);
                    telemetry.addLine();
                    telemetry.addData("Max Linear Accel", maxLinearAccel);
                    telemetry.addData("Min Linear Accel", minLinearAccel);
                    telemetry.addLine();
                    telemetry.addData("Max Angular Accel", maxAngularAccel);
                    telemetry.addData("Min Angular Accel", minAngularAccel);
                }


                lastX = pose.position.x;
                lastY = pose.position.y;
                lastHeading = pose.heading.toDouble();

                lastLinearVel = Math.hypot(pose.position.x - lastX, pose.position.y - lastY) / dt;
                lastAngularVel = (pose.heading.toDouble() - lastHeading) / dt;

                lastTime = currentTime;
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
