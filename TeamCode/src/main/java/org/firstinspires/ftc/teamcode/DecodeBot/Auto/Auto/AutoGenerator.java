package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;

import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.bigStartPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.smallStartPos;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.RRSubsystem;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AutoGenerator {

    public static SequentialCommandGroup buildAuto(int choice1, int choice2, int choice3,
                                                   RRSubsystem rrSubsystem, Pose2d startPos) {
        return new SequentialCommandGroup(
                // --- Set 1 choice ---
                branch1(choice1, startPos, rrSubsystem),
                // --- Set 2 choice ---
                branch2(choice2, rrSubsystem),
                // --- Set 3 choice ---
                branch3(choice3, rrSubsystem)
        );
    }

    private static SequentialCommandGroup branch1(int choice, Pose2d startPos, RRSubsystem rr) {
        if (startPos == bigStartPos){
            switch (choice) {
                case 0:
                    return new SequentialCommandGroup(
                            /*Commands*/
                    );
                case 1:
                    return new SequentialCommandGroup(
                            /*Commands*/
                    );
                case 2:
                    return new SequentialCommandGroup(
                            /*Commands*/
                    );
                default:
                    return new SequentialCommandGroup(); // empty
            }
        }
        if (startPos == smallStartPos){
            switch (choice) {
                case 0:
                    return new SequentialCommandGroup(
                            /*Commands*/
                    );
                case 1:
                    return new SequentialCommandGroup(
                            /*Commands*/
                    );
                case 2:
                    return new SequentialCommandGroup(
                            /*Commands*/
                    );
                default:
                    return new SequentialCommandGroup(); // empty
            }
        } else return new SequentialCommandGroup();
    }

    private static SequentialCommandGroup branch2(int choice, RRSubsystem rr) {
        switch (choice) {
            case 0:
                return new SequentialCommandGroup(
                        /*Commands*/
                );
            case 1:
                return new SequentialCommandGroup(
                        /*Commands*/
                );
            case 2:
                return new SequentialCommandGroup(
                        /*Commands*/
                );
            default:
                return new SequentialCommandGroup();
        }
    }

    private static SequentialCommandGroup branch3(int choice, RRSubsystem rr) {
        switch (choice) {
            case 0:
                return new SequentialCommandGroup(
                        /*Commands*/
                );
            case 1:
                return new SequentialCommandGroup(
                        /*Commands*/
                );
            case 2:
                return new SequentialCommandGroup(
                        /*Commands*/
                );
            default:
                return new SequentialCommandGroup();
        }
    }
}
