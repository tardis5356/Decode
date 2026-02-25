package org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto;


import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.AutoTrajectories.allianceValue;
import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.AutoTrajectories.gateRelease;
import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.AutoTrajectories.goalStartToGoalShoot;
import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.AutoTrajectories.intakeToShoot;
import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.AutoTrajectories.intakeWaypointToIntake;
import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.AutoTrajectories.startToIntake;
import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.AutoTrajectories.startToIntakeWaypoint;
import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.DecodeAuto.gateCycleIndex;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.aColor;
//import static org.firstinspires.ftc.teamcode.DecodeBot.Commands.AutoLaunchCommands.MotifLaunchSequenceCommand;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Zenith.Commands.IntakeWaitCommand;
import org.firstinspires.ftc.teamcode.Zenith.Commands.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Zenith.TeleOps.DecodeTeleOp;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class AutoGenerator {

    public static boolean gateReleased = false;

    /**
     * Build a sequential auto that:
     * for each cycle i:
     * - runs startToIntakeActions[i]
     * - runs intakeToShootActions[i]
     * after all cycles: runs gateAction
     */
    public static SequentialCommandGroup buildAuto(Set<Subsystem> requirements, int cycleCount, Intake intake, Storage storage, Turret turret, Shooter shooter) {
        List<Command> seq = new ArrayList<>();
        seq.add(new InstantCommand(() -> storage.gateOpen = true));
//seq.add(new InstantCommand(storage::closeGate));
        if (DecodeAuto.startPos == AutoTrajectories.audienceStartPos) {


            seq.add(new WaitCommand(1000));
            seq.add(new LaunchSequenceCommand(intake, storage, "FlyAuto"));
        }

        if (DecodeAuto.startPos == AutoTrajectories.goalStartPos) {

            seq.add(new ActionCommand(goalStartToGoalShoot, requirements));
            seq.add(new LaunchSequenceCommand(intake, storage, "FlyAuto"));
            seq.add(new InstantCommand(() -> turret.manualOffset = (int) Math.round(allianceValue(-200))));//-500 on blue
            // seq.add(new InstantCommand(()-> shooter.hoodOffset += 0.01));

        }


        for (int i = 0; i < cycleCount; i++) {
//            if (startToIntake[i] != null) {
//                seq.add(new InstantCommand(intake::in));
//                seq.add(new ActionCommand(startToIntake[i], requirements));
//
//            }

            if (startToIntakeWaypoint[i] != null) {
                seq.add(new InstantCommand(intake::in));

                seq.add(new ActionCommand(startToIntakeWaypoint[i], requirements));
                seq.add(new ActionCommand(intakeWaypointToIntake[i], requirements));
//                seq.add(new InstantCommand(intake::stop));

            } else {
                seq.add(new InstantCommand(intake::in));
                seq.add(new ActionCommand(startToIntake[i], requirements));
            }


            if (intakeToShoot[i] != null) {
                seq.add(
                        new ParallelCommandGroup(
                                new ActionCommand(intakeToShoot[i], requirements),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(intake::stop)
                                )
                        )
                );
            }


            seq.add(new LaunchSequenceCommand(intake, storage, "FlyAuto"));

        }

        return new SequentialCommandGroup(seq.toArray(new Command[0]));
    }
}
