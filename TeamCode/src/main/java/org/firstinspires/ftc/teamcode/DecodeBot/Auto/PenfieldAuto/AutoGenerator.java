package org.firstinspires.ftc.teamcode.DecodeBot.Auto.PenfieldAuto;


import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.PenfieldAuto.AutoTrajectories.gateExit;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.PenfieldAuto.AutoTrajectories.gateRelease;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.PenfieldAuto.AutoTrajectories.intakeToShoot;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.PenfieldAuto.AutoTrajectories.startToIntake;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.PenfieldAuto.DecodeAuto.gateCycleIndex;
//import static org.firstinspires.ftc.teamcode.DecodeBot.Commands.AutoLaunchCommands.MotifLaunchSequenceCommand;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.DecodeBot.Commands.AutoLaunchCommands;
import org.firstinspires.ftc.teamcode.DecodeBot.Commands.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Storage;

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
    public static SequentialCommandGroup buildAuto(Set<Subsystem> requirements, int cycleCount, Intake intake, Storage storage) {
        List<Command> seq = new ArrayList<>();

        seq.add(new LaunchSequenceCommand(intake, storage, "Fly"));

        for (int i = 0; i < cycleCount; i++) {
//


            if (startToIntake[i] != null) {
                seq.add(new InstantCommand(intake::in));
                seq.add(new ActionCommand(startToIntake[i], requirements));
                seq.add(new InstantCommand(intake::stop));

            }

            if (intakeToShoot[i] != null) {
                seq.add(new ActionCommand(intakeToShoot[i], requirements));
            }
//            else if (!gateReleased) {
//                seq.add(new LaunchSequenceCommand(intake, storage, "Fly"));
//            } else {
//                seq.add(new AutoLaunchCommands(intake, storage));
//            }

            if (i == gateCycleIndex) {
                seq.add(new ActionCommand(gateRelease, requirements));
            }
            if (i == gateCycleIndex) {
                seq.add(new ActionCommand(gateExit, requirements));
                gateReleased = true;
            }
        }

        return new SequentialCommandGroup(seq.toArray(new Command[0]));
    }
}
