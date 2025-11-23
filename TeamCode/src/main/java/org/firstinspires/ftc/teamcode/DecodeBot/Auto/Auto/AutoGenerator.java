package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;


import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.gateExit;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.gateRelease;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.spikeToShoot;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.startToSpike;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.DecodeAuto.gateCycleIndex;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.DecodeBot.Commands.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.DecodeBot.Commands.MotifLaunchSequenceCommand;
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
     * - runs startToSpikeActions[i]
     * - runs spikeToShootActions[i]
     * after all cycles: runs gateAction
     */
    public static SequentialCommandGroup buildAuto(Set<Subsystem> requirements, int cycleCount, Intake intake, Storage storage) {
        List<Command> seq = new ArrayList<>();

        for (int i = 0; i < cycleCount; i++) {
//


            if (startToSpike[i] != null) {
                seq.add(new InstantCommand(intake::in));
                seq.add(new ActionCommand(startToSpike[i], requirements));
                seq.add(new InstantCommand(intake::stop));

            }

            if (spikeToShoot[i] != null) {
                seq.add(new ActionCommand(spikeToShoot[i], requirements));
            }
            else {
                seq.add(new LaunchSequenceCommand(intake, storage, "Fly"));
            }
//            else {
//                seq.add(new MotifLaunchSequenceCommand(intake, storage));
//            }

            if (i == gateCycleIndex) {
                seq.add(new ActionCommand(gateRelease, requirements));
            }
            if(i == gateCycleIndex) {
                seq.add(new ActionCommand(gateExit, requirements));
                gateReleased = true;
            }
        }

        return new SequentialCommandGroup(seq.toArray(new Command[0]));
    }
}
