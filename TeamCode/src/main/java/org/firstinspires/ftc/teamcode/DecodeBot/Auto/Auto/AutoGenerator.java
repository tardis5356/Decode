package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;


import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.gateAction;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.spikeToShoot;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.AutoTrajectories.startToSpike;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.DecodeBot.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class AutoGenerator {

    /**
     * Build a sequential auto that:
     * for each cycle i:
     *   - runs startToSpikeActions[i]
     *   - runs spikeToShootActions[i]
     * after all cycles: runs gateAction
     */
    public static SequentialCommandGroup buildAuto(Set<Subsystem> requirements, int cycleCount) {
        List<ActionCommand> seq = new ArrayList<>();

        for (int i = 0; i < cycleCount; i++) {
            // null-checks defensive: if generateTrajectories wasn't called, these might be null.
            if (startToSpike[i] != null) {
                seq.add(new ActionCommand(startToSpike[i], requirements));
            }
            if (spikeToShoot[i] != null) {
                seq.add(new ActionCommand(spikeToShoot[i], requirements));
            }
        }

        if (gateAction != null) {
            seq.add(new ActionCommand(gateAction, requirements));
        }

        return new SequentialCommandGroup(seq.toArray(new ActionCommand[0]));
    }
}
