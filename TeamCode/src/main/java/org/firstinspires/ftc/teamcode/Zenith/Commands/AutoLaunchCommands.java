package org.firstinspires.ftc.teamcode.Zenith.Commands;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.currentArtifacts;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.motif;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Zenith.TeleOps.DecodeTeleOp;

public class AutoLaunchCommands extends SequentialCommandGroup {


    public AutoLaunchCommands(Intake intake, Storage storage) {
        switch (DecodeTeleOp.currentShootMode) {
            case FLY:
                new LaunchSequenceCommand(intake, storage, "Fly");
                break;
            case STORE_MIDDLE:
                new LaunchSequenceCommand(intake, storage, "Store Middle");
                break;
            case STORE_ONE_FOR_LAST:
                new LaunchSequenceCommand(intake, storage, "Store One For Last");
                break;
            case STORE_ONE_FOR_SECOND:
                new LaunchSequenceCommand(intake, storage, "Store One For Second");
                break;
            default:
                new LaunchSequenceCommand(intake, storage, "Fly");
                break;
        }

    }


    // External subsystems used by commands


    // wait constants (use your existing BotPositions constants)




    // ---------------------------
    // Helper commands (return Command objects so they can be composed)
    // ---------------------------


    static boolean shootAll() {
        int countG = 0;
        for (char c : currentArtifacts.toCharArray()) {
            if (c == 'G') countG++;
        }

        // Condition 1: more than one G
        if (countG > 1) return true;

        // Condition 2: all P
        if (currentArtifacts.equals("_PPP")) return true;

        // Condition 3: current matches motif (e.g. GPP matching GPP)
        if (currentArtifacts.substring(1).equals(motif)) return true;

        return false;
    }


}
