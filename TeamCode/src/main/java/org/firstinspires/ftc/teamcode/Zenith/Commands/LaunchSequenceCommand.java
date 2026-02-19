package org.firstinspires.ftc.teamcode.Zenith.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Zenith.TeleOps.DecodeTeleOp;

public class LaunchSequenceCommand extends SequentialCommandGroup {

    public LaunchSequenceCommand(Intake intake, Storage storage, String desiredSequence){
        switch(desiredSequence){
            case "Fly":
                //launch all as is
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> DecodeTeleOp.firing = true),
                                new InstantCommand(storage::openGate),
                                new InstantCommand(intake::stop),
                                new WaitCommand(200),
                                new InstantCommand(intake::in),
                                new WaitCommand(1300),
                                new InstantCommand(storage::raiseKicker),
                                new WaitCommand(300),
                                new InstantCommand(storage::lowerKicker),
                                new InstantCommand(storage::closeGate),
                                new InstantCommand(intake::stop),
                                new InstantCommand(() -> DecodeTeleOp.firing = false)
                        )
                );
            break;



            case "Launch":
                addCommands(
                        new SequentialCommandGroup(
                               launchOne(storage)
                        )
                );
            break;

        }

    }
    public static Command launchOne(Storage s){
        return

                new SequentialCommandGroup(
                        new InstantCommand(() -> DecodeTeleOp.firing = true),
                        new InstantCommand(s::openGate),
                        new InstantCommand(s::raiseKicker),
                        new WaitCommand(250),
                        new InstantCommand(s::lowerKicker),
                        new InstantCommand(s::closeGate),
        new InstantCommand(() -> DecodeTeleOp.firing = false)
                );

    }



//    public static Command openGate(Storage s){
//        return new SequentialCommandGroup(new InstantCommand(s::openGate),
//                new WaitCommand(BotPositions.GATE_WAIT));
//    }
//
//    public static Command closeGate(Storage s){
//        return new SequentialCommandGroup(new InstantCommand(s::closeGate),
//                new WaitCommand(BotPositions.GATE_WAIT));
//    }



}


