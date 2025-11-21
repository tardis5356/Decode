package org.firstinspires.ftc.teamcode.DecodeBot.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Storage;

public class LaunchSequenceCommand extends SequentialCommandGroup {

    public LaunchSequenceCommand(Intake intake, Storage storage, String desiredSequence){
        switch(desiredSequence){
            case "Fly":
                //launch all as is
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(storage::openGate),
                                launcOne(storage),
                                moveOne(intake),
                                launcOne(storage),
                                moveOne(intake),
                                launcOne(storage),
                                new InstantCommand(()-> GlobalVariables.ballsShot = 0),
                                moveOne(intake)
                        )
                );
            break;

            case "StoreMiddle":
                //launch first, store second, launch third, launch second
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(storage::openGate),
                                //launch first
                                launcOne(storage),

                                //pull in
                                moveOne(intake),
                                //store second
                                //new InstantCommand(storage::closeGate),
                                new InstantCommand(storage::storeSlot),
                                new WaitCommand(BotPositions.SWAP_WAIT),
                                //pull in
                                //new InstantCommand(storage::openGate),
                                //new WaitCommand(500),
                                moveOne(intake),
                                //launch third
                                launcOne(storage),

                                //new InstantCommand(storage::closeGate),
                                //return second
                                unstore(storage),
                                //launch second
                                launcOne(storage),
                                new InstantCommand(()-> GlobalVariables.ballsShot = 0)
                        )
                );
            break;

            case "StoreOneForLast":
                //store first, launch second and third, launch first
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(storage::openGate),
                                //store first
                                store(storage),
                                //pull in
                                moveOne(intake),
                                //launch second
                                launcOne(storage),

                                //pull in
                                moveOne(intake),
                                //launch third
                                launcOne(storage),

                                //return first
                                unstore(storage),
                                //launch first
                                launcOne(storage),
                                new InstantCommand(()-> GlobalVariables.ballsShot = 0)
                        )
                );
            break;

            case "StoreOneForSecond":
                //store first, launch second, launch first, launch 3rd
                addCommands(
                        new InstantCommand(storage::openGate),
                        //store the first
                        store(storage),
                        //pull in
                        moveOne(intake),
                        closeGate(storage),
                        //launch the second
                        launcOne(storage),

                        //return the first
                        unstore(storage),
                        //launch the first
                        launcOne(storage),

                        openGate(storage),
                        //pull in
                        moveOne(intake),
                        //launch the third
                        launcOne(storage),
                        new InstantCommand(()-> GlobalVariables.ballsShot = 0)

                );
            break;

            case "PullIn":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(storage::openGate),
                                new WaitCommand(BotPositions.GATE_WAIT),
                                moveOne(intake),
                                new InstantCommand(storage::openGate)
                        )
                );
            break;

            case "Launch":
                addCommands(
                        new SequentialCommandGroup(
                               launcOne(storage)
                        )
                );
            break;

            case "Store":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(storage::closeGate),
                                new WaitCommand(BotPositions.GATE_WAIT),
                                new InstantCommand(storage::storeSlot),
                                new WaitCommand(BotPositions.SWAP_WAIT),
                                new InstantCommand(storage::openGate)

                        )
                );
            break;

            case "UnStore":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(storage::closeGate),
                                new WaitCommand(BotPositions.GATE_WAIT),
                                new InstantCommand(storage::returnSlot),
                                new WaitCommand(BotPositions.SWAP_WAIT),
                                new InstantCommand(storage::openGate)

                        )
                );
            break;

            case "Scram":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::stop),
                                new InstantCommand(storage::lowerKicker),
                                new InstantCommand(storage::openGate),
                                new InstantCommand(storage::storeSlot)
                        )
                );
            break;
        }

    }
    public Command launcOne(Storage s){
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                new InstantCommand(s::raiseKicker),
                new WaitCommand(BotPositions.KICKER_WAIT),
                new InstantCommand(s::lowerKicker),
                new WaitCommand(BotPositions.KICKER_WAIT)
                ),
                new InstantCommand(()-> GlobalVariables.ballsShot += 1)
                );

    }

    public Command moveOne(Intake i){
        return new SequentialCommandGroup(
                new InstantCommand(i::in),
                new WaitCommand(BotPositions.INTAKE_WAIT),
                new InstantCommand(i::stop)
        );
    }

    public Command openGate(Storage s){
        return new SequentialCommandGroup(new InstantCommand(s::openGate),
                new WaitCommand(BotPositions.GATE_WAIT));
    }

    public Command closeGate(Storage s){
        return new SequentialCommandGroup(new InstantCommand(s::closeGate),
                new WaitCommand(BotPositions.GATE_WAIT));
    }

    public Command unstore(Storage s){

        return new SequentialCommandGroup(
                new InstantCommand(s::returnSlot),
                new WaitCommand(BotPositions.SWAP_WAIT)
        );


    }



    public Command store(Storage s){
        return new SequentialCommandGroup(
                new InstantCommand(s::storeSlot),
                new WaitCommand(BotPositions.SWAP_WAIT)
        );
    }

}


