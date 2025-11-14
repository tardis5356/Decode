package org.firstinspires.ftc.teamcode.DecodeBot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Storage;

public class LaunchSequenceCommand extends ParallelCommandGroup {

    public LaunchSequenceCommand(Intake intake, Storage storage, String desiredSequence){
        switch(desiredSequence){
            case "Fly":
                //launch all as is
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(storage::openGate),
                                new InstantCommand(()->launcOne(storage)),

                                new InstantCommand(()->moveOne(intake)),
                                new InstantCommand(()->launcOne(storage)),

                                new InstantCommand(()->moveOne(intake)),
                                new InstantCommand(()->launcOne(storage)),
                                new InstantCommand(()-> GlobalVariables.ballsShot = 0),
                                new InstantCommand(()->moveOne(intake))
                        )
                );
            break;

            case "StoreMiddle":
                //launch first, store second, launch third, launch second
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(storage::openGate),
                                //launch first
                                new InstantCommand(()->launcOne(storage)),

                                //pull in
                                new InstantCommand(()->moveOne(intake)),
                                //store second
                                //new InstantCommand(storage::closeGate),
                                new InstantCommand(storage::storeSlot),
                                new WaitCommand(BotPositions.SWAP_WAIT),
                                //pull in
                                //new InstantCommand(storage::openGate),
                                //new WaitCommand(500),
                                new InstantCommand(()->moveOne(intake)),
                                //launch third
                                new InstantCommand(()->launcOne(storage)),

                                //new InstantCommand(storage::closeGate),
                                //return second
                                new InstantCommand(()->unstore(storage)),
                                //launch second
                                new InstantCommand(()->launcOne(storage)),
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
                                new InstantCommand(()-> store(storage)),
                                //pull in
                                new InstantCommand(()->moveOne(intake)),
                                //launch second
                                new InstantCommand(()->launcOne(storage)),

                                //pull in
                                new InstantCommand(()->moveOne(intake)),
                                //launch third
                                new InstantCommand(()->launcOne(storage)),

                                //return first
                                new InstantCommand(()->unstore(storage)),
                                //launch first
                                new InstantCommand(()->launcOne(storage)),
                                new InstantCommand(()-> GlobalVariables.ballsShot = 0)
                        )
                );
            break;

            case "StoreOneForSecond":
                //store first, launch second, launch first, launch 3rd
                addCommands(
                        new InstantCommand(storage::openGate),
                        //store the first
                        new InstantCommand(()->store(storage)),
                        //pull in
                        new InstantCommand(()->moveOne(intake)),
                        new InstantCommand(()-> closeGate(storage)),
                        //launch the second
                        new InstantCommand(()->launcOne(storage)),

                        //return the first
                        new InstantCommand(()-> unstore(storage)),
                        //launch the first
                        new InstantCommand(()->launcOne(storage)),

                        new InstantCommand(()->openGate(storage)),
                        //pull in
                        new InstantCommand(()->moveOne(intake)),
                        //launch the third
                        new InstantCommand(()->launcOne(storage)),
                        new InstantCommand(()-> GlobalVariables.ballsShot = 0)

                );
            break;

            case "PullIn":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(storage::openGate),
                                new WaitCommand(BotPositions.GATE_WAIT),
                                new InstantCommand(()->moveOne(intake)),
                                new InstantCommand(storage::openGate)
                        )
                );
            break;

            case "Launch":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(()->launcOne(storage))
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
    public void launcOne(Storage s){
        new ParallelCommandGroup(
                new SequentialCommandGroup(
                new InstantCommand(s::raiseKicker),
                new WaitCommand(BotPositions.KICKER_WAIT),
                new InstantCommand(s::lowerKicker),
                new WaitCommand(BotPositions.KICKER_WAIT)
                ),
                new InstantCommand(()-> GlobalVariables.ballsShot += 1)
                );

    }

    public void moveOne(Intake i){
        new SequentialCommandGroup(
                new InstantCommand(i::in),
                new WaitCommand(BotPositions.INTAKE_WAIT),
                new InstantCommand(i::stop)
        );
    }

    public void openGate(Storage s){
        new SequentialCommandGroup(new InstantCommand(s::openGate),
                new WaitCommand(BotPositions.GATE_WAIT));
    }

    public void closeGate(Storage s){
        new SequentialCommandGroup(new InstantCommand(s::closeGate),
                new WaitCommand(BotPositions.GATE_WAIT));
    }

    public void unstore(Storage s){
        new SequentialCommandGroup(
                new InstantCommand(s::returnSlot),
                new WaitCommand(BotPositions.SWAP_WAIT)
        );
    }

    public void store(Storage s){
        new SequentialCommandGroup(
                new InstantCommand(s::storeSlot),
                new WaitCommand(BotPositions.SWAP_WAIT)
        );
    }

}

