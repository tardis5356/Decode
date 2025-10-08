package org.firstinspires.ftc.teamcode.DecodeBot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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
                                new WaitCommand(500),
                                new InstantCommand(()->moveOne(intake)),
                                new WaitCommand(500),
                                new InstantCommand(()->launcOne(storage)),
                                new WaitCommand(500),
                                new InstantCommand(()->moveOne(intake)),
                                new WaitCommand(500),
                                new InstantCommand(()->launcOne(storage)),
                                new WaitCommand(500),
                                new InstantCommand(()->moveOne(intake)),
                                new WaitCommand(500)

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
                                new WaitCommand(500),
                                //pull in
                                new InstantCommand(()->moveOne(intake)),
                                new WaitCommand(500),
                                //store second
                                new InstantCommand(storage::storeSlot),
                                new WaitCommand(500),
                                //pull in
                                new InstantCommand(()->moveOne(intake)),
                                new WaitCommand(500),
                                //launch third
                                new InstantCommand(()->launcOne(storage)),
                                new WaitCommand(500),
                                //return second
                                new InstantCommand(storage::returnSlot),
                                new WaitCommand(500),
                                //launch second
                                new InstantCommand(()->launcOne(storage)),
                                new WaitCommand(500)
                        )
                );
            break;

            case "StoreOneForLast":
                //store first, launch second and third, launch first
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(storage::openGate),
                                //store first
                                new InstantCommand(storage::storeSlot),
                                new WaitCommand(500),
                                //pull in
                                new InstantCommand(()->moveOne(intake)),
                                new WaitCommand(500),
                                //launch second
                                new InstantCommand(()->launcOne(storage)),
                                new WaitCommand(500),
                                //pull in
                                new InstantCommand(()->moveOne(intake)),
                                new WaitCommand(500),
                                //launch third
                                new InstantCommand(()->launcOne(storage)),
                                new WaitCommand(500),
                                //return first
                                new InstantCommand(storage::returnSlot),
                                new WaitCommand(500),
                                //launch first
                                new InstantCommand(()->launcOne(storage))
                        )
                );
            break;

            case "StoreOneForSecond":
                //store first, launch second, launch first, launch 3rd
                addCommands(
                        new InstantCommand(storage::openGate),
                        //store the first
                        new InstantCommand(storage::storeSlot),
                        new WaitCommand(500),
                        //pull in
                        new InstantCommand(()->moveOne(intake)),
                        new WaitCommand(500),
                        new InstantCommand(storage::closeGate),
                        //launch the second
                        new InstantCommand(()->launcOne(storage)),
                        new WaitCommand(500),
                        //return the first
                        new InstantCommand(storage::returnSlot),
                        new WaitCommand(500),
                        //launch the first
                        new InstantCommand(()->launcOne(storage)),
                        new WaitCommand(500),
                        new InstantCommand(storage::openGate),
                        new WaitCommand(500),
                        //pull in
                        new InstantCommand(()->moveOne(intake)),
                        new WaitCommand(500),
                        //launch the third
                        new InstantCommand(()->launcOne(storage))

                );
            break;
        }

    }
    public void launcOne(Storage s){
        new SequentialCommandGroup(
                new InstantCommand(s::raiseKicker),
                new WaitCommand(500),
                new InstantCommand(s::lowerKicker)
        );
    }

    public void moveOne(Intake i){
        new SequentialCommandGroup(
                new InstantCommand(i::in),
                new WaitCommand(1000),
                new InstantCommand(i::stop)
        );
    }

}

