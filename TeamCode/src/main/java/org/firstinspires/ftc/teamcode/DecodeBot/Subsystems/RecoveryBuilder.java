package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.DecodeBot.Commands.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.DecodeBot.TeleOps.DecodeTeleOp;

public class RecoveryBuilder {



    public SequentialCommandGroup BuildRecSequence(LaunchSequenceCommand pullIn, LaunchSequenceCommand pullInAgain, LaunchSequenceCommand store, LaunchSequenceCommand unstore, LaunchSequenceCommand launch, DecodeTeleOp.shootModes shootMode, Storage storage ){
        SequentialCommandGroup recovery = null;

        //The logic below has _ mean empty. If the yolk/storage is _ then it should be in the fly position

        //TODO: make this the primary if for the whole thing rather than a parallel one, I can imagine the sequence issues.
        if(GlobalVariables.ballsShot == 3){
            recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker));
            GlobalVariables.ballsShot = 0;
        }
        //First we look at what sequence we were running
        else if(shootMode == DecodeTeleOp.shootModes.FLY){
            //Check if there is already something in the shooter area
            if (GlobalVariables.currentArtifacts.substring(1,2) == "_"){
                recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), pullIn);
            }
            else{
                recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker));
            }
        }
        else if(shootMode == DecodeTeleOp.shootModes.STORE_MIDDLE){
            //Check how far along you were
            if(GlobalVariables.ballsShot == 0){
                recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker));
            }
            else if(GlobalVariables.ballsShot == 1){
                //Check yolk situation and the shooter

                //if the yolk is not stored and the shooter is empty
                if(GlobalVariables.currentArtifacts.substring(0,1) == "_" && GlobalVariables.currentArtifacts.substring(1,2) == "_"){
                    recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), pullIn, store, pullInAgain);
                }
                //if the yolk isn't empty (and thus stored) and the shooter is empty
                else if(GlobalVariables.currentArtifacts.substring(0,1) != "_" && GlobalVariables.currentArtifacts.substring(1,2) == "_"){

                    recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), pullIn);
                }
            }
            else if(GlobalVariables.ballsShot == 2){
                recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), unstore);
            }
        }
        else if(shootMode == DecodeTeleOp.shootModes.STORE_ONE_FOR_LAST){
            if(GlobalVariables.ballsShot == 0){
                //check state of the yolk
                if(GlobalVariables.currentArtifacts.substring(0,1) != "_" /*if the yolk is in stored*/){
                    if(GlobalVariables.currentArtifacts.substring(1,2) == "_" /*if the shooter is empty*/){
                        recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), pullIn);
                    }
                    else{
                        recovery = new SequentialCommandGroup();
                    }
                }
                else{
                    recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), store, pullIn);
                }
            }
            else if(GlobalVariables.ballsShot == 1){
                if(GlobalVariables.currentArtifacts.substring(1,2) == "_" /*if the shooter is empty*/){
                    recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), pullIn);
                }
                else{
                    recovery = new SequentialCommandGroup();
                }
            }
            else if(GlobalVariables.ballsShot == 2){
                recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), unstore);
            }
        }
        else if(shootMode == DecodeTeleOp.shootModes.STORE_ONE_FOR_SECOND){
            if(GlobalVariables.ballsShot == 0){
                if(GlobalVariables.currentArtifacts.substring(0,1) != "_" /*if the yolk is in stored*/){
                    if(GlobalVariables.currentArtifacts.substring(1,2) == "_" /*if the shooter is empty*/){
                        recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), pullIn);
                    }
                    else{
                        recovery = new SequentialCommandGroup();
                    }
                }
                else{
                    recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), store, pullIn);
                }
            }
            else if(GlobalVariables.ballsShot == 1){
                if(GlobalVariables.currentArtifacts.substring(0,1) != "_" /*if the yolk is in stored*/){
                    recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), unstore);
                }
                else{
                    recovery = new SequentialCommandGroup();
                }
            }
            else if(GlobalVariables.ballsShot == 2){
                //Check if there is already something in the shooter area
                if (GlobalVariables.currentArtifacts.substring(1,2) == "_"){
                    recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker), pullIn);
                }
                else{
                    recovery = new SequentialCommandGroup(new InstantCommand(storage :: lowerKicker));
                }
            }
        }
        else{
            recovery = new SequentialCommandGroup();
        }

        return recovery;
    }
}
