package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.DecodeBot.Commands.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.DecodeBot.TeleOps.DecodeTeleOp;

public class RecoveryBuilder {



    public SequentialCommandGroup BuildRecSequence(LaunchSequenceCommand pullIn, LaunchSequenceCommand store, LaunchSequenceCommand unstore, LaunchSequenceCommand launch, DecodeTeleOp.shootModes shootMode, Storage storage ){
        SequentialCommandGroup recovery = null;

        if(GlobalVariables.ballsShot == 3){
            GlobalVariables.ballsShot = 0;
        }

        //First we look at what sequence we were running
        if(shootMode == DecodeTeleOp.shootModes.FLY){
            //Check if there is already something in the shooter area
            if (GlobalVariables.currentArtifacts.substring(1,2) != "V"){
                recovery = new SequentialCommandGroup(pullIn);
            }
            else{
                recovery = new SequentialCommandGroup();
            }
        }
        else if(shootMode == DecodeTeleOp.shootModes.STORE_MIDDLE){
            //Check how far along you were
            if(GlobalVariables.ballsShot == 0){

            }
            else if(GlobalVariables.ballsShot == 1){

            }
            else if(GlobalVariables.ballsShot == 2){

            }
        }
        else if(shootMode == DecodeTeleOp.shootModes.STORE_ONE_FOR_LAST){
            if(GlobalVariables.ballsShot == 0){

            }
            else if(GlobalVariables.ballsShot == 1){

            }
            else if(GlobalVariables.ballsShot == 2){

            }
        }
        else if(shootMode == DecodeTeleOp.shootModes.STORE_ONE_FOR_SECOND){
            if(GlobalVariables.ballsShot == 0){

            }
            else if(GlobalVariables.ballsShot == 1){

            }
            else if(GlobalVariables.ballsShot == 2){

            }
        }

        return recovery;
    }
}
