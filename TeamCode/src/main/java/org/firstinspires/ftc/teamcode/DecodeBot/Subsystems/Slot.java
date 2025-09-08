package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

public class Slot {
    //The Spindex has 3 slots, one for each artifact.
    //We will make a seperate Slot object for each in the actual teleop
    //This is a way to store what color artifact is in what slot

    public String artifactColor;
    //this String is either purple, green, or empty.


    //method takes the rgb values of a color sensor defined in the Spindex
    public void setArtifactColor(int r, int g, int b){
        if ((r>BotPositions.PURPLE_R_MIN && r<BotPositions.PURPLE_R_MAX) || (g<BotPositions.PURPLE_G_MAX) || (b>BotPositions.PURPLE_B_MIN && b<BotPositions.PURPLE_B_MAX)){
            artifactColor = "purple";
        }
        else if((r>BotPositions.GREEN_R_MIN && r<BotPositions.GREEN_R_MAX) || (g>BotPositions.GREEN_G_MIN && g<BotPositions.GREEN_G_MAX) || (b>BotPositions.GREEN_B_MIN && b<BotPositions.GREEN_B_MAX)){
            artifactColor = "green";
        }
        else{
            artifactColor = "empty";
        }
    }

}
