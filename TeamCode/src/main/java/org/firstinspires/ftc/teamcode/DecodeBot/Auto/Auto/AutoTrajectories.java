package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;

import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.DecodeAuto.startPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.aColor;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;

public class AutoTrajectories {

   //heading input is degrees
    private static Pose2d allianceCoordinate(Pose2d coordinate) {

        Pose2d mirroredCoordinate;

        if(aColor == "blue"){
            mirroredCoordinate =new Pose2d(new Vector2d(coordinate.position.x, coordinate.position.y * -1) , Math.toRadians(-coordinate.heading.toDouble() ));
            return mirroredCoordinate;
        }else {
            coordinate =new Pose2d(new Vector2d(coordinate.position.x, coordinate.position.y ), Math.toRadians(coordinate.heading.toDouble()));
            return coordinate;
        }

    }

    //input is degrees
    private static double allianceTangent(double tangent) {

        double mirroredTangent;

        if(aColor == "blue"){
            mirroredTangent = Math.toRadians(-tangent);
            return mirroredTangent;
        }else {
            return Math.toRadians(tangent);
        }

    }
    public static final Pose2d bigStartPos = allianceCoordinate(new Pose2d(-56, 47, 305));
    public static final Pose2d smallStartPos =allianceCoordinate( new Pose2d(61, 31, 180));

    public static final Pose2d markPos =allianceCoordinate( new Pose2d(0, 0, 0));

    //Actions
public static Action startToMark;

private static double startPosTangent;



    public static void generateTrajectories(MecanumDrive drive, int choice1) {
        if (startPos == bigStartPos){
            startPosTangent = 305;
        } else if (startPos == smallStartPos){
            startPosTangent = 0;
        }



        startToMark =
                drive.actionBuilder(startPos)
                        .setTangent(allianceTangent(startPosTangent))
                        .splineToLinearHeading(markPos, allianceTangent(0))//fill in tangent
                        .build();


    }
}
