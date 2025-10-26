package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;

import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.DecodeAuto.startPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.aColor;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;

public class AutoTrajectories {

   //heading input is degrees
    public static Pose2d allianceCoordinate(Pose2d coordinate) {

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
    public static final Pose2d BackStartPos = allianceCoordinate(new Pose2d(63,33,180));
    public static final Pose2d FrontStartPos = allianceCoordinate(new Pose2d(-36,33,270));
    public static final Pose2d FrontSpikePos = allianceCoordinate(new Pose2d(-12,31.5,90)); //PPG
    public static final Pose2d MidSpikePos = allianceCoordinate(new Pose2d(12,31.5,90)); //PGP
    public static final Pose2d BackSpikePos = allianceCoordinate(new Pose2d(36,31.5,90)); //GPP
    public static final Pose2d CornerPickupPos = allianceCoordinate(new Pose2d(60,60,90)); //pick up corner PGP
    public static final Pose2d GatePrepPos = allianceCoordinate(new Pose2d(0,48,180)); //go to gate position
    public static final Pose2d GateReleasePos = allianceCoordinate(new Pose2d(0,56,180)); //open the gate

    //Actions
    public static Action startToBackSpike;
    public static Action BackSpikeToCornerPickup;
    public static Action CornerPickupToMidSpike;
    public static Action MidSpikeToGate;
    public static Action MidSpikeToFrontSpike;
    public static Action FrontSpikeToGate;

    private static double startPosTangent;



    public static void generateTrajectories(MecanumDrive drive, int choice1) {
        if (startPos == FrontStartPos){
            startPosTangent = 305;
        } else if (startPos == BackStartPos){
            startPosTangent = 0;
        }



        startToBackSpike =
                drive.actionBuilder(startPos)
                        .setTangent(allianceTangent(180))
                        .splineToLinearHeading(BackSpikePos, allianceTangent(90))//fill in tangent
                        .build();

        BackSpikeToCornerPickup =
                drive.actionBuilder(BackSpikePos)
                        .setTangent(allianceTangent(0))
                        .splineToLinearHeading(CornerPickupPos, allianceTangent(90))
                        .build();

        CornerPickupToMidSpike =
                drive.actionBuilder(CornerPickupPos)
                        .setTangent(allianceTangent(270))
                        .splineToLinearHeading(MidSpikePos, allianceTangent(90))
                        .build();

        MidSpikeToGate =
            drive.actionBuilder(MidSpikePos)
                    .setTangent(allianceTangent(180))
                    .splineToLinearHeading(GatePrepPos, allianceTangent(90))
                    .build();

//        MidSpikeToFrontSpike =
//            drive.actionBuilder(MidSpikePos)
//                    .setTangent(allianceTangent(180))
//                    .splineToLinearHeading(BackSpikePos, allianceTangent(90))
//                    .build();
//
//        FrontSpikeToGate =
//                drive.actionBuilder(FrontSpikePos)
//                     .setTangent(allianceTangent(180))
//                     .splineToLinearHeading(BackSpikePos, allianceTangent(90))
//                     .build(); //talk with drivers if this is needed
    }
}
