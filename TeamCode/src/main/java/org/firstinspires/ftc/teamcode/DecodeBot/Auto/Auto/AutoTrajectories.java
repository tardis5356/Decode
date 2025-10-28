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
    public static final Pose2d BackStartPos = allianceCoordinate(new Pose2d(63,30.5,Math.toRadians(180)));
    public static final Pose2d FrontStartPos = allianceCoordinate(new Pose2d(-36,33,Math.toRadians(270)));
    public static final Pose2d FrontSpikePos = allianceCoordinate(new Pose2d(-12,29,Math.toRadians(90))); //PPG
    public static final Pose2d FrontSpikeIntakePos = allianceCoordinate(new Pose2d(-12,44,Math.toRadians(90))); //PPG
    public static final Pose2d MidSpikePos = allianceCoordinate(new Pose2d(12,29,Math.toRadians(90))); //PGP
    public static final Pose2d MidSpikeIntakePos = allianceCoordinate(new Pose2d(12,44,Math.toRadians(90))); //PGP
    public static final Pose2d BackSpikePos = allianceCoordinate(new Pose2d(35,29,Math.toRadians(90))); //GPP
    public static final Pose2d BackSpikeIntakePos = allianceCoordinate(new Pose2d(35,44,Math.toRadians(90))); //PGP
    public static final Pose2d CornerPos = allianceCoordinate(new Pose2d(61,54,Math.toRadians(90))); //pick up corner PGP
    public static final Pose2d CornerIntakePos = allianceCoordinate(new Pose2d(61,61,Math.toRadians(90))); //pick up corner PGP
    public static final Pose2d GatePrepPos = allianceCoordinate(new Pose2d(0,48,Math.toRadians(180))); //go to gate position
    public static final Pose2d GateReleasePos = allianceCoordinate(new Pose2d(0,56,Math.toRadians(180))); //open the gate

    //Actions
    public static Action startToBackSpike;
    public static Action backSpikeIntake;
    public static Action backSpikeToCornerPickup;
    public static Action cornerIntake;
    public static Action cornerPickupToMidSpike;
    public static Action midSpikeIntake;
//    public static Action MidSpikeToGate;
    public static Action frontSpikeIntake;
    public static Action midSpikeToFrontSpike;
    public static Action frontSpikeToGate;
    public static Action shootPreload;
    public static Action shootBackSpike;
    public static Action shootMidSpike;
    public static Action shootFrontSpike;
    private static double startPosTangent;



    public static void generateTrajectories(MecanumDrive drive, int choice1) {
        if (startPos == FrontStartPos){
            startPosTangent = 0;
        } else if (startPos == BackStartPos){
            startPosTangent = 0;
        }



        startToBackSpike =
                drive.actionBuilder(startPos)
                        .setTangent(allianceTangent(180))
                        .splineToLinearHeading(BackSpikePos, allianceTangent(270))//fill in tangent
                        .build();

        backSpikeIntake =
                drive.actionBuilder(BackSpikePos)
                        .setTangent(allianceTangent(270))
                        .splineToLinearHeading(BackSpikeIntakePos, allianceTangent(270))
                        .build();

        backSpikeToCornerPickup =
                drive.actionBuilder(BackSpikeIntakePos)
                        .setTangent(allianceTangent(90))
                        .splineToLinearHeading(CornerPos, allianceTangent(270))
                        .build();

        cornerIntake =
                drive.actionBuilder(CornerPos)
                        .setTangent(allianceTangent(270))
                        .splineToLinearHeading(CornerIntakePos, allianceTangent(270))
                        .build();

        cornerPickupToMidSpike =
                drive.actionBuilder(CornerIntakePos)
                        .setTangent(allianceTangent(90))
                        .splineToLinearHeading(MidSpikePos, allianceTangent(270))
                        .build();

        midSpikeIntake =
                drive.actionBuilder(MidSpikePos)
                        .setTangent(allianceTangent(270))
                        .splineToLinearHeading(MidSpikeIntakePos, allianceTangent(270))
                        .build();

//        MidSpikeToGate =
//            drive.actionBuilder(MidSpikeIntakePos)
//                    .setTangent(allianceTangent(90))
//                    .splineToLinearHeading(GatePrepPos, allianceTangent(270))
//                    .build();

        midSpikeToFrontSpike =
            drive.actionBuilder(MidSpikeIntakePos)
                    .setTangent(allianceTangent(90))
                    .splineToLinearHeading(FrontSpikePos, allianceTangent(270))
                    .build();

        frontSpikeIntake =
                drive.actionBuilder(FrontSpikePos)
                        .setTangent(allianceTangent(270))
                        .splineToLinearHeading(FrontSpikeIntakePos,allianceTangent(270))
                        .build();
        frontSpikeToGate =
                drive.actionBuilder(FrontSpikePos)
                     .setTangent(allianceTangent(90))
                     .splineToLinearHeading(BackSpikePos, allianceTangent(270))
                     .build();
    }
}
