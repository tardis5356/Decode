package com.example.meepmeeptesting;

import static com.acmerobotics.roadrunner.geometry.Pose2dKt.times;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;



public class MeepMeepTesting {
static String aColor = "blue";

    public static final Pose2d BackStartPos = allianceCoordinate(new Pose2d(63,33,Math.toRadians(180)));
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
    static Pose2d mirroredCoordinate;
    private static Pose2d allianceCoordinate(Pose2d coordinate) {
        if(aColor == "blue"){
            mirroredCoordinate =new Pose2d(new Vector2d(coordinate.getX(), coordinate.getY() * -1) , Math.toRadians(Math.toDegrees(-coordinate.getHeading()) ));
            return mirroredCoordinate;
        }else {
        return coordinate;
        }

    }


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14, 17) // width, height in inches (or your field unit)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
               .followTrajectorySequence(driveShim ->
                       driveShim.trajectorySequenceBuilder(BackStartPos)
                               .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(BackSpikePos, Math.toRadians(270))//fill in tangent
                               .waitSeconds(0.5)
                               .setTangent(Math.toRadians(270))
                               .splineToLinearHeading(BackSpikeIntakePos, Math.toRadians(270))
                               .waitSeconds(0.5)
                               .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(CornerPos, Math.toRadians(270))
                               .waitSeconds(0.5)
                               .setTangent(Math.toRadians(270))
                               .splineToLinearHeading(CornerIntakePos, Math.toRadians(270))
                               .waitSeconds(0.5)
                               .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(MidSpikePos, Math.toRadians(270))
                               .waitSeconds(0.5)
                               .setTangent(Math.toRadians(270))
                               .splineToLinearHeading(MidSpikeIntakePos, Math.toRadians(270))
                               .waitSeconds(0.5)
                               .setTangent(Math.toRadians(90))
                               .splineToLinearHeading(FrontSpikePos, Math.toRadians(270))
                               .waitSeconds(0.5)
                               .setTangent(Math.toRadians(270))
                               .splineToLinearHeading(FrontSpikeIntakePos,Math.toRadians(270))
                               .waitSeconds(0.5)
                               .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(GatePrepPos, Math.toRadians(270))
                                .waitSeconds(0.5)
                               .build());


        Image img = null;
       // try { img = ImageIO.read(new File("C:\\Users\\trant\\Downloads\\field-2025-official.png")); }
        try { img = ImageIO.read(new File("C:\\Users\\Icy\\Downloads\\field-2025-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setAxesInterval(10)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}