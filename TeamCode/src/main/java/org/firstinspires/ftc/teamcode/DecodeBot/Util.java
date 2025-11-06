package org.firstinspires.ftc.teamcode.DecodeBot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Util {

    public static Pose2d vectorFToPose2d(VectorF vector, float aTagPose) {
        return new Pose2d(
                new Vector2d(vector.get(0), vector.get(1)), aTagPose
        );

    }



}
