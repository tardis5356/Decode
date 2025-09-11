package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;

public class AutoTrajectories {
    public static final Pose2d bigStartPos = new Pose2d(-56, 47, Math.toRadians(305));
    public static final Pose2d smallStartPos = new Pose2d(61, 31, Math.toRadians(180));


    // positions for strafeTo commands

    //Actions


    /*
     This defined constraint could also be called slowConstraint or
     something like that, describing it by speed like last year, but
     I do think it could be named after when it gets used during the
     program to better keep track of them (i.e. sweepingConstraint)

     Side note, because I am sure it will come up: I did some digging
     to find out why this is called MinVelConstraint. Essentially, it
     is because it's taking the lower constraint of the two given. At
     any given point, if the robot would be moving more slowly under
     the angular constraint, it will choose that. If it'd be slower
     under the translational constraint, it'd use that.
     */


    /*
     You are also able to define just an accel constraint or just a
     vel constraint, if you only want to constrain one.
     */


    /*
    See directly applied constraints (I recommend against using them
    though) on line 200.

     Let me know if you need help with this at all, we used these (at
     least their 0.5 equivalent) a bunch in Centerstage, so I'm pretty
     familiar with them   -Graham :)
     */

    public static void generateTrajectories(MecanumDrive drive) {




    }
}
