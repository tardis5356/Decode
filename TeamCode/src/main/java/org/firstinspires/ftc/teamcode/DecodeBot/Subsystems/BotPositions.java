package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
@Config
public class BotPositions {





    // These are variables throughout the code that will not change unless we reindex.

    public static int PURPLE_R_MIN, PURPLE_R_MAX, PURPLE_G_MIN, PURPLE_G_MAX, PURPLE_B_MIN, PURPLE_B_MAX;
    public static int GREEN_R_MIN, GREEN_R_MAX, GREEN_G_MIN, GREEN_G_MAX, GREEN_B_MIN, GREEN_B_MAX;
    public static double TURRET_P, TURRET_I, TURRET_D;
    public static double TURRET_360_TURN_TICKS = 0, TURRET_DEGREE_TO_TICK_MULTIPLIER = TURRET_360_TURN_TICKS/360, TURRET_TICK_TO_RADIAN_MULTIPLIER = TURRET_DEGREE_TO_TICK_MULTIPLIER * (Math.PI/180);
    public static double PTO_ENGAGED = 0, PTO_DISENGAGED = 0;

    public static double CAMERA_RADIUS = 2.5, TURRET_OFFSET_X = 0.0 , TURRET_OFFSET_Y = 0.0;

}
