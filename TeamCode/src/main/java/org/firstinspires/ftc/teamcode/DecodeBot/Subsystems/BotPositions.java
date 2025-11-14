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
    //public static int SLOT1_FIRE = 360, SLOT2_FIRE = 120, SLOT3_FIRE = 240, SLOT1_LOAD = 180, SLOT2_LOAD = 300, SLOT3_LOAD = 60;
    public static double TURRET_P = 0.0005, TURRET_I = 0.000, TURRET_D = 0.000;
    public static double TURRET_360_TURN_TICKS = 380 * 90/100 * 360/365, TURRET_DEGREE_TO_TICK_MULTIPLIER = TURRET_360_TURN_TICKS/360, TURRET_TICK_TO_RADIAN_MULTIPLIER = TURRET_DEGREE_TO_TICK_MULTIPLIER * (Math.PI/180);

   //PTO Relevant stuff
    public static double PTO_ENGAGED = 0.5, PTO_DISENGAGED = 0, BELLYPAN_LEFT_LATCHED = 0.27, BELLYPAN_RIGHT_LATCHED = 0.5, BELLYPAN_LEFT_UNLATCHED = 0., BELLYPAN_RIGHT_UNLATCHED = 0.2;

    public static double BREAKPAD_ACTIVE = 0, BREAKPAD_INACTIVE = 1;

    public static double CAMERA_RADIUS = 1.75, TURRET_OFFSET_X = -3.25, TURRET_OFFSET_Y = -3;

    //positions of servos in StorageSubsystem
    public static double GATE_OPEN = .205, GATE_CLOSED = .5, KICKER_UP = .55, KICKER_DOWN = .31, SLOT_STORED = .53, SLOT_FLY = .88, BACK_OPEN = .44, BACK_CLOSE = .57;

    public static long KICKER_WAIT = 500, GATE_WAIT = 500, SWAP_WAIT = 500, INTAKE_WAIT = 1000;

    public static double LONG_DISTANCE_TPS = 1400, SHORT_DISTANCE_TPS;

}
