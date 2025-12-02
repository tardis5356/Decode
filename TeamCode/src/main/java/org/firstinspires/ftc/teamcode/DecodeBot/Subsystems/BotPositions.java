package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BotPositions {





    // These are variables throughout the code that will not change unless we reindex.

    public static int PURPLE_R_MIN, PURPLE_R_MAX, PURPLE_G_MIN, PURPLE_G_MAX, PURPLE_B_MIN, PURPLE_B_MAX;
    public static int GREEN_R_MIN, GREEN_R_MAX, GREEN_G_MIN, GREEN_G_MAX, GREEN_B_MIN, GREEN_B_MAX;
    //public static int SLOT1_FIRE = 360, SLOT2_FIRE = 120, SLOT3_FIRE = 240, SLOT1_LOAD = 180, SLOT2_LOAD = 300, SLOT3_LOAD = 60;
    //public static double TURRET_P = 0.000000, TURRET_I = 0.000, TURRET_D = 0.00000;
    public static double TURRET_TICKS_PER_DEGREE = 104.55 * (180/187) , TURRET_RADIANS_PER_TICK = 1/(TURRET_TICKS_PER_DEGREE) * (Math.PI/180);
    public static double TURRET_P = 0.0000, TURRET_I = 0.000, TURRET_D = 0.00000;
    //public static double TURRET_TICKS_PER_DEGREE = (8192/360)*(373.6/360)*(358.91083/360) /*1.491851852*/  , TURRET_RADIANS_PER_TICK = 1/(TURRET_TICKS_PER_DEGREE) * (Math.PI/180);

   //PTO Relevant stuff
    public static double PTO_ENGAGED = 0.28, PTO_DISENGAGED = 0, BELLYPAN_LEFT_LATCHED = 0.27, BELLYPAN_RIGHT_LATCHED = 0.5, BELLYPAN_LEFT_UNLATCHED = 0., BELLYPAN_RIGHT_UNLATCHED = 0.2;

    public static double BREAKPAD_ACTIVE = 0, BREAKPAD_INACTIVE = 1;

    public static double CAMERA_RADIUS = 5.23956, TURRET_OFFSET_X = -2.5590, TURRET_OFFSET_Y = 0;

    //positions of servos in StorageSubsystem
    public static double GATE_OPEN = .205, GATE_CLOSED = .5, KICKER_UP = .58, KICKER_DOWN = .31, SLOT_STORED = .53, SLOT_FLY = .88, BACK_OPEN = .44, BACK_CLOSE = .57;

    public static long KICKER_WAIT = 500, GATE_WAIT = 500, SWAP_WAIT = 500, INTAKE_WAIT = 1000;

    public static double LONG_DISTANCE_TPS = 1400, SHORT_DISTANCE_TPS;

}
