package org.firstinspires.ftc.teamcode.PID;


import com.acmerobotics.dashboard.config.Config;

@Config
public class pivotPID {
    public static double kV = 0.01505;
    public static double kA = 0.0025;
    public static double kStatic = 0.00005;
    public static double MAX_VEL = 60;
    public static double MAX_ACCEL = 40;
}
