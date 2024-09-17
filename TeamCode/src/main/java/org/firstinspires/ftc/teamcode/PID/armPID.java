package org.firstinspires.ftc.teamcode.PID;


import com.acmerobotics.dashboard.config.Config;

@Config
public class armPID {
    public static double kV = 0.0005;
    public static double kA = 0.00005;
    public static double kStatic = 0.004;
    public static double MAX_VEL = 2000;
    public static double MAX_ACCEL =6000;
}

