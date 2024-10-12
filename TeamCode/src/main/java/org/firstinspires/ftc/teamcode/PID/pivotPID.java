package org.firstinspires.ftc.teamcode.PID;


import com.acmerobotics.dashboard.config.Config;

@Config
public class pivotPID {
    public static double dP = 0.003;
    public static double dI = 0;
    public static double dD = 0;

    public static double uP = 0.008;
    public static double uI = 0.007;
    public static double uD = 0.0000;

    public static double tP = 0.005;
    public static double tI = 0.007;
    public static double tD = 0.0001;
}
