package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class constants{
    public static final Pose2d startPos = new Pose2d(0,0,Math.toRadians(270));
public static final class autoTrajectory {
    public static final class testAutoConstants {

    }
}

public static final class autoGetPoints{
    public static final Pose2d sample1 = new Pose2d();
    public static final Pose2d sample2 = new Pose2d(-58,-47,Math.toRadians(90));
    public static final Pose2d sample3 = new Pose2d(-47.5,-47,Math.toRadians(90));
    public static final Pose2d sample4 = new Pose2d(47.5,-47,Math.toRadians(90));
    public static final Pose2d sample5 = new Pose2d(58,-47,Math.toRadians(90));
    public static final Pose2d sample6 = new Pose2d(58,-47,Math.toRadians(60));
    public static final Pose2d basket = new Pose2d(-57,-55,Math.toRadians(225));
    public static final Pose2d leftChamber = new Pose2d(-10,-35,Math.toRadians(90));
    public static final Pose2d rightChamber = new Pose2d(10,-35,Math.toRadians(90));
    public static final Pose2d leftStartPos = new Pose2d(-10, -62, Math.toRadians(90));
    public static final Pose2d rightStartPos = new Pose2d(10, -62, Math.toRadians(90));
}

public static final class clawPoints{
    public static final double closePos = .2;
    public static final double openPos = .5;
}

public static final class limelightCam{
    public static final double focalLengthMM = 3.6;
    public static final int resX = 640;
    public static final int resY = 480;
    public static final double sizeXMM = 3.6;
    public static final double sizeYMM = 2.7;
    public static final int objDetect = 0;
    public static final int aprilTag = 1;
}
public static final class armLimits{
    public static final double maxPivotRange =90;
    public static final double minExtensionRange = 0;
    public static final double maxExtensionRange =1900;
}
public static final class armConstants{
    public static final class middle{
        public static double P = 0.002;
        public static double I = 0.07;
        public static double D = 0.0002;
    }
    public static final class chambers{
        public static double P = 0.002;
        public static double I = 0.07;
        public static double D = 0.0002;
    }
    public static final class baskets{
        public static double P = 0.002;
        public static double I = 0.07;
        public static double D = 0.0002;
    }
    public static final double tolerance = 0;


}

public static final class pivotThreshold{
    public static final double low = 100;
    public static final double mid = 200;
    public static final double high = 300;
}
public static final class pivotConstants{


        public static double P = 0.003;
        public static double I = 0.001;
        public static double D = 0.000;


}
public static final class armGearRatio{
    public static final double ticksPerRevolution = 28;
    public static final double motorGearRatio = 60;
    public static final double motorToPivotRatio = 40.0/15.0;
    public static final double countsPerArmRev = ticksPerRevolution*motorGearRatio*motorToPivotRatio;
    public static final double countsPerDegree = countsPerArmRev/360;
}
public static final class elevatorSetpoints{
    public static final class armSetpoints {

        public static final int lowChamber = 0;
        public static final int highChamber = 1000;
        public static final int lowBasket = 800;
        public static final int highBasket = 2100;
        public static final int middle = 1000;
    }
    public static final class pivotSetpoints {

        public static final int middle = 100;
        public static final int chamber = 1000;
        public static final int basket = 1200;
        public static final int backStop = 1600;
    }


}
public static final class wristPoints{
    public static final double pickUp = 0.43;
    public static final double stow = 0.68;
    public static final double basket = 0.4;
    public static final double specimen = .35;
}

        }