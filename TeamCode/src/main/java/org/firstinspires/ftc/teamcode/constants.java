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
    public static final double maxExtensionRange =970;
}
public static final class armPID{
    public static final class middle{
        public static final double kP = .01;
        public static final double kI = .01;
        public static final double kD = .01;
    }
    public static final class chambers{
        public static final double kP = 1;
        public static final double kI = 1;
        public static final double kD = 1;
    }
    public static final class baskets{
        public static final double kP = 1;
        public static final double kI = 1;
        public static final double kD = 1;
    }
    public static final double tolerance = 0;


}

public static final class pivotThreshold{
    public static final double low = 100;
    public static final double mid = 200;
    public static final double high = 300;
}
public static final class pivotPID{

        public static final double kP = 1;
        public static final double kI = 1;
        public static final double kD = 1;
        public static final double tolerance = 0;

}
public static final class armGearRatio{
    public static final double ticksPerRevolution = 28;
    public static final double motorGearRatio = 20;
    public static final double motorToPivotRatio = 5;
    public static final double countsPerArmRev = ticksPerRevolution*motorGearRatio*motorToPivotRatio;
    public static final double countsPerDegree = countsPerArmRev/360;
}
public static final class elevatorSetpoints{
    public static final class armSetpoints {

        public static final int lowChamber = 100;
        public static final int highChamber = 200;
        public static final int lowBasket = 150;
        public static final int highBasket = 150;
        public static final boolean middle = false;
    }
    public static final class pivotSetpoints {

        public static final int middle = 0;
        public static final int chamber = 200;
        public static final int basket = 150;
    }


}

        }