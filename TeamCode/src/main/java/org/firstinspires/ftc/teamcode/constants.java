package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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
    public static final Pose2d basket = new Pose2d(-56.5,-54.5,Math.toRadians(225));
    public static final Pose2d leftChamber = new Pose2d(-10,-35,Math.toRadians(90));
    public static final Pose2d rightChamber = new Pose2d(10,-35,Math.toRadians(90));
    public static final Pose2d leftStartPos = new Pose2d(-10, -62, Math.toRadians(90));
    public static final Pose2d rightStartPos = new Pose2d(10, -62, Math.toRadians(90));
}

public static final class clawPoints{
    public static final double closePos = 0.3;
    public static final double openPos = .6;
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
        public static double P = 0.005;
        public static double I = 0.01;
        public static double D = 0.0000;
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


        public static double P = 0.007;
        public static double I = 0.002;
        public static double D = 0.000;
        public static final double pivotRange = 120;
        public static final double degPerCount = 360/(28*15*40/15.0);


}
public static final class aprilTagPos{

}
public static final class armGearRatio{
    public static final double ticksPerRevolution = 28;
    public static final double motorGearRatio = 100;
    public static final double motorToPivotRatio = 40.0/15.0;
    public static final double countsPerArmRev = ticksPerRevolution*motorGearRatio*motorToPivotRatio;
    public static final double countsPerDegree = countsPerArmRev/360;
}


public static final class points{
    public static final List<Double> highBasket = Arrays.asList(2200.0,2100.0,.9);
    public static final List<Double> lowBasket = Arrays.asList(800.0,2100.0,.8);
    public static final List<Double> highChamber = Arrays.asList(1000.0,1000.0,.8);
    public static final List<Double> lowChamber = Arrays.asList(0.0,1000.0,.4);
    public static final List<Double> stow = Arrays.asList(0.0,0.0,.2);
    public static final List<Double> pickUp = Arrays.asList(1000.0,600.0,.9);
    public static final List<Double> sample3PickUp = Arrays.asList(1000.0,600.0,.8);
    public static final Map<globals.armVal, List<Double>> map = Stream.of(new Object[][]

    {
        {
            globals.armVal.HIGH_BASKET, highBasket
        },
        {
            globals.armVal.LOW_BASKET, lowBasket
        },
            {
            globals.armVal.STOW, stow
            },
            {
                globals.armVal.PICKUP,pickUp
            },
            {
                globals.armVal.SAMPLE3PICKUP,sample3PickUp
            }
    }).collect(Collectors.toMap(
                data -> (globals.armVal) data[0],
                data -> (List<Double>) data[1]
        ));
    };

public static final class wristPoints{
    public static final double pickUp = 1;
    public static final double stow = 0;
    public static final double basket = .8;
    public static final double specimen = .35;
}

        }