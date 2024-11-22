package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.example.meepmeeptesting.trajectories.redBasket3;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.Constraints;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

    public class MeepMeepTesting {


        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(800);
            Constraints constraints = new Constraints(60,60,Math.toRadians(180),Math.toRadians(180),14);

            redBasket3 auto = new redBasket3();
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(47, 40, Math.toRadians(227), Math.toRadians(240), 14)
                    .setDimensions(14,14)
                    .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(18, -62, Math.toRadians(90)))
                            .lineToLinearHeading(new Pose2d(34,-45,Math.toRadians(90)))
                            .lineToLinearHeading(new Pose2d(37,0,Math.toRadians(90)))
                            .lineToLinearHeading(new Pose2d(47.5,5,Math.toRadians(90)))
                            .lineToLinearHeading(new Pose2d(47.5,-60,Math.toRadians(90)))
                            .lineToLinearHeading(new Pose2d(47.5,5,Math.toRadians(90)))
                            .lineToLinearHeading(new Pose2d(57.5,5,Math.toRadians(90)))
                            .lineToLinearHeading(new Pose2d(57.5,-60,Math.toRadians(90)))
                            .build());





            //RoadRunnerBotEntity myBot = new RoadRunnerBotEntity(meepMeep,constraints,15,18,null,null,1, DriveTrainType.MECANUM,true);
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width


            meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
