package com.example.meepmeeptesting;


import com.example.meepmeeptesting.trajectories.redBasket3;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.Constraints;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

    public class MeepMeepTesting {


        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(800);
            Constraints constraints = new Constraints(60,60,Math.toRadians(180),Math.toRadians(180),14);

            redBasket3 auto = new redBasket3();
            RoadRunnerBotEntity myBot = new RoadRunnerBotEntity(new MeepMeep(800),constraints,14,14,auto.start,new ColorSchemeRedDark(),.95, DriveTrainType.MECANUM,true);
            myBot.followTrajectorySequence(auto.traj);





            //RoadRunnerBotEntity myBot = new RoadRunnerBotEntity(meepMeep,constraints,15,18,null,null,1, DriveTrainType.MECANUM,true);
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width


            meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
