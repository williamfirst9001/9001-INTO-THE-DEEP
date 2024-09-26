package com.example.meepmeeptesting.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.Constraints;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;


public class redBasket3 {
    private Constraints constraints = new Constraints(60,60,Math.toRadians(180),Math.toRadians(180),14);
    private MeepMeep meepMeep = new MeepMeep(800);
    public RoadRunnerBotEntity bot;
    public TrajectorySequence traj;
    public Pose2d start = new Pose2d(-36,-60,Math.toRadians(90));
    public redBasket3(){
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(constraints)
                .setDimensions(14,14)

                //.trajectorySequenceBuilder(new Pose2d(-36,-60,Math.toRadians(90)))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(new Pose2d(-50,-47,Math.toRadians(225)))
                .addDisplacementMarker(()->{
                    //place in high basket
                })
                .lineToLinearHeading(new Pose2d(-47.5,-47,Math.toRadians(90)))
                .addDisplacementMarker(()->{
                    //get new sample
                })
                .lineToLinearHeading(new Pose2d(-50,-47,Math.toRadians(225)))
                .addDisplacementMarker(()->{
                    //place in high basket
                })
                .lineToLinearHeading(new Pose2d(-58,-47,Math.toRadians(90)))
                .addDisplacementMarker(()->{
                    //get new sample
                })
                .lineToLinearHeading(new Pose2d(-62,-47,Math.toRadians(270)))
                .addDisplacementMarker(()->{
                    //place in high basket
                })
                .lineToLinearHeading(new Pose2d(-57,-47,Math.toRadians(120)))
                .addDisplacementMarker(()->{
                    //get new sample
                })
                .lineToLinearHeading(new Pose2d(-50,-47,Math.toRadians(225)))
                .addDisplacementMarker(()->{
                    //place in high basket
                })
                .lineToLinearHeading(new Pose2d(40,-62,Math.toRadians(90)))
                    .build());

        this.bot = myBot;
        this.traj = myBot.getCurrentTrajectorySequence();
        this.start = start;

    }
}
