package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.roadRunner.startup.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.startup.trajectorysequence.TrajectorySequence;

@Autonomous(name ="testAuto",group = "Concept")
public class testAuto extends LinearOpMode {

    boolean isDone = false;


@Override
   public void runOpMode(){
       SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
       drive.setPoseEstimate(constants.startPos);
       drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, 60, Math.toRadians(270)))
               .lineToLinearHeading(new Pose2d(11,57,Math.toRadians(270)))
               .lineToLinearHeading(new Pose2d(35,34,Math.toRadians(180)))

               .addDisplacementMarker(()->{
                   //purple pixel code
               })
               .lineToLinearHeading(new Pose2d(41,37,Math.toRadians(0)))
               .lineToLinearHeading(new Pose2d(49,41,Math.toRadians(0)))
               .addDisplacementMarker(()->{
                   //yellow pixel
               })
               .lineToLinearHeading(new Pose2d(35,14,Math.toRadians(180)))
               .lineToLinearHeading(new Pose2d(-56,11,Math.toRadians(180)))
               .addDisplacementMarker(()->{
                   //2 white pixels
               })
               .lineToLinearHeading(new Pose2d(35,13,Math.toRadians(0)))
               .lineToLinearHeading(new Pose2d(49,28,Math.toRadians(0)))
               .addDisplacementMarker(()->{
                   //place 2 white pixels
               })
               .lineToLinearHeading(new Pose2d(49,58,Math.toRadians(0)))

               //.splineTo(new Vector2d(59,59),Math.toRadians(0))

               .build();
       TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(-15,0,Math.toRadians(90)))

               .lineToLinearHeading(new Pose2d(0,15,Math.toRadians(270)))
               .lineToLinearHeading(new Pose2d(15,0,Math.toRadians(180)))
               .lineToLinearHeading(new Pose2d(0,-15,Math.toRadians(90)))
               .lineToLinearHeading(new Pose2d(-15,0,Math.toRadians(0)))




               .build();



       waitForStart();
           drive.followTrajectorySequence(traj2);


   }
}
