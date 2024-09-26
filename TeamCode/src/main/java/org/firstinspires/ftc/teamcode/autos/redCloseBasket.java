package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadRunner.startup.drive.SampleMecanumDrive;

public class redCloseBasket extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36,-57,0));
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
          //              .linetoLinearHeading(new Pose2d)
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
