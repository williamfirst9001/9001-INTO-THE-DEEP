package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robotHardware;

public class driveBase extends SubsystemBase {
    private robotHardware robot = robotHardware.getInstance();
    public driveBase(){

    }
    public void goToPos(Pose2d pos){
        Trajectory traj = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                        .lineToLinearHeading(pos)
                        .build();
        robot.drive.followTrajectory(traj);


    }
    public void setDriveMotorPower(Pose2d power){

        robot.drive.setWeightedDrivePower(power);
    }
    public void move(Pose2d pos){
        robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(new Pose2d())
                        .lineToLinearHeading(pos)
                        .build()
        );
    }
    public void turn(double ang){
        robot.drive.turn(Math.toRadians(ang));
    }
    public Pose2d getPos(){
        return robot.drive.getPoseEstimate();
    }
    public void update(){
        robot.drive.update();
    }
    public void setPos(Pose2d pos){
        robot.drive.setPoseEstimate(pos);
    }

}
