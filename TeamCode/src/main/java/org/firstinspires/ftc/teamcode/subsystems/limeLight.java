package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.robotHardware;

import java.util.HashMap;
import java.util.List;

public class limeLight extends SubsystemBase{
    robotHardware robot = robotHardware.getInstance();

    private double strafeDistance_3D;
    private double forwardDistance_3D;
    private double yawDistance_3D;
    private LLResult result = null;
    public limeLight(){

    }
    public void periodic(){
        result = robot.limelight.getLatestResult();
    }
    /**returns the pos of the robot based on april tags **/
    public Pose2d getFieldPos(){

        if (result != null) {
            if (result.isValid()) {
                List<FiducialResult> fiducials = result.getFiducialResults();
                for (FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    double x = fiducial.getRobotPoseFieldSpace().getPosition().x * 39.3701; // Where it is (left-right)
                    double y = fiducial.getRobotPoseFieldSpace().getPosition().y* 39.3701; // Where it is (up-down)
                    double yaw = fiducial.getRobotPoseTargetSpace().getOrientation().getYaw();
                    return new Pose2d(x,y,yaw);
                }

            }
        }
        return null;
    }
    public Pose2d getDetectPos(){
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.ColorResult> detects = result.getColorResults();
                for (LLResultTypes.ColorResult detect : detects) {
                    double x = detect.getTargetXDegrees();
                    double y = detect.getTargetYDegrees();
                    return new Pose2d(x,y,0);
                }

            }
        }
        return null;
    }
    public void setPipeLine(int line){
        robot.limelight.pipelineSwitch(line);
    }

    public void setRobotPos(){

    }

    public Pose2d botPoseToPose(Pose3D pos){

        return new Pose2d(pos.getPosition().x,pos.getPosition().y,Math.toRadians(pos.getOrientation().getYaw()));


    }

}
