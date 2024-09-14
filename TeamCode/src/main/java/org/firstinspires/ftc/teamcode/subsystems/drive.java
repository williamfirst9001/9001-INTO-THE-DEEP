package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadRunner.startup.drive.SampleMecanumDrive;

public class drive {
    private SampleMecanumDrive robot;
    private limeLight limelight;
    public drive(HardwareMap hardwareMap){
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        this.robot = robot;
    }
    public void goToAprilTag(){
        LLResult results= limelight.getFidResults();

        double xError =0;
        double yError = 0;
        double turnError = 0;
        double distance_MM = 0;
        

    }

}
