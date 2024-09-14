package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public abstract class Utils extends LinearOpMode {
    public double toRads(double degs){
        return Math.toRadians(degs);
    }
    public WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam1");
    public DcMotorEx elevatorMotor = hardwareMap.get(DcMotorEx.class,"elevator");
}
