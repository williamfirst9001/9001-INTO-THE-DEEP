package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadRunner.startup.drive.SampleMecanumDrive;


@Config
public class robotHardware extends Robot {
    public DcMotorEx elevatorMotor;
    public DcMotorEx pivotMotor;
    public DcMotorEx leftFront,leftRear,rightRear,rightFront;
    public SampleMecanumDrive drive;
    private HardwareMap hardwareMap;
    private static robotHardware instance = null;
    private boolean enabled;

    public static robotHardware getInstance(){
        if (instance == null) {
            instance = new robotHardware();
        }
        instance.enabled = true;
        return instance;
    }
//TODO fix the robot not intiliazing

    public void init(HardwareMap hardwareMap) {
        this.drive = new SampleMecanumDrive(hardwareMap) ;
        this.hardwareMap = hardwareMap;
        this.elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevator");
        this.pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        ///acded


        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(constants.startPos);


    }

}
