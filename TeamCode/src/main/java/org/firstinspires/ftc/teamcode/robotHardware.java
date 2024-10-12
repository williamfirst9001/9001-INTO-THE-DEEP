package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadRunner.startup.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.elevator;


@Config
public class robotHardware extends Robot {
    public DcMotorEx elevatorMotor;
    public DcMotorEx pivotMotor;
    public DcMotorEx leftFront,leftRear,rightRear,rightFront;
    public SampleMecanumDrive drive;
    public Servo claw;
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


    public void init(HardwareMap hardwareMap) {
        this.drive = new SampleMecanumDrive(hardwareMap) ;
        this.hardwareMap = hardwareMap;
        this.elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevator");
        this.pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        this.claw = hardwareMap.get(Servo.class, "claw");

        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(constants.startPos);
        claw.setPosition(constants.clawPoints.closePos);

        globals.hardwareInit = true;



    }

}
