package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.roadRunner.startup.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive;


@Config
public class robotContainer extends Robot {
    public DcMotorEx elevatorMotor;
    public DcMotorEx pivotMotor;
    //public SampleMecanumDrive drive;
    private HardwareMap hardwareMap;
    private static robotContainer instance = null;
    private boolean enabled;

    public static robotContainer getInstance(){
        if (instance == null) {
            instance = new robotContainer();
        }
        instance.enabled = true;
        return instance;
    }


    public void init(HardwareMap hardwareMap) {
        //this.drive = drive;
        this.hardwareMap = hardwareMap;
        this.elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevator");
        this.pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

}
