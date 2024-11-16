package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.roadRunner.startup.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.elevatorMotors;


@Config
public class robotHardware extends Robot {
    private DcMotorEx left,right;
    public DcMotorEx pivotMotor;
    public SampleMecanumDrive drive;
    public Servo claw;
    private HardwareMap hardwareMap;
    private static robotHardware instance = null;
    public Servo wristServo;
    private boolean enabled;
    public TouchSensor armSwitch;
    public TouchSensor pivotLimit;
    public Limelight3A limelight;
    public IMU imu;
    public elevatorMotors eMotors;

    //public Blinker led;
    public VoltageSensor voltageSensor;

    public static robotHardware getInstance(){
        if (instance == null) {
            instance = new robotHardware();
        }
        instance.enabled = true;
        return instance;
    }


    public void init(HardwareMap hardwareMap) {
        globals.hardwareInit = false;
        imu = hardwareMap.get(IMU.class, "imu");
        this.eMotors = new elevatorMotors(left,right);
        this.drive = new SampleMecanumDrive(hardwareMap) ;
        this.hardwareMap = hardwareMap;
        this.pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.armSwitch = hardwareMap.get(TouchSensor.class, "slideswitch");
        this.pivotLimit = hardwareMap.get(TouchSensor.class, "pivotlimit");
        this.wristServo = hardwareMap.get(Servo.class, "wristservo");
        this.limelight = hardwareMap.get(Limelight3A.class,"limeLight");

        claw.setPosition(constants.clawPoints.closePos);
        eMotors.init(hardwareMap);



        //this.led = hardwareMap.get(Blinker.class,"led");


        eMotors.resetEncoder();
        eMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        //imu.initialize(new IMU.Parameters(orientationOnRobot));



        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(constants.startPos);
        claw.setPosition(constants.clawPoints.closePos);

        //limelight.pipelineSwitch(0);
        //limelight.start();

        globals.hardwareInit = true;
        globals.manualArm = false;





    }

}
