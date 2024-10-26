package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.armStart;
import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.clawCloseCMD;
import org.firstinspires.ftc.teamcode.commands.clawOpenCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.commands.highBasketCMD;
import org.firstinspires.ftc.teamcode.commands.highChamberCMD;
import org.firstinspires.ftc.teamcode.commands.lowBasketCMD;
import org.firstinspires.ftc.teamcode.commands.lowChamberCMD;
import org.firstinspires.ftc.teamcode.commands.scoreChamberCMD;
import org.firstinspires.ftc.teamcode.commands.stowCMD;
import org.firstinspires.ftc.teamcode.commands.wristCMD;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.storage;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;
import static org.firstinspires.ftc.teamcode.constants.elevatorSetpoints.*;


@TeleOp(name = "mainOpMode", group = "Linear OpMode")
public class mainOpMode extends CommandOpMode {

    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private Claw claw = new Claw();
    private Wrist wrist = new Wrist();
    private GamepadEx driverOp;
    private GamepadEx controlOp;
    private boolean FOD = true;
    private Vector2d input;
    private double manArmP = 0;
    private double manPivP = 0;

//TODO: add the position storage back
    @Override
    public void initialize() {
        armStart.reset();


        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(arm);

        driverOp = new GamepadEx(gamepad1);
        controlOp = new GamepadEx(gamepad2);
        robot.init(hardwareMap);

       // CommandScheduler.getInstance().setDefaultCommand(arm, new stowCMD(arm));
        //drive.setPos(storage.currentPose);
        drive.setPos(new Pose2d(-10, -62, Math.toRadians(90)));
        while(!opModeIsActive() && globals.hardwareInit){
            armStart.start();
            telemetry.addData("status","ready");
            telemetry.update();
        }
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        controlOp.readButtons();
        driverOp.readButtons();




        // Read pose
        Pose2d poseEstimate = drive.getPos();

        //telemetry.addData("heading",Math.toDegrees(poseEstimate.getHeading()));

// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
        if(FOD) {
            input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading()+Math.toRadians(90));
        } else {
            input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );
        }

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately
        if(driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0 ){
            drive.setDriveMotorPower(
                    new Pose2d(
                            input.getX()/4,
                            input.getY()/4,
                            -gamepad1.right_stick_x/4
                    )
            );
        } else{
            drive.setDriveMotorPower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x/2
                    )
            );
        }




        controlOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new armMoveCMD(arm,wrist,armSetpoints.lowBasket,pivotSetpoints.basket, constants.wristPoints.basket),true);
        controlOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new armMoveCMD(arm,wrist,armSetpoints.highBasket,pivotSetpoints.basket,constants.wristPoints.basket),true);
        controlOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new armMoveCMD(arm,wrist,armSetpoints.highChamber,pivotSetpoints.chamber,constants.wristPoints.specimen),true);
        controlOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new armMoveCMD(arm,wrist,armSetpoints.lowChamber,pivotSetpoints.chamber,constants.wristPoints.specimen),true);
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new driveCMD(drive,new Pose2d(-10, -62, Math.toRadians(90))));
        if(controlOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            manArmP = 0;
            manPivP = 0;
            schedule(new stowCMD(arm,wrist));
        }
        if(controlOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0){
            schedule(new scoreChamberCMD(arm,claw));
        }
        controlOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new wristCMD(wrist,constants.wristPoints.pickUp));
        controlOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new armMoveCMD(arm,1000,300));
        controlOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new armMoveCMD(arm,1000,225));

        if(controlOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0){
            CommandScheduler.getInstance().schedule(new wristCMD(wrist,constants.wristPoints.stow));
        }
        controlOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new clawCloseCMD(claw));
        controlOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new clawOpenCMD(claw));
        if(Math.abs(controlOp.getLeftY())>.1){
            robot.pivotMotor.setPower(controlOp.getLeftY());
            manPivP = 1;
        }else if(manPivP == 1){
            robot.pivotMotor.setPower(0);
            manPivP = 0;
        }

        if(Math.abs(controlOp.getRightY())>.1){
            robot.elevatorMotor.setPower(-controlOp.getRightY());
            manArmP = 1;
        } else if(manArmP == 1){
            robot.elevatorMotor.setPower(0);
            manArmP = 0;
        }

        if(driverOp.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)){
                FOD = !FOD;
        }
        telemetry.addData("right stick y",controlOp.getRightY());
        drive.update();
        telemetry.update();
    }


}
