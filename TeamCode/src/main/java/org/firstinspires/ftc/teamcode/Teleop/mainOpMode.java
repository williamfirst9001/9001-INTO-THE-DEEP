package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.highBasketCMD;
import org.firstinspires.ftc.teamcode.commands.highChamberCMD;
import org.firstinspires.ftc.teamcode.commands.lowBasketCMD;
import org.firstinspires.ftc.teamcode.commands.lowChamberCMD;
import org.firstinspires.ftc.teamcode.commands.stowCMD;
import org.firstinspires.ftc.teamcode.roadRunner.startup.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotContainer;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

@TeleOp(name = "mainOpMode", group = "Linear OpMode")
public class mainOpMode extends CommandOpMode {

    private elevator arm = new elevator();
    //private SampleMecanumDrive drive;
    private robotContainer robot = robotContainer.getInstance();
    private GamepadEx driverOp;
    private GamepadEx controlOp;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        driverOp = new GamepadEx(gamepad1);
        controlOp = new GamepadEx(gamepad2);
        robot.init(hardwareMap);
        CommandScheduler.getInstance().setDefaultCommand(arm, new stowCMD(arm));
        //drive = robot.drive;

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

/*
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y/2,
                        -gamepad1.left_stick_x/2,
                        -gamepad1.right_stick_y/2
                )
        );
        */


        controlOp.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new lowBasketCMD(arm));
        controlOp.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new highBasketCMD(arm));
        controlOp.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(new highChamberCMD(arm));
        controlOp.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new lowChamberCMD(arm));
        //drive.update();
    }


}
