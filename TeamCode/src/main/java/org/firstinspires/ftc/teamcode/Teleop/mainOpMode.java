package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

@TeleOp(name = "mainOpMode", group = "Linear OpMode")
public class mainOpMode extends CommandOpMode {

    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private GamepadEx driverOp;
    private GamepadEx controlOp;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        driverOp = new GamepadEx(gamepad1);
        controlOp = new GamepadEx(gamepad2);
        robot.init(hardwareMap);
        CommandScheduler.getInstance().setDefaultCommand(arm, new stowCMD(arm));
        while(!opModeIsActive() && globals.hardwareInit){
            telemetry.addData("status: ","ready");
            telemetry.update();
        }

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();


        // Read pose
        Pose2d poseEstimate = drive.getPos();

// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately
        drive.setDriveMotorPower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );



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
