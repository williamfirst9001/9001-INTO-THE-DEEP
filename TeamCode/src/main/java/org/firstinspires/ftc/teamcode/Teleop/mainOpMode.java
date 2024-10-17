package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.clawCloseCMD;
import org.firstinspires.ftc.teamcode.commands.clawOpenCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.commands.highBasketCMD;
import org.firstinspires.ftc.teamcode.commands.highChamberCMD;
import org.firstinspires.ftc.teamcode.commands.lowBasketCMD;
import org.firstinspires.ftc.teamcode.commands.lowChamberCMD;
import org.firstinspires.ftc.teamcode.commands.stowCMD;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.claw;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

@TeleOp(name = "mainOpMode", group = "Linear OpMode")
public class mainOpMode extends CommandOpMode {

    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private claw grabber = new claw();
    private GamepadEx driverOp;
    private GamepadEx controlOp;
    private boolean toggleClawBoolean = false;


    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        driverOp = new GamepadEx(gamepad1);
        controlOp = new GamepadEx(gamepad2);
        robot.init(hardwareMap);
       // CommandScheduler.getInstance().setDefaultCommand(arm, new stowCMD(arm));
        //drive.setPos(poseStorage.currentPose);
        drive.setPos(new Pose2d(-10, -62, Math.toRadians(90)));
        while(!opModeIsActive() && globals.hardwareInit){
            telemetry.addData("status: ","ready");
            telemetry.update();
        }

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        controlOp.readButtons();




        // Read pose
        Pose2d poseEstimate = drive.getPos();
        //telemetry.addData("heading",Math.toDegrees(poseEstimate.getHeading()));

// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading()+90);

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
                .whenPressed(new lowBasketCMD(arm),true);
        controlOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new highBasketCMD(arm),true);
        controlOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new highChamberCMD(arm),true);
        controlOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new lowChamberCMD(arm),true);
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenPressed(new driveCMD(drive,basket));
        controlOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new stowCMD(arm));
        if(controlOp.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)){
            if(robot.claw.getPosition()== constants.clawPoints.openPos){
                CommandScheduler.getInstance().schedule(new clawCloseCMD(grabber));
            } else {
                CommandScheduler.getInstance().schedule(new clawOpenCMD(grabber));
            }
        }
        drive.update();
        telemetry.update();
    }


}
