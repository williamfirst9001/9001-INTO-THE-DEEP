package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.armStart;
import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.armScoreCMD;
import org.firstinspires.ftc.teamcode.commands.clawCloseCMD;
import org.firstinspires.ftc.teamcode.commands.clawOpenCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.commands.wristCMD;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;
import org.firstinspires.ftc.teamcode.subsystems.limeLight;

import java.util.function.Consumer;


@TeleOp(name = "mainOpMode", group = "Linear OpMode")
public class mainOpMode extends CommandOpMode {

    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private Claw claw = new Claw();
    private Wrist wrist = new Wrist();
    private limeLight limelight = new limeLight();
    private GamepadEx driverOp;
    private GamepadEx controlOp;
    private boolean FOD = true;
    private Vector2d input;
    private double manArmP = 0;
    private double manPivP = 0;
    private boolean sniper = false;
    private boolean hijack = false;


//TODO: add the position storage back
    @Override
    public void initialize() {


        CommandScheduler.getInstance().reset();


        driverOp = new GamepadEx(gamepad1);
        controlOp = new GamepadEx(gamepad2);
        arm.setState(elevator.armState.DOWNDOWN);


       // CommandScheduler.getInstance().setDefaultCommand(arm, new stowCMD(arm));
        //drive.setPos(storage.currentPose);


        robot.init(hardwareMap);
        arm.setSetPoint(constants.points.stow);
        drive.setPos(new Pose2d(-10, -62, Math.toRadians(90)));
       // CommandScheduler.getInstance().registerSubsystem(arm);
        arm.zero();
        while(!opModeIsActive() && globals.hardwareInit){
            arm.update();
        }

        robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void run() {
        long startTime = System.nanoTime();
        //robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CommandScheduler.getInstance().run();
        controlOp.readButtons();
        driverOp.readButtons();
        hijack = false;

        arm.update();



        Pose2d poseEstimate = drive.getPos();

        //telemetry.addData("heading",Math.toDegrees(poseEstimate.getHeading()));

// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
        /**if(FOD) {
            input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading()+Math.toRadians(90));
        } else {
         **/
            input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );

        sniper = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0;
        if(!hijack) {
            if  (sniper){
                drive.setDriveMotorPower(
                        new Pose2d(
                                input.getX() / 4,
                                input.getY() / 4,
                                -gamepad1.right_stick_x / 4
                        )
                );


            } else{
                drive.setDriveMotorPower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x / 2
                        )
                );

            }
        }
        sniper = robot.eMotors.getPosition() > 1200;
        //low basket
        controlOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new armMoveCMD(arm,wrist, globals.armVal.LOW_BASKET),true);
        //high basket
        controlOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new armMoveCMD(arm,wrist, globals.armVal.HIGH_BASKET),true);

        //high chamber
        //controlOp.getGamepadButton(GamepadKeys.Button.X)
                //whenPressed(new armMoveCMD(arm,wrist,armSetpoints.highChamber,pivotSetpoints.chamber,constants.wristPoints.specimen),true);
        //low chamber
        //controlOp.getGamepadButton(GamepadKeys.Button.A)
           //     .whenPressed(new armMoveCMD(arm,wrist,armSetpoints.lowChamber,pivotSetpoints.chamber,constants.wristPoints.specimen),true);

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new driveCMD(drive,new Pose2d(-10, -62, Math.toRadians(90))));
        if(controlOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            schedule(new armMoveCMD(arm,wrist, globals.armVal.STOW));
        }

        controlOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new armMoveCMD(arm,wrist,globals.armVal.PICKUP));



        controlOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new wristCMD(wrist,globals.armVal.PICKUP));


        if(controlOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0){
            CommandScheduler.getInstance().schedule(new wristCMD(wrist,globals.armVal.STOW));
        }
        controlOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new clawCloseCMD(claw));
        controlOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new clawOpenCMD(claw));
        if(Math.abs(controlOp.getLeftY())>.1){
            arm.setPivotPoint(arm.getPivotSetPoint()+controlOp.getLeftY()*20);
            arm.setState(elevator.armState.MANUAL);
        }

        

        if(Math.abs(controlOp.getRightY())>.1){
            arm.setArmPoint(arm.getArmSetPoint()-controlOp.getRightY()*30);
            arm.setState(elevator.armState.MANUAL);
        }



        if(driverOp.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)){
                FOD = !FOD;
        }
        telemetry.addData("pivot position",robot.pivotMotor.getCurrentPosition());
        telemetry.addData("armstate",arm.getarmState());
        telemetry.addData("left stick",controlOp.getLeftY());
        telemetry.addData("elevator setpoint",arm.getArmSetPoint());
        telemetry.addData("pivot motor runmode",robot.pivotMotor.getMode());
        telemetry.addData("pivot set point",arm.getPivotSetPoint());
        drive.update();
        long endTime = System.nanoTime();

        long duration = (endTime - startTime)/1000000;  //divide by 1000000 to get milliseconds.
        telemetry.addData("update duration",duration);
        telemetry.update();
    }
/**
    public boolean getHijack(){
        return Math.abs(robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES)) > 5;
    }
    public void stabilize(){
        if(getHijack()){
            if(robot.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES)>0){
                drive.setDriveMotorPower(new Pose2d(-.5));
            } else{
                drive.setDriveMotorPower(new Pose2d(.5));
            }
        }
 }
 **/


}
