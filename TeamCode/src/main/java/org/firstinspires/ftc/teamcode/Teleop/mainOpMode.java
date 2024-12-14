package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.armStart;
import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
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
import org.tensorflow.*;



@TeleOp(name = "threaded mainOpMode", group = "Linear OpMode")
public class mainOpMode extends CommandOpMode {

    private elevator arm = new elevator();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private Claw claw = new Claw();
    private Wrist wrist = new Wrist();
    private GamepadEx driverOp;
    private GamepadEx controlOp;
    //private Thread armThread= new Thread(arm);
    private Vector2d input;
    private boolean sniper = false;
    private boolean forceRun = false;

    



    @Override
    public void initialize() {


        robot.init(hardwareMap);
        CommandScheduler.getInstance().reset();



        driverOp = new GamepadEx(gamepad1);
        controlOp = new GamepadEx(gamepad2);

        controlOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new armMoveCMD(arm,wrist, globals.armVal.LOW_BASKET),true);
        //high basket
        controlOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new armMoveCMD(arm,wrist, globals.armVal.HIGH_BASKET),true);

        controlOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenPressed(new armMoveCMD(arm,wrist, globals.armVal.STOW));

        controlOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new wristCMD(wrist,globals.armVal.STOW));

        controlOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new armMoveCMD(arm,wrist,globals.armVal.PICKUPHIGH));
        controlOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new armMoveCMD(arm,wrist,globals.armVal.PICKUPLOW));

        controlOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new clawCloseCMD(claw));
        controlOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new clawOpenCMD(claw));



       



        armStart.reset();
        drive.setPos(new Pose2d(-10, -62, Math.toRadians(90)));

        while(!opModeIsActive() && globals.hardwareInit){
            armStart.start();
            telemetry.addData("status","ready");
            telemetry.update();
        }
        armStart.stop();
        robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.startThread();
        //armThread.setName("armThread");
        //armThread.start();
    }


    @Override
    public void run() {
        arm.update();



            Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
            long runStartTime = System.nanoTime();
            long CSStartTime = System.nanoTime();
            CommandScheduler.getInstance().run();
            long CSEndTime = System.nanoTime();
            long CSDuration = (CSEndTime - CSStartTime) / 1000000;  //divide by 1000000 to get milliseconds.
            controlOp.readButtons();
            driverOp.readButtons();
            long armStartTime = System.nanoTime();

            long armEndTime = System.nanoTime();
            long armDuration = (armEndTime - armStartTime) / 1000000;

            long bodyStartTime = System.nanoTime();
            input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );


// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately
            sniper = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 || robot.eMotors.getPosition() > 2000;

            if (sniper) {
                drive.setDriveMotorPower(
                        new Pose2d(
                                input.getX() / 4,
                                input.getY() / 4,
                                -gamepad1.right_stick_x / 4
                        )
                );

            } else {
                drive.setDriveMotorPower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x / 2
                        )
                );

            }

            //low basket

            if (controlOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1 ) {
                schedule(new armMoveCMD(arm, wrist, globals.armVal.HIGH_STOW));
            }


            if (controlOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
                CommandScheduler.getInstance().schedule(new wristCMD(wrist, constants.wristPoints.pickUp));
            }

            if (Math.abs(controlOp.getLeftY()) > .1) {
                arm.setPivotPoint(arm.getPivotSetPoint() + controlOp.getLeftY() * 20);
            }


            if (Math.abs(controlOp.getRightY()) > .1) {
                arm.setArmPoint(arm.getArmSetPoint() - controlOp.getRightY() * 30);
            }
            /**
            if(robot.pivotLimit.isPressed()){
                robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
             */


            long bodyEndTime = System.nanoTime();
            long bodyDuration = (bodyEndTime - bodyStartTime) / 1000000;
            long driveStartTime = System.nanoTime();
            drive.update();
            long driveEndTime = System.nanoTime();
            long driveDuration = (driveEndTime - driveStartTime) / 1000000;
            long runEndTime = System.nanoTime();
            long runDuration = (runEndTime - runStartTime) / 1000000;
            //telemetry.addData("Command Scheduler time", CSDuration);
            //telemetry.addData("arm time", armDuration);
            //telemetry.addData("body duration", bodyDuration);
            //telemetry.addData("drive duration", driveDuration);
            //telemetry.addData("run duration", runDuration);
            telemetry.addData("emotor position", robot.eMotors.getPosition());
            telemetry.addData("pivot position",robot.pivotMotor.getCurrentPosition());
        telemetry.addData("arm done",arm.isDone());
        telemetry.addData("pivot done",arm.pivotDone());
        telemetry.addData("slide done",arm.armDone());
        telemetry.addData("arm case",arm.getState());
            telemetry.update();
            if (isStopRequested()) {
                //armThread.interrupt();
                arm.endThread();
            }

        }



}
