package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.leftStartPos;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.armStart;
import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.armScoreCMD;
import org.firstinspires.ftc.teamcode.commands.armUpdateCMD;
import org.firstinspires.ftc.teamcode.commands.clawCloseCMD;
import org.firstinspires.ftc.teamcode.commands.clawOpenCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.commands.parkCMD;
import org.firstinspires.ftc.teamcode.commands.wristCMD;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.storage;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

@Autonomous(name = "left - 2 Sample",group = "Linear OpMode",preselectTeleOp = "mainOpMode")
public class left2Sample extends CommandOpMode {
    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private Claw claw = new Claw();
    private Wrist wrist = new Wrist();
    private boolean held = false;
    //private Thread armThread= new Thread(arm);


    public void init1(){

    }

@Override
    public void initialize() {
    CommandScheduler.getInstance().reset();
    robot.init(hardwareMap);
    drive.setPos(leftStartPos);


    armStart.reset();


    robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    while (opModeInInit() && globals.hardwareInit) {
        telemetry.addData("status: ", "ready");
        armStart.start();
        telemetry.update();
    }
    //armStart.stop();
    robot.eMotors.resetEncoder();
    robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    arm.setSetPoint(0, 0);
    //arm.startThread();
    //  armThread.setName("armThread");
    //armThread.start();
    CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                    new WaitCommand(500),
                    new driveCMD(drive, constants.autoGetPoints.basket),

                    new SequentialCommandGroup(
                            new armMoveCMD(arm, wrist, globals.armVal.HIGH_BASKET),
                            new clawOpenCMD(claw),
                            new wristCMD(wrist, globals.armVal.STOW),
                            new WaitCommand(300)
                    ),

                    new armMoveCMD(arm, wrist, globals.armVal.STOW),
                    new driveCMD(drive, constants.autoGetPoints.sample3).alongWith(new armMoveCMD(arm,wrist,globals.armVal.STOW)),
                    new WaitCommand(250),
                    new armMoveCMD(arm, wrist, globals.armVal.SAMPLE3PICKUP),
                    new WaitCommand(500),
                    new clawCloseCMD(claw),
                    new WaitCommand(250),
                    new armMoveCMD(arm, wrist, globals.armVal.HIGH_STOW),
                    new driveCMD(drive, constants.autoGetPoints.basket),

                    new SequentialCommandGroup(
                            new armMoveCMD(arm, wrist, globals.armVal.HIGH_BASKET),
                            new clawOpenCMD(claw),
                            new wristCMD(wrist, globals.armVal.STOW),
                            new WaitCommand(300)
                    ),
                    new armMoveCMD(arm, wrist, globals.armVal.STOW)

            ));
}
@Override
        public void run(){
            arm.update();
            CommandScheduler.getInstance().run();



            telemetry.addData("pivot pos", robot.pivotMotor.getCurrentPosition());
            telemetry.addData("arm pos", robot.eMotors.getPosition());
            telemetry.addData("robot pos", drive.getPos());
            telemetry.addData("arm done",arm.isDone());
            telemetry.addData("pivot done",arm.pivotDone());
            telemetry.addData("slide done",arm.armDone());
            telemetry.addData("arm case",arm.getState());
            telemetry.addData("elevator power",arm.getElevatorPower());
            telemetry.update();
            if(robot.pivotLimit.isPressed()&&!held){
                robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                held = true;
            }
            if(!robot.pivotLimit.isPressed()){
                held = false;
            }




        }


    }

