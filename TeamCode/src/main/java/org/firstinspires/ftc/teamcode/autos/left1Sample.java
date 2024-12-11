package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.leftStartPos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.armStart;
import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.armScoreCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.commands.parkCMD;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.storage;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

@Autonomous(name = "left - 1 Sample",group = "Linear OpMode",preselectTeleOp = "mainOpMode")
public class left1Sample extends CommandOpMode {
    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private Claw claw = new Claw();
    private Wrist wrist = new Wrist();
   // private Thread armThread= new Thread(arm);

    public void initialize(){
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap);
        drive.setPos(leftStartPos);
        CommandScheduler.getInstance().registerSubsystem(arm);


        while(!opModeIsActive() && globals.hardwareInit){
            armStart.start();
            telemetry.addData("status","ready");
            telemetry.update();
        }


        robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        arm.startThread();
        //armThread.setName("armThread");
        //armThread.start();
        schedule(
                new SequentialCommandGroup(
                        new driveCMD(drive, constants.autoGetPoints.basket),
                        new armScoreCMD(arm, wrist, claw, globals.armVal.HIGH_BASKET),

                        new armMoveCMD(arm, wrist, globals.armVal.STOW),

                        new parkCMD(drive)


                ));
    }

@Override
    public void run() {


        CommandScheduler.getInstance().run();


            telemetry.addData("pivot pos", robot.pivotMotor.getCurrentPosition());
            telemetry.addData("arm pos", robot.eMotors.getPosition());
            telemetry.addData("robot pos", drive.getPos());
            telemetry.update();



        storage.currentPose = drive.getPos();
        if (isStopRequested()) {
           // armThread.interrupt();
            arm.endThread();
        }


    }
}
