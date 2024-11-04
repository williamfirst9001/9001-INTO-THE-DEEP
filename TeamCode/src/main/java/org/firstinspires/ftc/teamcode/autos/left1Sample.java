package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.basket;
import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.leftStartPos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.armStart;
import org.firstinspires.ftc.teamcode.commands.armCheckCMD;
import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.autoArmMoveCMD;

import org.firstinspires.ftc.teamcode.commands.clawOpenCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.commands.highBasketCMD;
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
public class left1Sample extends LinearOpMode {
    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private Claw grabber = new Claw();
    private Wrist wrist = new Wrist();


    public void runOpMode(){
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap);
        drive.setPos(leftStartPos);
        CommandScheduler.getInstance().registerSubsystem(arm);
        //CommandScheduler.getInstance().setDefaultCommand(arm,new autoArmStowCMD(arm,0,0));








        while(!opModeIsActive() && globals.hardwareInit){
           armStart.start();
            telemetry.addData("status: ","ready");
            telemetry.update();
        }
        robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //stow and move to place pos
                        new ParallelDeadlineGroup(
                        new driveCMD(drive,basket),
                                new armMoveCMD(arm,0,0)
                                ),
                                new highBasketCMD(arm,wrist),
                                new armCheckCMD(arm),
                        new ParallelDeadlineGroup(
                                new clawOpenCMD(grabber),
                        new armMoveCMD(arm,constants.elevatorSetpoints.armSetpoints.highBasket,constants.elevatorSetpoints.pivotSetpoints.basket)
                                ),
                        new ParallelRaceGroup(
                        new armMoveCMD(arm,wrist,0,0,constants.wristPoints.stow),
                                new armCheckCMD(arm)
                                ),
                        new ParallelDeadlineGroup(
                        //stow
                        //park
                                new driveCMD(drive,new Pose2d(40,-35,Math.toRadians(90))),
                                new armMoveCMD(arm,wrist,0,0,constants.wristPoints.stow)

                                ),


                        new parkCMD(drive)



                ));
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("pivot pos",robot.pivotMotor.getCurrentPosition());
            telemetry.addData("arm pos",robot.elevatorMotor.getCurrentPosition());
            telemetry.update();

        }
        storage.currentPose = drive.getPos();

    }




}
