package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.armStart;
import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.clawCloseCMD;
import org.firstinspires.ftc.teamcode.commands.clawOpenCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.commands.highBasketCMD;
import org.firstinspires.ftc.teamcode.commands.highChamberCMD;
import org.firstinspires.ftc.teamcode.commands.parkCMD;
import org.firstinspires.ftc.teamcode.commands.stowCMD;
import org.firstinspires.ftc.teamcode.commands.wristCMD;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.storage;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;
import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;
import static org.firstinspires.ftc.teamcode.constants.*;

@Autonomous(name = "left - 1 Specimen - 1 Sample",group = "Linear OpMode",preselectTeleOp = "mainOpMode")
public class left1Specimen1Sample extends LinearOpMode {
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







        while(!opModeIsActive() && globals.hardwareInit){
            armStart.start();
            telemetry.addData("status: ","ready");
            telemetry.update();
        }
        waitForStart();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new driveCMD(drive,leftChamber),
                                new highChamberCMD(arm,wrist),
                                new wristCMD(wrist,wristPoints.specimen)
                        ),
                        new clawOpenCMD(grabber),

                        //move to new sample pickup and pickup sample
                        new ParallelCommandGroup(
                                new driveCMD(drive,sample3),
                                new armMoveCMD(arm,1700,0),
                                new wristCMD(wrist,wristPoints.pickUp)
                        ),
                        new clawCloseCMD(grabber),
                        //stow and move to place pos
                        new armMoveCMD(arm,0,0),
                        new ParallelCommandGroup(
                                new highBasketCMD(arm,wrist),
                                new driveCMD(drive,basket),
                                new wristCMD(wrist,wristPoints.basket)

                        ),
                        new clawOpenCMD(grabber),


                        //stow
                        new stowCMD(arm,wrist),
                        //park
                        new driveCMD(drive,new Pose2d(40,-40,Math.toRadians(90))),
                        new parkCMD(drive)

                ));
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }
        storage.currentPose = drive.getPos();

    }




}
