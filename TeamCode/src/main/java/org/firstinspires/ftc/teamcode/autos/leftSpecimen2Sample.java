package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.commands.highBasketCMD;
import org.firstinspires.ftc.teamcode.commands.highChamberCMD;
import org.firstinspires.ftc.teamcode.commands.parkCMD;
import org.firstinspires.ftc.teamcode.commands.stowCMD;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.poseStorage;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;
import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

@Autonomous(name = "left - 1 Specimen - 2 Sample",group = "Linear OpMode",preselectTeleOp = "mainOpMode")
public class leftSpecimen2Sample extends LinearOpMode {
    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();


    public void runOpMode(){
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap);
        drive.setPos(leftStartPos);






        CommandScheduler.getInstance().setDefaultCommand(arm,new stowCMD(arm));



        while(!opModeIsActive() && globals.hardwareInit){
            telemetry.addData("status: ","ready");
            telemetry.update();
        }
        waitForStart();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new driveCMD(drive,leftChamber),
                                new highChamberCMD(arm)
                                //deposit
                        ),
                        //move to new sample pickup and pickup sample
                        new ParallelCommandGroup(
                                new driveCMD(drive,sample3),
                                new armMoveCMD(arm,1700,0)
                                //TODO: grab
                        ),
                        //stow and move to place pos
                        new armMoveCMD(arm,0,0),
                        new ParallelCommandGroup(
                                new highBasketCMD(arm),
                                new driveCMD(drive,basket)

                        ),
                        //TODO: place in high basket

                        //stow
                        new armMoveCMD(arm,0,0),
                        //get new sample and move to sample pos
                        new ParallelCommandGroup(
                                new armMoveCMD(arm,1700,0),
                                new driveCMD(drive,sample2)
                        ),
                        //stow and move back to place pos
                        new ParallelCommandGroup(
                                new armMoveCMD(arm,0,0),
                                new driveCMD(drive,basket)

                        ),
                        //TODO: deposit
                        //park
                        new driveCMD(drive,new Pose2d(40,-40,Math.toRadians(90))),
                        new parkCMD(drive)
                ));
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }
        poseStorage.currentPose = drive.getPos();
    }




}
