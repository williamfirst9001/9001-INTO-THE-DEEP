package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.highBasketCMD;
import org.firstinspires.ftc.teamcode.commands.highChamberCMD;
import org.firstinspires.ftc.teamcode.commands.stowCMD;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;
@Autonomous(name = "red close basket",group = "Linear OpMode",preselectTeleOp = "mainOpMode")
public class redCloseBasket extends CommandOpMode {
    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();



    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        //drive.setPos(new Pose2d());


        //CommandScheduler.getInstance().setDefaultCommand(arm,new stowCMD(arm));
    }
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        schedule(
                new SequentialCommandGroup(
                        //new driveCMD(drive,new Pose2d(-36,-60,Math.toRadians(90))),
                        new highChamberCMD(arm),
                        //deposit

                        //get new sample and move
                        new ParallelCommandGroup(
                                //new driveCMD(drive,new Pose2d(-47.5,-47,Math.toRadians(90))),
                                new armMoveCMD(arm,1700,0)
                                //grab
                        ),
                        new stowCMD(arm),
                        new ParallelCommandGroup(
                                new highBasketCMD(arm)//,
                                //new driveCMD(drive,new Pose2d(-50,-47,Math.toRadians(225)))
                        ),
                        //deposit
                        new stowCMD(arm),
                        new ParallelCommandGroup(
                                new armMoveCMD(arm,1700,0)//,
                                //new driveCMD(drive,new Pose2d(-58,-47,Math.toRadians(90)))
                        ),
                        new ParallelCommandGroup(
                                new stowCMD(arm)//,
                                //new driveCMD(drive,new Pose2d(-50,-47,Math.toRadians(225)))
                        )//,
                        //park
                        //new driveCMD(drive,new Pose2d(40,-62,Math.toRadians(90)))
                ));
       // CommandScheduler.getInstance().disable();

    }




}
