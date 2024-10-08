package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.commands.highChamberCMD;
import org.firstinspires.ftc.teamcode.commands.parkCMD;
import org.firstinspires.ftc.teamcode.commands.stowCMD;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;


@Autonomous(name = "right - 1 specimen - 1 hold",group = "Linear OpMode",preselectTeleOp = "mainOpMode")
public class right1Specimen extends LinearOpMode {
    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();


    public void runOpMode(){
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap);
        //TODO set start pos
        drive.setPos(new Pose2d(10, -62, Math.toRadians(90)));


        CommandScheduler.getInstance().setDefaultCommand(arm,new stowCMD(arm));
        globals.setLocation(globals.Location.RIGHT);


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new highChamberCMD(arm),
                                new driveCMD(drive,new Pose2d(10,-35,Math.toRadians(90)))
                        ),
                        new ParallelCommandGroup(
                        new driveCMD(drive,new Pose2d(47.5,-47,Math.toRadians(90))),
                                new armMoveCMD(arm,1700,0)
                        ),
                        new parkCMD(drive)


                ));


        while(!opModeIsActive() && globals.hardwareInit){
            telemetry.addData("status: ","ready");
            telemetry.update();
        }
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }

    }




}
