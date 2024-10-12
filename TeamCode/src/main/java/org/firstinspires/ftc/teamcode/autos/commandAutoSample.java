package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.stowCMD;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;
@Disabled
@Autonomous(name = "command test auto",group = "Linear OpMode",preselectTeleOp = "mainOpMode")
public class commandAutoSample extends LinearOpMode {
    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();


    public void runOpMode(){
        robot.init(hardwareMap);
        CommandScheduler.getInstance().reset();
        //TODO set start pos
        drive.setPos(new Pose2d(10, -62, Math.toRadians(90)));
        //TODO set side


        CommandScheduler.getInstance().setDefaultCommand(arm,new stowCMD(arm));


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

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
