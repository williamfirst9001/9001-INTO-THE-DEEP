package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.armStart;
import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.armScoreCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;
import static org.firstinspires.ftc.teamcode.tuning.autoTunePoints.*;

import java.util.Arrays;
import java.util.List;
@Disabled
@TeleOp(name="auto tuning", group="Linear OpMode")
public class autoTuning extends CommandOpMode {
    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private Claw claw = new Claw();
    private Wrist wrist = new Wrist();
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        // CommandScheduler.getInstance().setDefaultCommand(arm, new stowCMD(arm));
        //drive.setPos(storage.currentPose);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        robot.init(hardwareMap);
        armStart.reset();
        drive.setPos(new Pose2d(-10, -62, Math.toRadians(90)));

        while(!opModeIsActive() && globals.hardwareInit){
            //armStart.start();
            telemetry.addData("status","ready");
            telemetry.update();
        }
        robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
/*
        if(command.equals("move elevator")){
                CommandScheduler.getInstance().schedule(new armMoveCMD(arm,wrist,Arrays.asList(elevatorPoint,pivotPoint,wristPoint)));
                command = "";
        }
        if(command.equals("move bot")){
            CommandScheduler.getInstance().schedule(new driveCMD(drive, new Pose2d(xDrivePoint,yDrivePoint,Math.toRadians(headingDrivePoint))));
            command = "";
        }
        if(command.equals("reset from start")){
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new armMoveCMD(arm,wrist,0,0,wristPoint),
                    new driveCMD(drive, new Pose2d(-10, -62, Math.toRadians(90)))
            ));
            command = "";
        }
        if(command.equals("score")){
            CommandScheduler.getInstance().schedule(new armScoreCMD(arm,wrist,claw, globals.armVal.HIGH_BASKET));
            command = "";
        }

        telemetry.update();
*/
    }


}
