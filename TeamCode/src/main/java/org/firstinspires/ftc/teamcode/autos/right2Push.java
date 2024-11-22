package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.leftStartPos;
import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.rightStartPos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "right - 2 push",group = "Linear OpMode",preselectTeleOp = "mainOpMode")
public class right2Push extends CommandOpMode {

    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private Claw claw = new Claw();
    private Wrist wrist = new Wrist();
    private elevator arm = new elevator();
    public void initialize(){
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap);
        drive.setPos(rightStartPos);
        CommandScheduler.getInstance().registerSubsystem(arm);



        armStart.start();
        telemetry.addData("status: ", "ready");
        telemetry.update();

        robot.eMotors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        schedule(
                        new armMoveCMD(arm,wrist,globals.armVal.STOW),
                        new driveCMD(drive,(new Pose2d(39,-45,Math.toRadians(90)))),
                        new driveCMD(drive,(new Pose2d(39,-7,Math.toRadians(90)))),
                        new driveCMD(drive,(new Pose2d(49,-7,Math.toRadians(90)))),
                        new driveCMD(drive,(new Pose2d(49,-60,Math.toRadians(90)))),
                        new driveCMD(drive,(new Pose2d(49,-7,Math.toRadians(90)))),
                        new driveCMD(drive,(new Pose2d(60,-7,Math.toRadians(90)))),
                        new driveCMD(drive,(new Pose2d(60,-60,Math.toRadians(90)))),
                        new driveCMD(drive,(new Pose2d(67,-60,Math.toRadians(90))))

                );
    }


    public void run() {


        CommandScheduler.getInstance().run();



            telemetry.addData("pivot pos", robot.pivotMotor.getCurrentPosition());
            telemetry.addData("arm pos", robot.eMotors.getPosition());
            telemetry.addData("robot pos", drive.getPos());
            telemetry.update();



        storage.currentPose = drive.getPos();


    }
}
