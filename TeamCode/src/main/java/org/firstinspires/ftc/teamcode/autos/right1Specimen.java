package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.armMoveCMD;
import org.firstinspires.ftc.teamcode.commands.clawCloseCMD;
import org.firstinspires.ftc.teamcode.commands.clawOpenCMD;
import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.commands.highChamberCMD;
import org.firstinspires.ftc.teamcode.commands.parkCMD;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.driveBase;
import org.firstinspires.ftc.teamcode.subsystems.elevator;
import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;


@Autonomous(name = "right - 1 specimen - 1 hold",group = "Linear OpMode",preselectTeleOp = "mainOpMode")
public class right1Specimen extends LinearOpMode {
    private elevator arm = new elevator();
    private driveBase drive = new driveBase();
    private robotHardware robot = robotHardware.getInstance();
    private Claw grabber = new Claw();
    private Wrist wrist = new Wrist();


    public void runOpMode(){
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap);
        //TODO set start pos
        drive.setPos(rightStartPos);
        CommandScheduler.getInstance().registerSubsystem(arm);

        globals.setLocation(globals.Location.RIGHT);






        while(!opModeIsActive() && globals.hardwareInit){
            telemetry.addData("status: ","ready");
            telemetry.update();
        }
        waitForStart();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new highChamberCMD(arm,wrist),
                                new driveCMD(drive,rightChamber)
                        ),
                        new clawOpenCMD(grabber),
                        new ParallelCommandGroup(
                                new driveCMD(drive,sample4),
                                new armMoveCMD(arm,1700,0)
                        ),
                        new clawCloseCMD(grabber),
                        new parkCMD(drive)


                ));
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }

    }




}
