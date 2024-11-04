package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotHardware;
import org.firstinspires.ftc.teamcode.subsystems.limeLight;
@TeleOp(name = "limelight test", group = "Linear OpMode")
public class limelightTest extends LinearOpMode {
    private robotHardware robot = robotHardware.getInstance();
    private limeLight limelight = new limeLight();
    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            limelight.periodic();
            if(limelight.getFieldPos()!= null) {
                telemetry.addData("x", limelight.getFieldPos().getX());
                telemetry.addData("y", limelight.getFieldPos().getY());
                telemetry.addData("heading", limelight.getFieldPos().getHeading());
                telemetry.update();
            }

        }
    }
}
