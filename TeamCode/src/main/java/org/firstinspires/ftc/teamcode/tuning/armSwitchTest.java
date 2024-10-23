package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp(name = "armSwitchTest", group = "Linear OpMode")
@Disabled
public class armSwitchTest extends LinearOpMode {

    private TouchSensor armSwitch;

    public void runOpMode(){
        armSwitch = hardwareMap.get(TouchSensor.class, "slideswitch");
        waitForStart();
        while(opModeIsActive()){
            if(armSwitch.isPressed()){
                telemetry.addData("switch detected",true);
            } else {
                telemetry.addData("switch detected",false);
            }
            telemetry.update();

        }
    }
}
