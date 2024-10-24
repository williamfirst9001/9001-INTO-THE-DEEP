package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;

public class lights extends SubsystemBase {

    robotHardware robot = robotHardware.getInstance();

    public lights(globals.ledState state){


    }
}
