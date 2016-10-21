package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;

public class NavxMicro {
    LinearOpMode curOpMode;
    FTCRobot robot;
    String navxDeviceName;
    public NavxMicro(LinearOpMode curOpMode, FTCRobot robot, String navxDeviceName) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.navxDeviceName = navxDeviceName;
        // ToDo
    }
}
