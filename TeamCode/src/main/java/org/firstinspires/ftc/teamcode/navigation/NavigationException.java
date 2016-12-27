package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;

import java.io.BufferedReader;

public class NavigationException {
    public enum NavExceptions {OPMODE_NOTACTIVE, ROBOT_TILTING, TIMED_OUT, FOUND_WHITElINE}
    private NavExceptions[] excsToCheck;
    NavExceptions excThatOccurred;
    LinearOpMode curOpMode;
    FTCRobot robot;
    Navigation navigationObj;
    long timeoutMilliSec;

    public NavigationException(FTCRobot robot, LinearOpMode curOpMode, Navigation navigationObj,
                               long timeoutMilliSec, NavExceptions[] exceptions) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.navigationObj = navigationObj;
        this.timeoutMilliSec = timeoutMilliSec;
        this.excsToCheck = exceptions;
        excThatOccurred = null;
    }

    public boolean checkExceptions() {
        for (NavExceptions e: this.excsToCheck) {
            if (this.exceptionOccurred(e)) {
                this.excThatOccurred = e;
                return (true);
            }
        }
        return (false);
    }

    public boolean exceptionOccurred(NavExceptions e) {
        boolean rc = false;
        switch (e) {
            case OPMODE_NOTACTIVE:
                if (!curOpMode.opModeIsActive()) {
                    rc = true;
                }
                break;
            case ROBOT_TILTING:
                break;
            case TIMED_OUT:
                break;
        }
        return (rc);
    }
}
