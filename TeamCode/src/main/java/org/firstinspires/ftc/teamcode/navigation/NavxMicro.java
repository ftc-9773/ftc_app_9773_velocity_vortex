package org.firstinspires.ftc.teamcode.navigation;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.LoopStatistics;

/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class NavxMicro {
    LinearOpMode curOpMode;
    FTCRobot robot;
    Navigation navigation;

    private enum NAVX_Status {STATUS_NOT_SET, WORKING, NOT_WORKING}
    private AHRS navx_device;
    private double angleTolerance = 0.0;
    private double driveSysInitialPower = 0.0;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    navXPIDController yawPIDController=null;
    navXPIDController.PIDResult yawPIDResult=null;
    private double drive_speed=0.0;
    public double straightPID_kp=0.005, turnPID_kp=0.005;
    private double pid_minSpeed=-1.0, pid_maxSpeed=1.0;
    private DcMotor.ZeroPowerBehavior prev_zp=null;
    private NAVX_Status navxStatus;

    private double lastAccelX = 0.0;
    private double lastAccelY = 0.0;
    private final double COLLISION_THRESHOLD = 0.5;



    public NavxMicro(LinearOpMode curOpMode, FTCRobot robot, Navigation navigation, String dimName, int portNum,
                     double driveSysInitialPower, double angleTolerance, double straightPID_kp,
                     double turnPID_kp, double pid_minSpeed, double pid_maxSpeed) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.navigation = navigation;
        this.driveSysInitialPower = driveSysInitialPower;
        this.angleTolerance = angleTolerance;

        DbgLog.msg("ftc9773: dimName=%s, portNum=%d", dimName, portNum);
        navx_device = AHRS.getInstance(curOpMode.hardwareMap.deviceInterfaceModule.get(dimName),
                portNum, AHRS.DeviceDataType.kProcessedData);

        // Set the yaw to zero
        if (navx_device.isConnected()) {
            curOpMode.telemetry.addData("navx: ", "navx is connected");
        } else {
            curOpMode.telemetry.addData("navx: ", "navx is not connected");
        }
        if (navx_device.isCalibrating()) {
            // sleep for 20 milli seconds
            DbgLog.msg("ftc9773: still calibating navx....");
            curOpMode.telemetry.addData("navx: ", "calibrating %s", "navx");
//            curOpMode.sleep(20);
        } else {
            DbgLog.msg("ftc9773: Done with calibrating navx");
            curOpMode.telemetry.addData("navx: ", "Done with calibrating %s", "navx");
        }

        // ToDo:  The should be done only in the autonomous mode.
        navx_device.zeroYaw();
        DbgLog.msg("ftc9773: Current yaw = %f", getModifiedYaw());

        /* Configure the PID controller */
        this.pid_minSpeed = pid_minSpeed;
        this.pid_maxSpeed = pid_maxSpeed;
        /* Drive straight forward at 3/4 of full drive speed */
        this.drive_speed = pid_maxSpeed * 0.75;
        this.straightPID_kp = straightPID_kp;
        this.turnPID_kp = turnPID_kp;
        curOpMode.telemetry.addData("navx: ", "minSpeed=%f, maxSpeed=%f, driveSpeed=%f",
                pid_minSpeed, pid_maxSpeed, drive_speed);
        curOpMode.telemetry.addData("navx: ", "straightKp=%f, turnKp=%f", straightPID_kp, turnPID_kp);
        curOpMode.telemetry.update();

        // navxStatus is set after the play button is pressed. It is not set during the init stage.
        this.navxStatus = NAVX_Status.STATUS_NOT_SET;
    }

    public void setNavxStatus() {
        double updateCount1 = navx_device.getUpdateCount();
        curOpMode.sleep(200);
        double updateCount2 = navx_device.getUpdateCount();
        if (navx_device.isConnected() && !navx_device.isCalibrating() &&
                (updateCount2 > updateCount1)) {
            navxStatus = NAVX_Status.WORKING;
        }
        else {
            navxStatus = NAVX_Status.NOT_WORKING;
        }
    }

    public boolean navxIsWorking() {
        if (navxStatus == NAVX_Status.WORKING) {
            return (true);
        }
        else {
            return (false);
        }
    }

    public double getModifiedYaw() {
        double newYaw = 0.0;
        double curYaw = navx_device.getYaw();
        // Note:  The navx outputs values from 0 to 180 degrees and -180 to 0 degrees as the
        // robot spins clockwise. Convert this to a 0 to 360 degrees scale.
        if (curYaw > -180 && curYaw < 0) {
            newYaw = 360 + curYaw;
        } else {
            newYaw = curYaw;
        }
        return (newYaw);
    }

    public double getPitch() {
        return ((double)navx_device.getPitch());
    }

    public double convertToNavxYaw(double modifiedYaw) {
        // This method does the inverse of getModifiedYaw()
        double navxYaw = 0.0;

        if (modifiedYaw > 180) {
            navxYaw = modifiedYaw - 360;
        } else {
            navxYaw = modifiedYaw;
        }
        return (navxYaw);
    }

    public void setRobotOrientation(double targetAngle, double speed, NavigationChecks navigationChecks) {
        // The orientation is with respect to the initial autonomous starting position
        // The initial orientation of the robot at the beginning of the autonomous period
        // is '0'. targetAngle is between 0 to 360 degrees.
        double curYaw = getModifiedYaw();
        double diff = targetAngle - curYaw;
        double angleToTurn = diff>180 ? diff-360 : diff<-180 ? diff+360 : diff;
        turnRobot(angleToTurn, speed, navigationChecks);
    }

    public void turnRobot(double angle, double speed, NavigationChecks navigationChecks) {
        double leftPower=0.0, rightPower=0.0;
        double startingYaw, targetYaw, yawDiff;
        double min_angleToTurn=0.0;
        LoopStatistics instr = new LoopStatistics();
        if (angle > 0 && angle < 360) {
            // Spin clockwise
            leftPower = speed;
            rightPower = -1 * leftPower;
        }
        else if (angle < 0 && angle > -360) {
            // Spin counter clockwise
            rightPower = speed;
            leftPower = -1 * rightPower;
        } else {
            DbgLog.msg("ftc9773: angle %f is invalid!", angle);
            return;
        }

        // Note the current yaw value
        startingYaw = getModifiedYaw();
        min_angleToTurn = Math.abs(angle) - angleTolerance;
        targetYaw = startingYaw + angle;
        if (targetYaw > 360) {
            targetYaw %= 360;
        } else if (targetYaw < 0) {
            targetYaw += 360;
        }
        DbgLog.msg("ftc9773: power left = %f, right = %f",leftPower, rightPower);
        DbgLog.msg("ftc9773: raw Yaw = %f, Starting yaw = %f, Current Yaw = %f, targetYaw = %f",
                navx_device.getYaw(), startingYaw, getModifiedYaw(), targetYaw);

        instr.startLoopInstrumentation();
        while (curOpMode.opModeIsActive() && !navigationChecks.stopNavigation()) {
            this.robot.driveSystem.turnOrSpin(leftPower,rightPower);
            instr.updateLoopInstrumentation();
            yawDiff = navigation.distanceBetweenAngles(getModifiedYaw(), startingYaw);
            if (yawDiff > min_angleToTurn)
                break;
            //DbgLog.msg("ftc9773: yawDiff=%f", yawDiff);
        }

        DbgLog.msg("ftc9773: angle = %f", angle);
        this.robot.driveSystem.stop();
        instr.printLoopInstrumentation();
    }

    public void navxGoStraightPID(boolean driveBackwards, double degrees, float speed) {
        // degrees specified robot orientation
        double error=0.0, correction=0.0;
        double leftSpeed, rightSpeed;
        error = getModifiedYaw() - degrees;
        if (error > 180) {
            error = error - 360;
        } else if (error < -180) {
            error = error + 360;
        }
        correction = this.straightPID_kp * error / 2;
        leftSpeed = Range.clip(speed - correction, 0, drive_speed);
        rightSpeed = Range.clip(speed + correction, 0, drive_speed);
//        leftSpeed = drive_speed - correction;
//        rightSpeed = drive_speed + correction;
//        DbgLog.msg("ftc9773: error=%f, correction=%f, leftSpeed,=%f, rightSpeed=%f", error, correction, leftSpeed, rightSpeed);
        if (!driveBackwards) {
            robot.driveSystem.turnOrSpin(leftSpeed, rightSpeed);
        } else {
            robot.driveSystem.turnOrSpin(-rightSpeed, -leftSpeed);
        }
    }

    public void testNavxCalibrateConnection() {
        if (this.navx_device.isCalibrating()) {
            DbgLog.msg("ftc9773: Navx device is calibrating");
        } else {
            DbgLog.msg("ftc9773: Navx device is done with calibration");
        }

        if (this.navx_device.isConnected()) {
            DbgLog.msg("ftc9773: Navx device is connected");
        } else {
            DbgLog.msg("ftc9773: Navx device is not connected");
        }
    }

    //update collision detection
    public boolean detectCollision() {
        double curAccelX = navx_device.getWorldLinearAccelX();
        double curJerkX = curAccelX - lastAccelX;
        lastAccelX = curAccelX;

        double curAccelY = navx_device.getWorldLinearAccelY();
        double curJerkY = curAccelY - lastAccelY;
        lastAccelY = curAccelY;

        boolean collisionIsDetected = lastAccelX > COLLISION_THRESHOLD || lastAccelY > COLLISION_THRESHOLD;
        DbgLog.msg("collision is detected? %s", collisionIsDetected ? "COLLISION!!!!!!!!!" : "NO COLLISION");
        return collisionIsDetected;
    }
}
