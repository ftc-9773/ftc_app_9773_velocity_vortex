package org.firstinspires.ftc.teamcode.navigation;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.kauailabs.navx.ftc.navXPerformanceMonitor;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTCRobot;

/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class NavxMicro implements GyroInterface {
    LinearOpMode curOpMode;
    FTCRobot robot;
    Navigation navigation;

    private enum NAVX_Status {STATUS_NOT_SET, WORKING, NOT_WORKING}
    public AHRS navx_device;
    public double angleTolerance = 0.0;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 30; // changed to 30 Hz for lower update frequency
    public double straightPID_kp=0.005, turnPID_kp=0.005;
    private DcMotor.ZeroPowerBehavior prev_zp=null;
    private NAVX_Status navxStatus;
    public navXPerformanceMonitor navx_perfmon;


    public NavxMicro(LinearOpMode curOpMode, FTCRobot robot, Navigation navigation, String dimName, int portNum,
                     double angleTolerance, double straightPID_kp, double turnPID_kp) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.navigation = navigation;
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
        if (robot.autoOrTeleop.equalsIgnoreCase("Autonomous")) {
            navx_device.zeroYaw();
        }
        DbgLog.msg("ftc9773: Current yaw = %f", getModifiedYaw());

        this.straightPID_kp = straightPID_kp;
        this.turnPID_kp = turnPID_kp;
        curOpMode.telemetry.addData("navx: ", "straightKp=%f, turnKp=%f", straightPID_kp, turnPID_kp);
        curOpMode.telemetry.update();

        // navxStatus is set after the play button is pressed. It is not set during the init stage.
        this.navxStatus = NAVX_Status.STATUS_NOT_SET;

        navx_perfmon = new navXPerformanceMonitor(navx_device);
    }

    @Override
    public void initAfterStart() {
//        navx_device.registerCallback(navx_perfmon);
        testAndSetGyroStatus();
    }

    @Override
    public Navigation.GyroType getGyroType() {
        return Navigation.GyroType.NAVX_MICRO;
    }

    @Override
    public double getAngleTolerance() {
        return (angleTolerance);
    }

    @Override
    public void testAndSetGyroStatus() {
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

    public boolean navxIsWorking() {
        if (navxStatus == NAVX_Status.WORKING) {
            return (true);
        }
        else {
            return (false);
        }
    }
    @Override
    public double getYaw() {
        return getModifiedYaw();
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

    @Override
    public double getPitch() {
        return ((double)navx_device.getPitch());
    }

    @Override
    public boolean isGyroWorking() {
        return navxIsWorking();
    }

    private double convertToNavxYaw(double modifiedYaw) {
        // This method does the inverse of getModifiedYaw()
        double navxYaw = 0.0;

        if (modifiedYaw > 180) {
            navxYaw = modifiedYaw - 360;
        } else {
            navxYaw = modifiedYaw;
        }
        return (navxYaw);
    }

    @Override
    public void setRobotOrientation(double targetYaw, double speed, NavigationChecks navigationChecks) {
        // The orientation is with respect to the initial autonomous starting position
        // The initial orientation of the robot at the beginning of the autonomous period
        // is '0'. targetAngle is between 0 to 360 degrees.
        double curYaw = getModifiedYaw();
        double yawDiff;
        double diff = targetYaw - curYaw;
        double angleToTurn = diff>180 ? diff-360 : diff<-180 ? diff+360 : diff;
//        turnRobot(angleToTurn, speed, navigationChecks);
        double rightPower, leftPower;
        if (angleToTurn > 0 && angleToTurn < 360) {
            // Spin clockwise
            leftPower = speed;
            rightPower = -1 * leftPower;
        }
        else if (angleToTurn < 0 && angleToTurn > -360) {
            // Spin counter clockwise
            rightPower = speed;
            leftPower = -1 * rightPower;
        } else {
            DbgLog.msg("ftc9773: angle %f is invalid!", angleToTurn);
            return;
        }
        DbgLog.msg("ftc9773: power left = %f, right = %f",leftPower, rightPower);
        robot.instrumentation.reset();
        while (!navigationChecks.stopNavigation()) {
            this.robot.driveSystem.turnOrSpin(leftPower, rightPower);
            robot.instrumentation.addInstrData();
            yawDiff = navigation.distanceBetweenAngles(getModifiedYaw(), targetYaw);
            if (yawDiff <= angleTolerance)
                break;
        }
        this.robot.driveSystem.stop();
        robot.instrumentation.printToConsole();
    }

    @Override
    public void turnRobot(double angle, double speed, NavigationChecks navigationChecks) {
        double leftPower=0.0, rightPower=0.0;
        double startingYaw, targetYaw, yawDiff;
        double min_angleToTurn=0.0;
//        LoopStatistics instr = new LoopStatistics();
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

//        instr.startLoopInstrumentation();
        robot.instrumentation.reset();
        while (curOpMode.opModeIsActive() && !navigationChecks.stopNavigation()) {
            this.robot.driveSystem.turnOrSpin(leftPower,rightPower);
//            instr.updateLoopInstrumentation();
            robot.instrumentation.addInstrData();
            yawDiff = navigation.distanceBetweenAngles(getModifiedYaw(), startingYaw);
            if (yawDiff > min_angleToTurn)
                break;
            //DbgLog.msg("ftc9773: yawDiff=%f", yawDiff);
        }

        DbgLog.msg("ftc9773: angle = %f", angle);
        this.robot.driveSystem.stop();
//        instr.printLoopInstrumentation();
        robot.instrumentation.printToConsole();
    }

    @Override
    public void goStraightPID(boolean driveBackwards, double degrees, float speed) {
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
        // Ensure that 0.25 <= speed <= 0.75 so that correction can be meanigful.
        speed = Range.clip(speed, 0.25f, 0.75f);
        leftSpeed = Range.clip(speed - correction, 0, 1);
        rightSpeed = Range.clip(speed + correction, 0, 1);
//        DbgLog.msg("ftc9773: error=%f, correction=%f, leftSpeed,=%f, rightSpeed=%f", error, correction, leftSpeed, rightSpeed);
        if (!driveBackwards) {
            robot.driveSystem.turnOrSpin(leftSpeed, rightSpeed);
        } else {
            robot.driveSystem.turnOrSpin(-rightSpeed, -leftSpeed);
        }
    }

    @Override
    public double getUpdateCount() {
        return (navx_device.getUpdateCount());
    }
}
