package org.firstinspires.ftc.teamcode.navigation;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.FourMotorSteeringDrive;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.NavigationOptionsReader;

import java.lang.annotation.Target;
import java.util.concurrent.TimeUnit;

public class NavxMicro {
    LinearOpMode curOpMode;
    FTCRobot robot;

    private AHRS navx_device;
    private double angleTolerance = 0.0;
    private double driveSysInitialPower = 0.0;
    private double driveSysTargetPower = 0.0;

    public NavxMicro(LinearOpMode curOpMode, FTCRobot robot, String dimName, int portNum,
                     double driveSysInitialPower, double driveSysTargetPower, double angleTolerance) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.driveSysInitialPower = driveSysInitialPower;
        this.driveSysTargetPower = driveSysTargetPower;
        this.angleTolerance = angleTolerance;

        navx_device = AHRS.getInstance(curOpMode.hardwareMap.deviceInterfaceModule.get(dimName),
                portNum, AHRS.DeviceDataType.kProcessedData);

        // Set the yaw to zero
        // ToDo:  The should be done only in the autonomous mode.
        navx_device.zeroYaw();
    }

    public void setRobotOrientation(double targetAngle) {
        // The orientation is with respect to the initial autonomous starting position
        // The initial orientation of the robot at the beginning of the autonomous period
        // is '0'. targetAngle is between 0 to 360 degrees.
        double angleToTurn=0;
        double curYaw = getModifiedYaw();
        boolean spinClockwise=false;

        // 1. Calculate the angle to turn
        angleToTurn = distanceBetweenAngles(curYaw, targetAngle);
        double diff = targetAngle - curYaw;
        if (diff == angleToTurn) {
            spinClockwise = true;
        } else if (Math.abs(diff) > 180) {
            spinClockwise = true;
        } else {
            spinClockwise = false;
        }

        // 2. turn the calculated angle
        if (spinClockwise) {
            turnRobot(angleToTurn);
        } else {
            turnRobot((-1 * angleToTurn));
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

    public double distanceBetweenAngles(double angle1, double angle2) {
        // Both angle1 and angle2 are assumed to be positive numbers between 0 and 360
        // The returnValue is between 0 and 180.
        double angleDistance= Math.abs(angle1 - angle2);

        if (angleDistance > 180) {
            angleDistance = 360 - angleDistance;
        }

        return (angleDistance);
    }

    public void turnRobot(double angle) {
        double leftInitialPower=0.0, rightInitialPower=0.0;
        double leftTargetPower=0.0, rightTargetPower=0.0;
        double startingYaw, targetYaw;
        boolean spinClockwise = false;
        if (angle > 0 && angle < 360) {
            // Spin clockwise
            leftInitialPower = this.driveSysInitialPower;
            rightInitialPower = -1 * leftInitialPower;
            leftTargetPower = this.driveSysTargetPower;
            rightTargetPower = -1 * leftTargetPower;
            spinClockwise = true;
        }
        else {
            if (angle < 0 && angle > -360) {
                // Spin counter clockwise
                rightInitialPower = this.driveSysInitialPower;
                leftInitialPower = -1 * rightInitialPower;
                rightTargetPower = this.driveSysTargetPower;
                leftTargetPower = -1 * rightTargetPower;
            } else {
                DbgLog.msg("angle %f is invalid!", angle);
                return;
            }
        }

        // Note the current yaw value
        startingYaw = getModifiedYaw();
        targetYaw = startingYaw + angle;
        if (targetYaw > 360) {
            targetYaw %= 360;
        } else if (targetYaw < 0) {
            targetYaw += 360;
        }
        DbgLog.msg("initial power left = %f, right = %f",leftInitialPower, rightInitialPower);
        DbgLog.msg("target power left = %f, right = %f",leftTargetPower, rightTargetPower);
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        this.robot.driveSystem.setMaxSpeed((float) driveSysInitialPower);
//        while(elapsedTime.time()<0.1) {
//            this.robot.driveSystem.turnOrSpin(leftInitialPower, rightInitialPower);
//        }
//        this.robot.driveSystem.setMaxSpeed((float) driveSysTargetPower);
        while (true) {
            this.robot.driveSystem.turnOrSpin(leftInitialPower,rightInitialPower);
            if (distanceBetweenAngles(getModifiedYaw(), targetYaw) < this.angleTolerance)
                break;
        }
        this.robot.driveSystem.stop();
    }
}
