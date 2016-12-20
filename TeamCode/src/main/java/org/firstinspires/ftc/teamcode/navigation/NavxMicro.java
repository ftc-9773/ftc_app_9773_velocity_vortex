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
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    navXPIDController yawPIDController=null;
    navXPIDController.PIDResult yawPIDResult=null;
    private double drive_speed=0.0;
    public double straightPID_kp=0.005, turnPID_kp=0.005;
    private double pid_minSpeed=-1.0, pid_maxSpeed=1.0;
    private DcMotor.ZeroPowerBehavior prev_zp=null;

    public NavxMicro(LinearOpMode curOpMode, FTCRobot robot, String dimName, int portNum,
                     double driveSysInitialPower, double angleTolerance, double straightPID_kp,
                     double turnPID_kp, double pid_minSpeed, double pid_maxSpeed) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.driveSysInitialPower = driveSysInitialPower;
        this.angleTolerance = angleTolerance;

        DbgLog.msg("dimName=%s, portNum=%d", dimName, portNum);
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
            DbgLog.msg("still calibating navx....");
            curOpMode.telemetry.addData("navx: ", "calibrating %s", "navx");
//            curOpMode.sleep(20);
        } else {
            DbgLog.msg("Done with calibrating navx");
            curOpMode.telemetry.addData("navx: ", "Done with calibrating %s", "navx");
        }

        // ToDo:  The should be done only in the autonomous mode.
        navx_device.zeroYaw();
        DbgLog.msg("Current yaw = %f", getModifiedYaw());

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
    }

    public void setRobotOrientation(double targetAngle, double speed) {
        // The orientation is with respect to the initial autonomous starting position
        // The initial orientation of the robot at the beginning of the autonomous period
        // is '0'. targetAngle is between 0 to 360 degrees.
        double curYaw = getModifiedYaw();
        double diff = targetAngle - curYaw;
        double angleToTurn = diff>180 ? diff-360 : diff<-180 ? diff+360 : diff;
        turnRobot(angleToTurn, speed);
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

    public double distanceBetweenAngles(double angle1, double angle2) {
        // Both angle1 and angle2 are assumed to be positive numbers between 0 and 360
        // The returnValue is between 0 and 180.
        double angleDistance= Math.abs(angle1 - angle2);

        if (angleDistance > 180) {
            angleDistance = 360 - angleDistance;
        }

        return (angleDistance);
    }

    public void turnRobot(double angle, double speed) {
        double leftPower=0.0, rightPower=0.0;
        double startingYaw, targetYaw, yawDiff, prevYawDiff;
        boolean spinClockwise = false;
        if (angle > 0 && angle < 360) {
            // Spin clockwise
            leftPower = this.driveSysInitialPower;
            rightPower = -1 * leftPower;
            spinClockwise = true;
        }
        else {
            if (angle < 0 && angle > -360) {
                // Spin counter clockwise
                rightPower = this.driveSysInitialPower;
                leftPower = -1 * rightPower;
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
        DbgLog.msg("initial power left = %f, right = %f",leftPower, rightPower);
            DbgLog.msg("raw Yaw = %f, Starting yaw = %f, Current Yaw = %f, targetYaw = %f",
                    navx_device.getYaw(), startingYaw, getModifiedYaw(), targetYaw);
        yawDiff = prevYawDiff = distanceBetweenAngles(startingYaw, targetYaw);
        this.robot.driveSystem.setMaxSpeed((float) speed);
        while (curOpMode.opModeIsActive()) {
            this.robot.driveSystem.turnOrSpin(leftPower,rightPower);
//            DbgLog.msg("raw Yaw = %f, Starting yaw = %f, Current Yaw = %f, targetYaw = %f",
//                    navx_device.getYaw(), startingYaw, getModifiedYaw(), targetYaw);
            /* ToDo:  we may miss the small window of time when the robot is within the angleTolerance.
               In this case, the robot keeps spinning until it comes within the angleTolerance again.
               To avoid this scenario, keep track of whether the robot is getting closer to targetYaw
               or getting farther away from targetYaw.  If it is getting farther away, stop immediately.
             */
            yawDiff = distanceBetweenAngles(getModifiedYaw(), targetYaw);
//            if ((yawDiff < this.angleTolerance) || ((yawDiff - prevYawDiff) > 0))
            if (yawDiff < this.angleTolerance)
                break;
            DbgLog.msg("yawDiff=%f", yawDiff);
            prevYawDiff = yawDiff;
        }
        this.robot.driveSystem.stop();
        this.robot.driveSystem.resumeMaxSpeed();
    }

    public void MygoStraightPID(boolean driveBackwards, double degrees) {
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
        leftSpeed = drive_speed - correction;
        rightSpeed = drive_speed + correction;
        DbgLog.msg("error=%f, correction=%f, leftSpeed,=%f, rightSpeed=%f", error, correction, leftSpeed, rightSpeed);
        if (!driveBackwards) {
            robot.driveSystem.turnOrSpin(leftSpeed, rightSpeed);
        } else {
            robot.driveSystem.turnOrSpin(-rightSpeed, -leftSpeed);
        }
    }

    public void testNavxCalibrateConnection() {
        if (this.navx_device.isCalibrating()) {
            DbgLog.msg("Navx device is calibrating");
        } else {
            DbgLog.msg("Navx device is done with calibration");
        }

        if (this.navx_device.isConnected()) {
            DbgLog.msg("Navx device is connected");
        } else {
            DbgLog.msg("Navx device is not connected");
        }
    }

    public void shiftRobot(double distance, boolean isForward){
        double moveDistance = Math.sqrt(100 + Math.pow(Math.abs(distance), 2));
        double angle = 90 - Math.toDegrees(Math.asin(10/moveDistance));

        if (isForward){
            if (distance < 0) {
                angle *= -1;
            }
            this.turnRobot(angle, this.driveSysInitialPower);
            robot.driveSystem.driveToDistance((float) this.drive_speed, moveDistance);
            this.turnRobot(-angle, this.driveSysInitialPower);
        }
        else{
            if (distance > 0){
                angle *= -1;
            }
            this.turnRobot(angle, this.driveSysInitialPower);
            robot.driveSystem.driveToDistance((float) this.drive_speed, -moveDistance);
            this.turnRobot(-angle, this.driveSysInitialPower);
        }
    }
}
