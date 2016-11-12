package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FourMotorSteeringDrive extends DriveSystem {
    DcMotor motorL1 = null;
    DcMotor motorL2 = null;
    DcMotor motorR1 = null;
    DcMotor motorR2 = null;
    double frictionCoefficient;
    double maxSpeed;
    double minSpeed;
    Wheel wheel;
    int motorCPR;  // Cycles Per Revolution.  == 1120 for Neverest40

    public FourMotorSteeringDrive(DcMotor motorL1, DcMotor motorL2, DcMotor motorR1, DcMotor motorR2,
                                  double maxSpeed, double minSpeed, double frictionCoefficient,
                                  Wheel wheel, int motorCPR){
        this.motorL1 = motorL1;
        this.motorL2 = motorL2;
        this.motorR1 = motorR1;
        this.motorR2 = motorR2;
        this.motorR1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorR2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.frictionCoefficient = frictionCoefficient;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.wheel = wheel;
        this.motorCPR = motorCPR;
    }

    @Override
    public void drive(float speed, float direction){
        double left = (speed + direction) * frictionCoefficient;
        double right = (speed - direction) * frictionCoefficient;

        motorL1.setPower(left);
        motorL2.setPower(left);
        motorR1.setPower(right);
        motorR2.setPower(right);
    }

    @Override
    public void turnOrSpin(double leftSpeed, double rightSpeed) {
        motorL1.setPower(leftSpeed);
        motorL2.setPower(leftSpeed);
        motorR1.setPower(rightSpeed);
        motorR2.setPower(rightSpeed);
    }

    @Override
    public void stop() {
        motorL1.setPower(0.0);
        motorL2.setPower(0.0);
        motorR1.setPower(0.0);
        motorR2.setPower(0.0);
    }

//    @Override
//    public void drive(float speed, float direction, int distanceInCounts){
//        double left = (speed + direction) * frictionCoefficient;
//        double right = (speed - direction) * frictionCoefficient;
//
//        int targetDistance = getAvgEncoderVal() + distanceInCounts;
//
//        motorR1.setTargetPosition(targetDistance);
//        motorR2.setTargetPosition(targetDistance);
//        motorL1.setTargetPosition(targetDistance);
//        motorL2.setTargetPosition(targetDistance);
//
//        motorL1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        motorL1.setPower(left);
//        motorL2.setPower(left);
//        motorR1.setPower(right);
//        motorR2.setPower(right);
//
//        while (motorL1.isBusy() && motorL2.isBusy() && motorR1.isBusy() && motorR2.isBusy()){
//
//        }
//
//        motorL1.setPower(0);
//        motorL2.setPower(0);
//        motorR1.setPower(0);
//        motorR2.setPower(0);
//
//        motorL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

    @Override
    public void driveToDistance(float speed, double distanceInInches){
        double countsPerInch = motorCPR / wheel.getCircumference();
        double targetCounts = countsPerInch * distanceInInches;

        motorL1.setTargetPosition((int) targetCounts);
        motorL2.setTargetPosition((int) targetCounts);
        motorR1.setTargetPosition((int) targetCounts);
        motorR2.setTargetPosition((int) targetCounts);

        setDriveSysMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorL1.setPower(speed * frictionCoefficient);
        motorL2.setPower(speed * frictionCoefficient);
        motorR1.setPower(speed * frictionCoefficient);
        motorR2.setPower(speed * frictionCoefficient);

        while(motorL1.isBusy()&&motorL2.isBusy()&&motorR1.isBusy()&&motorR2.isBusy()){
            curOpMode.idle();
        }

        this.stop();

        setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void setDriveSysMode(DcMotor.RunMode runMode){
        motorL1.setMode(runMode);
        motorL2.setMode(runMode);
        motorR1.setMode(runMode);
        motorR2.setMode(runMode);
    }
    /*public void driveToDistance(float speed, float direction, double distance){
        double startingPositionL = motorL1.getCurrentPosition();
        double startingPositionR = motorR1.getCurrentPosition();

        double targetPosition =(distance / wheelValues[1]) * 1120;

        while(((motorL1.getCurrentPosition()-startingPositionL)<targetPosition) && ((motorR.getCurrentPosition()-startingPositionR)<targetPosition)){
            drive(speed, direction);
        }
        motorR.setPower(0);
        motorL.setPower(0);
    }*/
}
