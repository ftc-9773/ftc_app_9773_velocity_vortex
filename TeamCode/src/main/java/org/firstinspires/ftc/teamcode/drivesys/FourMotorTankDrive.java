package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.robotcore.hardware.DcMotor;

public class FourMotorTankDrive {
    DcMotor motorL1 = null;
    DcMotor motorL2 = null;
    DcMotor motorR1 = null;
    DcMotor motorR2 = null;
    double frictionCoefficientR;
    double frictionCoefficientL;
    double maxSpeed;
    double minSpeed;
    Wheel wheel;
    double[] wheelValues;

    public FourMotorTankDrive(DcMotor motorL1, DcMotor motorL2, DcMotor motorR1, DcMotor motorR2, double maxSpeed, double minSpeed, double frictionCoefficientR, double frictionCoefficientL, Wheel wheel){
        this.motorL1 = motorL1;
        this.motorL2 = motorL2;
        this.motorR1 = motorR1;
        this.motorR2 = motorR2;
        this.frictionCoefficientR = frictionCoefficientR;
        this.frictionCoefficientL = frictionCoefficientL;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.wheel = wheel;
        this.wheelValues = wheel.getValues();
    }

    public void drive(float speed, float direction){
        double left = (-speed + direction) * frictionCoefficientL;
        double right = (speed + direction) * frictionCoefficientR;

        motorL1.setPower(left);
        motorL2.setPower(left);
        motorR1.setPower(right);
        motorR2.setPower(right);
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
