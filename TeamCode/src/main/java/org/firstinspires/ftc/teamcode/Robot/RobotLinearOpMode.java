package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import static java.lang.Thread.sleep;

public class RobotLinearOpMode extends Robot {

    //constructor
    public RobotLinearOpMode(LinearOpMode linearOpMode) {

        //creates the robot so that I can use all of the motors
        super(linearOpMode);

        //needs the linear opmode so that I can use telemetry and opModeIsActive()
        this.linearOpMode = linearOpMode;
    }

    LinearOpMode linearOpMode;

    //sets the powers on either a power or velocity, and with variables or an array, where 0 is x, 1 is y, and 2 is r
    //if I control it using an array, then it just sends it into an earlier method
    public void mecPowerDrive(double x, double y, double r){
        leftFrontDriveMotor.setPower(y - x + r);
        leftBackDriveMotor.setPower(y + x + r);
        rightFrontDriveMotor.setPower(y + x - r);
        rightBackDriveMotor.setPower(y - x - r);
    }
    public void mecPowerDrive(double[] controller){
        mecPowerDrive(controller[0], controller[1], controller[2]);
    }
    public void mecVelocityDrive(double x, double y, double r){
        leftFrontDriveMotor.setVelocity(y - x + r);
        leftBackDriveMotor.setVelocity(y + x + r);
        rightFrontDriveMotor.setVelocity(y + x - r);
        rightBackDriveMotor.setVelocity(y - x - r);
    }
    public void mecVelocityDrive(double[] controller){
        mecVelocityDrive(controller[0], controller[1], controller[2]);
    }

    //outputs the average of the 4 drive train motors, be sure to reset the encoders before you engage this.
    public double getAverageDriveTrainEncoder(){
        double averageEncoderPosition = leftFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
        averageEncoderPosition += rightFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += rightBackDriveMotor.getCurrentPosition();

        return averageEncoderPosition/4;
    }

    //two methods that turn precisely, which Casey made and I don't fully understand
    public void turnByDegree(double degree, BallisticMotionProfile TurnProfile){
        initIMU();

        double currentAngle;
        double adjustedMotorPower;

        //gets the current angle from the IMU
        double startAngle = getRev2IMUAngle()[2];

        //calculates the angles the angle that it needs to be at, at the end
        double neededAngle = startAngle + degree;
        currentAngle = startAngle;

        //the loop that will run until the needed Angle is acheived
        if(currentAngle < neededAngle) {
            while ((currentAngle < neededAngle && linearOpMode.opModeIsActive())) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecPowerDrive(0,0,adjustedMotorPower);
            }
        }
        else if(currentAngle > neededAngle && linearOpMode.opModeIsActive()){
            while ((currentAngle > neededAngle)) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecPowerDrive(0,0,adjustedMotorPower);
            }
        }
    }
    public void moveByInches(double inches, BallisticMotionProfile DriveProfile){
        double startPosition = getAverageDriveTrainEncoder();
        double currentPosition;

        double neededInches = inches;//change this

        double neededPosition = startPosition + (neededInches*4000/69);
        double adjustedMotorPower;

        setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(startPosition < neededPosition) {
            while ((getAverageDriveTrainEncoder() < neededPosition) && linearOpMode.opModeIsActive()) {
                currentPosition = getAverageDriveTrainEncoder();

                adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startPosition, currentPosition, neededPosition);

                mecPowerDrive(0,adjustedMotorPower,0);
            }
        }
        else if (startPosition > neededPosition) {
            while ((getAverageDriveTrainEncoder() > neededPosition) && linearOpMode.opModeIsActive()) {
                currentPosition = getAverageDriveTrainEncoder();

                adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startPosition, currentPosition, neededPosition);

                mecPowerDrive(0,adjustedMotorPower,0);
            }
        }
        else {
            //no where to move in here.
        }
    }

    //eliminates residual forces
    public void stopMotorsAndWait(double seconds){
        mecPowerDrive(0,0,0);
        linearOpMode.sleep((int)(seconds * 1000));
    }

    //sets the intake power and velocity
    public void setIntakePower(double intakePower){
        rightIntakeMotor.setPower(intakePower);
        leftIntakeMotor.setPower(intakePower);
    }
    public void setIntakeVelocity(double intakeVelocity){
        rightIntakeMotor.setVelocity(intakeVelocity);
        leftIntakeMotor.setVelocity(intakeVelocity);
    }

    //sets the lift power and veloctiy
    public void setLiftPower(double liftPower){
        liftMotor.setPower(liftPower);
    }
    public int getLiftEncoder(){
        return liftMotor.getCurrentPosition();
    }

    //sets the arm power and velocity
    public void setArmPower(double armPower){
        armMotor.setPower(armPower);
    }
    public int getArmEncoder(){
        return armMotor.getCurrentPosition();
    }

    public void closeGrabber(){
        blockGrabberServo.setPosition(.85);
    }
    public void openGrabber(){
        blockGrabberServo.setPosition(.55);
    }

    public void closeLatch(){
        leftLatchServo.setPosition(.3);
        rightLatchServo.setPosition(.45);
    }
    public void openLatch(){
        leftLatchServo.setPosition(.5);
        rightLatchServo.setPosition(.65);
    }
}
