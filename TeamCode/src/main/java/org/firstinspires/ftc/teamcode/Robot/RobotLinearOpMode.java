package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;

public class RobotLinearOpMode extends Robot {

    LinearOpMode linearOpMode;

    //constructor
    public RobotLinearOpMode(LinearOpMode linearOpMode) {

        //creates the robot so that I can use all of the motors
        super(linearOpMode);

        //needs the linear opmode so that I can use telemetry and opModeIsActive()
        this.linearOpMode = linearOpMode;
    }

    //sets the powers on either a power or velocity, and with variables or an array, where 0 is x, 1 is y, and 2 is r
    //if I control it using an array, then it just sends it into an earlier method
    public void mecPowerDrive(double x, double y, double r) {
        leftFrontDriveMotor.setPower(y - x + r);
        leftBackDriveMotor.setPower(y + x + r);
        rightFrontDriveMotor.setPower(y + x - r);
        rightBackDriveMotor.setPower(y - x - r);
    }

    public void mecPowerDrive(double[] controller) {
        mecPowerDrive(controller[0], controller[1], controller[2]);
    }

    public void mecVelocityDrive(double x, double y, double r) {
        leftFrontDriveMotor.setVelocity(y - x + r);
        leftBackDriveMotor.setVelocity(y + x + r);
        rightFrontDriveMotor.setVelocity(y + x - r);
        rightBackDriveMotor.setVelocity(y - x - r);
    }

    public void mecVelocityDrive(double[] controller) {
        mecVelocityDrive(controller[0], controller[1], controller[2]);
    }

    //outputs the average of the 4 drive train motors, be sure to reset the encoders before you engage this.
    public double getAverageYDriveTrainEncoder() {
        double averageEncoderPosition = 0;

        averageEncoderPosition += leftFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
        averageEncoderPosition += rightFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += rightBackDriveMotor.getCurrentPosition();

        return averageEncoderPosition / 4;
    }

    public double getAverageXDriveTrainEncoder() {
        double averageEncoderPosition = 0;

        averageEncoderPosition -= leftFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
        averageEncoderPosition += rightFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition -= rightBackDriveMotor.getCurrentPosition();

        return averageEncoderPosition / 4;
    }

    public double getAverageRDriveTrainEncoder() {
        double averageEncoderPosition = 0;

        averageEncoderPosition += leftFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
        averageEncoderPosition -= rightFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition -= rightBackDriveMotor.getCurrentPosition();

        return averageEncoderPosition / 4;
    }

    //two methods that turn precisely, which Casey made and I don't fully understand
    public void turnByDegree(double degree) {
        initIMU();

        BallisticMotionProfile TurnProfile = new BallisticMotionProfile(0, 0, 90, .05, 1, .65);

        double currentAngle;
        double adjustedMotorPower;

        //gets the current angle from the IMU
        double startAngle = getRev2IMUAngle()[2];

        //calculates the angles the angle that it needs to be at, at the end
        double neededAngle = startAngle + degree;
        currentAngle = startAngle;

        //the loop that will run until the needed Angle is acheived
        if (currentAngle < neededAngle) {
            while ((currentAngle < neededAngle && linearOpMode.opModeIsActive())) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecPowerDrive(0, 0, adjustedMotorPower);
            }
        } else if (currentAngle > neededAngle && linearOpMode.opModeIsActive()) {
            while ((currentAngle > neededAngle)) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecPowerDrive(0, 0, adjustedMotorPower);
            }
        }
    }

    public static enum MOVEMENT_DIRECTION {
        X,
        Y,
        R,
    }


    public void moveByInches(double desiredMovementInInches, MOVEMENT_DIRECTION movement_direction) {
        double startPosition = 0;
        double currentPosition = 0;
        double neededPosition = 0;
        double adjustedMotorPower = 0;

        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 800, 0.05, 1, 0.75);

        if (movement_direction == MOVEMENT_DIRECTION.X)
            startPosition = getAverageXDriveTrainEncoder();
        if (movement_direction == MOVEMENT_DIRECTION.Y)
            startPosition = getAverageYDriveTrainEncoder();
        if (movement_direction == MOVEMENT_DIRECTION.R)
            startPosition = getAverageRDriveTrainEncoder();

        neededPosition = startPosition + (desiredMovementInInches * 4000 / 69);

        setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (startPosition < neededPosition) {

            do {
                if (movement_direction == MOVEMENT_DIRECTION.X)
                    currentPosition = getAverageXDriveTrainEncoder();
                if (movement_direction == MOVEMENT_DIRECTION.Y)
                    currentPosition = getAverageYDriveTrainEncoder();
                if (movement_direction == MOVEMENT_DIRECTION.R)
                    currentPosition = getAverageRDriveTrainEncoder();

                adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startPosition, currentPosition, neededPosition);

                if (movement_direction == MOVEMENT_DIRECTION.X)
                    mecPowerDrive(adjustedMotorPower, 0, 0);
                if (movement_direction == MOVEMENT_DIRECTION.Y)
                    mecPowerDrive(0, adjustedMotorPower, 0);
                if (movement_direction == MOVEMENT_DIRECTION.R)
                    mecPowerDrive(0, 0, adjustedMotorPower);

            } while ((currentPosition < neededPosition) && linearOpMode.opModeIsActive());

        } else if (startPosition > neededPosition) {

            do {
                if (movement_direction == MOVEMENT_DIRECTION.X)
                    currentPosition = getAverageXDriveTrainEncoder();
                if (movement_direction == MOVEMENT_DIRECTION.Y)
                    currentPosition = getAverageYDriveTrainEncoder();
                if (movement_direction == MOVEMENT_DIRECTION.R)
                    currentPosition = getAverageRDriveTrainEncoder();

                adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startPosition, currentPosition, neededPosition);

                if (movement_direction == MOVEMENT_DIRECTION.X)
                    mecPowerDrive(adjustedMotorPower, 0, 0);
                if (movement_direction == MOVEMENT_DIRECTION.Y)
                    mecPowerDrive(0, adjustedMotorPower, 0);
                if (movement_direction == MOVEMENT_DIRECTION.R)
                    mecPowerDrive(0, 0, adjustedMotorPower);

            } while ((currentPosition > neededPosition) && linearOpMode.opModeIsActive());

        } else {
            //no where to move in here.
        }
    }

    //eliminates residual forces
    public void stopMotorsAndWait(double seconds) {
        mecPowerDrive(0, 0, 0);
        linearOpMode.sleep((int) (seconds * 1000));
    }

    //sets the intake power and velocity
    public void setIntakePower(double intakePower) {
        if (intakeIsOpen()) {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(-intakePower);
        } else {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(intakePower);
        }

    }

    public void setIntakeVelocity(double intakeVelocity) {
        if (intakeIsOpen()) {
            rightIntakeMotor.setVelocity(intakeVelocity);
            leftIntakeMotor.setVelocity(-intakeVelocity);
        } else {
            rightIntakeMotor.setVelocity(intakeVelocity);
            leftIntakeMotor.setVelocity(intakeVelocity);
        }
    }

    //sets the lift power and veloctiy
    public void setLiftPower(double liftPower) {
        liftMotor.setPower(liftPower);
    }

    public int getLiftEncoder() {
        return liftMotor.getCurrentPosition();
    }

    //sets the arm power and velocity
    public void setArmPower(double armPower) {
        armMotor.setPower(armPower);
    }

    public int getArmEncoder() {
        return armMotor.getCurrentPosition();
    }

    public void closeGrabber() {
        blockGrabberServo.setPosition(.85);
    }

    public void openGrabber() {
        blockGrabberServo.setPosition(.55);
    }

    public void closeLatch() {
        leftLatchServo.setPosition(.3);
        rightLatchServo.setPosition(.45);
    }

    public void openLatch() {
        leftLatchServo.setPosition(.5);
        rightLatchServo.setPosition(.65);
    }
}