package org.firstinspires.ftc.teamcode.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Control.Elapsed;
import org.firstinspires.ftc.teamcode.Control.PID;

import static java.lang.Math.E;
import static java.lang.Math.log;
import static java.lang.Math.pow;

public class SigmoidMotor {

    private DcMotor motor;
    private Stage currentStage;
    public PID decellerationError = new PID();

    private double acceleration;
    private double distance;

    private double precision = 5;
    private double maxSpeed = 1120 * 2.4;
    private double precision2 = .001;


    private Elapsed accelerationTime = new Elapsed();
    private Elapsed fullTime = new Elapsed();
    private Elapsed deccelerationTime = new Elapsed();

    private Elapsed fullEncoder = new Elapsed();
    private Elapsed deccelerationEncoder = new Elapsed();

    public SigmoidMotor(DcMotor motor, double acceleration) {
        this.motor = motor;
        this.acceleration = acceleration;
        this.currentStage = Stage.IDLE;
    }

    public double remainingDistance(){return distance - motor.getCurrentPosition();}

    public void start(double distance) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        currentStage = Stage.START;
        this.distance = distance;
    }

    //the actually sigmoid function that is used thoughout the code
    private double sigmoid(double x) {
        return 1 / (1 + pow(E, -x));
    }

    //the true velocity profiles that control the acceleration and the decelleration
    private double sigmoidAcceleration(double x){
        return sigmoid((acceleration*x) - precision);
    }
    private double sigmoidDecelleration(double x){
        return sigmoid((-acceleration*x) + precision);
    }

    //the decelleration needs to integrated, here is the math for that
    private double sigmoidDecellerationIntegral(double x){
        double output = pow(E, acceleration*x);
        output += pow(E, precision);
        output = x - log(output)/acceleration;
        return output;
    }

    //that is then converted to a definant integral as the math is not tied to a single point
    private double sigmoidDecellerationDefinateIntegral(double lowerBound, double upperBound){
        return sigmoidDecellerationIntegral(upperBound) - sigmoidDecellerationIntegral(lowerBound);
    }

    //uses the integration to calculate the distance that remains in the velocity function
    public double decellerationDistance(){
        return sigmoidDecellerationDefinateIntegral(0,2*precision/acceleration) * maxSpeed;
    }

    public void setVelocity(double time){
        double velocity = 0;

        switch (currentStage) {
            case START:
                currentStage = Stage.ACCELERATION;
                accelerationTime.start(time);

                break;
            case ACCELERATION:
                velocity = sigmoidAcceleration(accelerationTime.get(time));

                if(velocity > 1 - precision2){
                    currentStage = Stage.FULL_SPEED_AHEAD;
                    fullTime.start(time);
                    fullEncoder.start(motor.getCurrentPosition());
                }

                break;
            case FULL_SPEED_AHEAD:
                velocity = 1;
                this.maxSpeed = fullEncoder.get(motor.getCurrentPosition())/fullTime.get(time);

                if(decellerationDistance() >= remainingDistance()){
                    currentStage = Stage.DECCELERATION;
                    deccelerationEncoder.start(motor.getCurrentPosition());
                    deccelerationTime.start(time);

                }

                break;
            case DECCELERATION:
                double pidVelocity = 0;

                if(deccelerationTime.get(time) > .1 && !decellerationStart){
                    decellerationError.start();
                    decellerationStart = true;

                }
                if(decellerationStart){
                    decellerationError.onSensorChanged(sigmoidDecellerationDefinateIntegral(0,deccelerationTime.get(time))*maxSpeed - deccelerationEncoder.get(motor.getCurrentPosition()));
                    pidVelocity = decellerationError.get(0.05, 0.03, -.02, 1);
                }

                if( pidVelocity > -.5 && pidVelocity < .5) {
                    velocity = (1 - pidVelocity) * sigmoidDecelleration(deccelerationTime.get(time));
                }
                else velocity = sigmoidDecelleration(deccelerationTime.get(time));

                if(velocity < precision2)currentStage = Stage.END;

                break;
            case END:
                velocity = 0;
        }

        this.velocity = velocity;
        motor.setPower(velocity);
    }

    public double velocity;
    private boolean decellerationStart = false;

    public boolean isDone(){ return currentStage == Stage.END;}
    public String getCurrentStage(){ return currentStage.toString();}

    private enum Stage {
        IDLE,
        START,
        ACCELERATION,
        FULL_SPEED_AHEAD,
        DECCELERATION,
        END;
    }
}

