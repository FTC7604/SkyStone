package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.IO.*;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.*;


@TeleOp(name="PID Test")
public class PIDTest extends LinearOpMode{
    private RobotLinearOpMode robot;

    private PropertiesLoader loader = new PropertiesLoader("Robot");
    private double P = loader.getDoubleProperty("P");
    private double I = loader.getDoubleProperty("I");
    private double D = loader.getDoubleProperty("D");
    private double F = loader.getDoubleProperty("F");

    private RunMode runMode = RunMode.RUN_USING_ENCODER;

    @Override
    public void runOpMode(){
        robot = new RobotLinearOpMode(this);

        DcMotorEx[] driveMotors = new DcMotorEx[4];

        driveMotors[0] = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        driveMotors[1] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        driveMotors[2] = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        driveMotors[3] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");
        driveMotors[0].setDirection(DcMotorEx.Direction.REVERSE);
        driveMotors[1].setDirection(DcMotorEx.Direction.FORWARD);
        driveMotors[2].setDirection(DcMotorEx.Direction.REVERSE);
        driveMotors[3].setDirection(DcMotorEx.Direction.FORWARD);

        for(int i = 0; i < 4; i++){
            driveMotors[i].setMode(runMode);
        }

        PIDFCoefficients[] newCoeffs = new PIDFCoefficients[4];

        for(int i = 0; i < newCoeffs.length; i++){
            newCoeffs[i] = new PIDFCoefficients(P, I, D, F);
        }

        setCoeffs(driveMotors, newCoeffs);
        PIDFCoefficients[] coeffs = getCoeffs(driveMotors);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        while(!isStopRequested()){

            if(gamepad1.y){
                while(gamepad1.y){}
                robot.moveByInchesFast(24, FORWARD);
            } else if(gamepad1.a){
                while(gamepad1.a){}
                robot.moveByInchesFast(-24, FORWARD);
            } else if(gamepad1.x){
                while(gamepad1.x){}
                robot.moveByInchesFast(-24, STRAFE);
            } else if(gamepad1.b){
                while(gamepad1.b){}
                robot.moveByInchesFast(24, STRAFE);
            }

            telemetry.addLine("LF");
            printCoeffs(coeffs[0]);
            telemetry.addLine("RF");
            printCoeffs(coeffs[1]);
            telemetry.addLine("LB");
            printCoeffs(coeffs[2]);
            telemetry.addLine("RB");
            printCoeffs(coeffs[3]);
            telemetry.update();
        }

    }

    private PIDFCoefficients[] getCoeffs(DcMotorEx[] motors){
        PIDFCoefficients[] coeffs = new PIDFCoefficients[motors.length];

        for(int i = 0; i < motors.length; i++){
            coeffs[i] = motors[i].getPIDFCoefficients(runMode);
        }

        return coeffs;
    }

    private void setCoeffs(DcMotorEx[] motors, PIDFCoefficients[] coeffs){

        for(int i = 0; i < motors.length; i++){
            motors[i].setPIDFCoefficients(runMode, coeffs[i]);
        }

    }

    private void printCoeffs(PIDFCoefficients coeffs){
        telemetry.addData("P", coeffs.p);
        telemetry.addData("I", coeffs.i);
        telemetry.addData("D", coeffs.d);
        telemetry.addData("F", coeffs.f);
    }

}
