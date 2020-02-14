package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.IO.*;

import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.*;


@TeleOp(name="PID Test")
@Disabled
public class PIDTest extends LinearOpMode{
    private RobotLinearOpMode robot;
    private RuntimeLogger logger = new RuntimeLogger("MotorVelocity");

    @Override
    public void runOpMode(){
        robot = new RobotLinearOpMode(this);

        DcMotorEx rf = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        double p = 1;
        double i = 0;
        double d = 0;
        double f = 0;
        double increment = 0;

        boolean modP = false;
        boolean modI = false;
        boolean modD = false;
        boolean modF = false;

        while(!isStopRequested()){
            robot.setDrivePIDCoefficients(p, i, d, f);
            telemetry.addData("Velocity", rf.getVelocity(AngleUnit.DEGREES));

            if(gamepad1.y){
                robot.mecanumPowerDrive(0, 0.2, 0);
                while(gamepad1.y){}
                robot.mecanumPowerDrive(0, 0, 0);
            }

            /*if(gamepad1.y){
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
            }*/

            if(modP){
                telemetry.addLine("Modifying P");
            } else if(modI){
                telemetry.addLine("Modifying I");
            } else if(modD){
                telemetry.addLine("Modifying D");
            } else if(modF){
                telemetry.addLine("Modifying F");
            }

            if(gamepad1.right_stick_button){
                while(gamepad1.right_stick_button){}
                increment *= 10;
            } else if(gamepad1.left_stick_button){
                while(gamepad1.left_stick_button){}
                increment /= 10;
            }

            if(increment > 1){
                increment = 1;
            } else if(increment < 0.01){
                increment = 0.01;
            }

            if(gamepad1.right_trigger > 0.3){
                while(gamepad1.right_trigger > 0.3){}

                if(modP){
                    p += increment;
                } else if(modI){
                    i += increment;
                } else if(modD){
                    d += increment;
                } else if(modF){
                    f += increment;
                }

            } else if(gamepad1.left_trigger > 0.3){
                while(gamepad1.left_trigger > 0.3){}

                if(modP){
                    p -= increment;
                } else if(modI){
                    i -= increment;
                } else if(modD){
                    d -= increment;
                } else if(modF){
                    f -= increment;
                }

            } else if(gamepad1.left_bumper && gamepad1.right_bumper){
                p = 1;
                i = 0;
                d = 0;
                f = 0;
            }

            if(gamepad1.dpad_up){
                modP = true;
                modI = false;
                modD = false;
                modF = false;
            } else if(gamepad1.dpad_down){
                modP = false;
                modI = false;
                modD = true;
                modF = false;
            } else if(gamepad1.dpad_right){
                modP = false;
                modI = true;
                modD = false;
                modF = false;
            } else if (gamepad1.dpad_left){
                modP = false;
                modI = false;
                modD = false;
                modF = true;
            }

            telemetry.addData("Increment", increment);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("F", f);
            telemetry.update();
        }

    }

}
