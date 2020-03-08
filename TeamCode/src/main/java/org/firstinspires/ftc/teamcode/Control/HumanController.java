package org.firstinspires.ftc.teamcode.Control;

/**
 * Created by Will McCormick, 2019-2020
 * Contains (two?) drive profiles for the controller, one is linear and the other has smoothed input
 */
public class HumanController {

    public HumanController(double minimumPower, double maximumPower) {
        this.minimumPower = minimumPower;
        this.maximumPower = maximumPower;
    }

    double minimumPower;
    double maximumPower;

//    public static final double humanController(double input) {
//        double output = 0;
//
//        if (input > .075) {
//            output = .5 * Math.log10(input - 0.05) + 1.012;
//        } else if (input >= -.075 && input <= .075) {
//            output = 500 * input * input * input;
//        } else if (input < -.075) {
//            output = -.5 * Math.log10(-input - 0.05) - 1.012;
//        }
//
//        return output;
//    }

    public final double linearDriveProfile(double inputPower){
        if(inputPower == 0){
            return 0;
        }
        else if(inputPower > 0){
            return ((1 - minimumPower)/(maximumPower - 0)) * inputPower + minimumPower;
        }
        else if(inputPower < 0){
            return ((1 - minimumPower)/(maximumPower - 0)) * inputPower - minimumPower;
        }
        else{
            return 0;
        }
    }

    //Neverest Claims that the motor can freeload 160 rpm.
    //given that the motors will in all likelihood be understress, I'm going to assume that the robot can do at most 10 rpm

}