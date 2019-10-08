package org.firstinspires.ftc.teamcode.Control;

/**
 * Created by Will McCormick, 2019-2020
 * Logic class for a boolean that, once changed, will remain true until it is reset
 */
public class EverHit {

    boolean everHit;

    public EverHit(boolean startValue){
        this.everHit = startValue;
    }
    public EverHit(){
        everHit = false;
    }

    public void update(boolean newValue){
        if(newValue) everHit = true;
    }

    public boolean isHit(){
        return everHit;
    }

    public void reset(){
        everHit = false;
    }
}
