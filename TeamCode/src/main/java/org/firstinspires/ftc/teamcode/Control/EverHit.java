package org.firstinspires.ftc.teamcode.Control;

public class EverHit {

    boolean everHit;

    EverHit(boolean startValue){
        this.everHit = startValue;
    }
    public EverHit(){
        everHit = false;
    }

    public void update(boolean newValue){
        if(newValue = true)everHit = true;
    }

    public boolean get(){
        return everHit;
    }

    public void reset(){
        everHit = false;
    }
}
