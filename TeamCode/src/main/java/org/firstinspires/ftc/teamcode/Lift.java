package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.TouchSensor;

import static java.lang.Math.abs;

// BEGIN CLASS //
public class Lift extends Thread {
    
    /* CLASS MEMBERS */
    
    private HardwareMap hardwareMap;
    private LinearOpMode opModeTool;
    
    private DcMotor motorL, motorR;
    private Servo roller;
    private int positionL, positionR, zero, max, threshold, buffer, bottomBuffer, topBuffer;
    private int currentPos, highPos, midPos, lowPos, hoverPos, sidewall, poleBuffer, pickup, pickupPos;
    private double fast, slow, mediumMove;
    private boolean terminate;
    
     public Lift(HardwareMap hardwareMap, LinearOpMode opModeTool) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
       // initialize();
    } // end constructor
    
    public int initialize(double lowSpeed, double highSpeed, double moveSpeed) {
        
        motorL = hardwareMap.get(DcMotor.class, "leftLift");
        motorR = hardwareMap.get(DcMotor.class, "rightLift");
        roller = hardwareMap.get(Servo.class, "roller");
        
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL.setDirection(DcMotor.Direction.FORWARD);
        motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setDirection(DcMotor.Direction.FORWARD);
        motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        max = 2450;
        
        bottomBuffer = 250;
        topBuffer = 250;  // was 400
        
        highPos = 2400;
        midPos = 1720;
        lowPos = 1050;
        sidewall = 695;
        hoverPos = 317;  // 370 for floor cones
        poleBuffer = 150;
        
        pickup = 200;
        
        //zero = -motor.getCurrentPosition()-50;
        threshold = zero + bottomBuffer;
        buffer = max - topBuffer;
        //position = -motor.getCurrentPosition();
        
        fast = highSpeed;
        slow = lowSpeed;
        mediumMove = moveSpeed;
        
        terminate = false;

        //return zero;
        return -99;
        
    } // end method initialize
    
    
    
    public void moveManual(double power) {
        motorL.setPower(mediumMove * power);
        motorR.setPower(mediumMove * power);
    }
    

    
    public int getPositionL() {
        return motorL.getCurrentPosition();
    }
    
    
    public int getPositionR() {
        return motorR.getCurrentPosition();
    }
    
    //********************************
    
    
    public void motorSetPower(double power) {
        motorL.setPower(power);
        motorR.setPower(power);
    }
    
    
    /* METHOD MOVE - WITH LIMITS */
    public int move(double power) {
        if (power < 0.0) {
            up(mediumMove * power);
        } else if (power > 0.0) {
            down(mediumMove * power);
        } else {
            motorSetPower(0.0);
        }
        return -motorL.getCurrentPosition();
    }  // end method move

    
    /* METHOD UP */
    private void up(double power) {
        if (-motorL.getCurrentPosition() < max) {
            if (-motorL.getCurrentPosition() < buffer) {
                motorSetPower(fast * power);
            } else {
                motorSetPower(slow * power);
            }
        } else {
            motorSetPower(0.0);
        }  // end if-else
    }  // end method up
    
    
    /* METHOD DOWN */
    private void down(double power) {
        if (-motorL.getCurrentPosition() > zero) {
            if (-motorL.getCurrentPosition() > threshold) {
                motorSetPower(fast * power);
            } else {
                motorSetPower(slow * power);
            }
        } else {
            motorSetPower(0.0);
        }  // end if-else
    }  // end method down


    /* LIFT TO TOP SIGNAL POLE */
    public int topSignal() {
        currentPos = -motorL.getCurrentPosition();
        while (currentPos < (highPos)) {
            if (currentPos < (highPos - poleBuffer)) {
                motorSetPower(-fast); // change this to fast
            } else {
                motorSetPower(-slow);
            }
            currentPos = -motorL.getCurrentPosition();
        }  // end while
        motorSetPower(0.0);
        return -motorL.getCurrentPosition();
    }  // end method topSignal


    /* LIFT TO MIDDLE SIGNAL POLE */
    public int midSignal() {
        currentPos = -motorL.getCurrentPosition();
        if (currentPos < midPos) {
            while (currentPos < midPos) {
                if (currentPos < (midPos - poleBuffer)) {
                    motorSetPower(-fast); // change this to fast
                } else {
                    motorSetPower(-slow);
                }
            currentPos = -motorL.getCurrentPosition();
            }  // end while
        } else if (currentPos > midPos) {
            while (currentPos > midPos) {
                if (currentPos > (midPos + poleBuffer)) {
                    motorSetPower(fast); // change this to fast
                } else {
                    motorSetPower(slow);
                }
            currentPos = -motorL.getCurrentPosition();
            }  // end while
        }  // end if-else
        motorSetPower(0.0);
        return -motorL.getCurrentPosition();
    }  // end method midSignal
    

    /* LIFT TO LOW SIGNAL POLE */
    public int lowSignal() {
        currentPos = -motorL.getCurrentPosition();
        if (currentPos < lowPos) {
            while (currentPos < lowPos) {
                if (currentPos < (lowPos - poleBuffer)) {
                    motorSetPower(-fast); // change this to fast
                } else {
                    motorSetPower(-slow);
                }
                currentPos = -motorL.getCurrentPosition();
            }  // end while
        
        } else if (currentPos > lowPos) {
            while (currentPos > lowPos) {
                if (currentPos > (lowPos + poleBuffer)) {
                    motorSetPower(fast); // change this to fast
                } else {
                    motorSetPower(slow);
                }
                currentPos = -motorL.getCurrentPosition();
            }  // end while
        }
        motorSetPower(0.0);
        return -motorL.getCurrentPosition();
    }  // end method lowSignal



public int sidePickup() {
    currentPos = -motorL.getCurrentPosition();
    if (currentPos < sidewall) {
        while (currentPos < sidewall) {
            motorSetPower(-slow);
            currentPos = -motorL.getCurrentPosition();
        }  // end while 
    } else if (currentPos > sidewall) {
        while (currentPos > sidewall) {
            if (currentPos > (sidewall + poleBuffer)) {
                motorSetPower(fast);
            } else {
                motorSetPower(slow);
            }
            currentPos = -motorL.getCurrentPosition();
        }  // end sidePickup 
    }
    motorSetPower(0.0);
    return -motorL.getCurrentPosition();
}  // end method hover



public int hover() {
    currentPos = -motorL.getCurrentPosition();
    if (currentPos < hoverPos) {
        while (currentPos < hoverPos) {
            motorSetPower(-slow);
            currentPos = -motorL.getCurrentPosition();
        }  // end while 
    } else if (currentPos > hoverPos) {
        while (currentPos > hoverPos) {
            if (currentPos > (hoverPos + poleBuffer)) {
                motorSetPower(fast);
            } else {
                motorSetPower(slow);
            }
            currentPos = -motorL.getCurrentPosition();
        }  // end while 
    }
    motorSetPower(0.0);
    return -motorL.getCurrentPosition();
}  // end method hover

public void pickUp(){
    currentPos = -motorL.getCurrentPosition();
    pickupPos = currentPos - pickup;
    roller.setPosition(0.25);
    if (currentPos > (pickup + 50)) {
        while (currentPos > pickupPos) {
            motorSetPower(mediumMove);
            currentPos = -motorL.getCurrentPosition();
        }  // end while
        currentPos = -motorL.getCurrentPosition();
        pickupPos = currentPos + pickup + 50;
        while (currentPos < pickupPos) {
            motorSetPower(-mediumMove);
            currentPos = -motorL.getCurrentPosition();
            if (currentPos == pickupPos - 50){
                roller.setPosition(0);
            }
        }  // end while
    } // end if
    motorSetPower(0.0);
}

public void outake(){
    long i = (System.nanoTime()) / 1000000;
    roller.setPosition(0.75);
    if(((System.nanoTime()) / 1000000)- i > 500) {
        roller.setPosition(0.5);
    }
}

public int resetToZero() {
    currentPos = -motorL.getCurrentPosition();
    if (currentPos > 0) {
        while (currentPos >0) {
            if (currentPos > 100) {
                motorSetPower(fast); // change this to fast
            } else {
                motorSetPower(slow);
            }
            currentPos = -motorL.getCurrentPosition();
        }  // end while 
    }
    motorSetPower(0.0);
    return -motorL.getCurrentPosition();
}


public void terminateLift() {
    terminate = true;
}


public void run() {
    while (!terminate && !motorL.isBusy()) {
        
        //liftPower = gamepad2.right_stick_y;
        //move(liftPower);
        moveManual(opModeTool.gamepad2.right_stick_y);
        
        if (opModeTool.gamepad2.y) {
            topSignal();
        } else if (opModeTool.gamepad2.x) {
            midSignal();
        } else if (opModeTool.gamepad2.a) {
            sidePickup();
        } else if (opModeTool.gamepad2.b) {
            lowSignal();
        } else if (opModeTool.gamepad2.dpad_down) {
            hover();
        } else if (opModeTool.gamepad2.dpad_up) {
            sidePickup();
        } else if (opModeTool.gamepad2.left_bumper) {
            currentPos = -motorL.getCurrentPosition();
            pickupPos = currentPos - pickup;
            if (currentPos > (pickup + 50)) {
                while (currentPos > pickupPos) {
                    motorSetPower(mediumMove);
                    currentPos = -motorL.getCurrentPosition();
                }  // end while 
                currentPos = -motorL.getCurrentPosition();
                pickupPos = currentPos + pickup + 50;
                while (currentPos < pickupPos) {
                    motorSetPower(-mediumMove);
                    currentPos = -motorL.getCurrentPosition();
                }  // end while 
            } // end if
            motorSetPower(0.0);
        }  // end if-else
        
    }  // end while
}  // end method run
    


}  // end class

