package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static java.lang.Math.PI;


// BEGIN CLASS //

public class DriveTrain {
    
    private HardwareMap hardwareMap;
    private LinearOpMode opModeTool;
    
    private ElapsedTime timer;
    
//    private Gyroscope imu;
    
    private IMU imu;
    private DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    
    
    
    private int k;
    private int wheelD,TPR; //Wheel Diameter = 82mm, Ticks per Revolution = 560 for Octobot
    private int avgPosition, positionLF, positionRF, positionLB, positionRB;
    private int start, finish, getGoing, slowDown, driveBuffer;
    private double setHeading, driveHeading, botHeading, headingOffset, previous, duration;
    
    private YawPitchRollAngles orientation;
    private AngularVelocity angularVelocity;
    
    private double kp, kAuto;
    private double scaleTurn;
    private double turn;
    private boolean flag, warning;
    
    //private Headings headings; 
    
    /* CLASS CONSTRUCTOR */
    public DriveTrain(HardwareMap hardwareMap, LinearOpMode opModeTool) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
        // initialize();
    }

    public DriveTrain(HardwareMap hardwareMap, LinearOpMode opModeTool, int wheelD, int TPR) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
        this.wheelD = wheelD;
        this.TPR = TPR;
       // initialize();
    }
    
// @TeleOp(name="HeidiDrive", group="Classes")    
    
    public void initialize() {
        
        //
        // Initialize IMU using Parameters
        //
        
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 45, 0, 0, 0));
        
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        orientation = imu.getRobotYawPitchRollAngles();
        setHeading = -orientation.getYaw(AngleUnit.RADIANS);
        
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        
        //
        // Initialize Motors
        //
        
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        positionLF = leftFrontMotor.getCurrentPosition();
        positionRF = rightFrontMotor.getCurrentPosition();
        positionLB = leftBackMotor.getCurrentPosition();
        positionRB = rightBackMotor.getCurrentPosition();
        
        //
        // Initialize scalars and flags
        //
       
        flag = true;
        warning = false;
        
        driveHeading = 0.0;
        botHeading = 0.0;
        headingOffset = 0.0;
        previous = 0.0;
        duration = 0.0;
        
        kp = 0.35;
        kAuto = 0.4;
        scaleTurn = 0.25;
        flag = true;
        
        driveBuffer = 300;
        
    }  // end method initialize
    
    
    public void testMotors(DriveVector vector) {
        leftFrontMotor.setPower(vector.mag * sin((vector.angle + PI/4)) );  //yPower
        rightFrontMotor.setPower(vector.mag * sin((vector.angle - PI/4)) );  //yPower
        leftBackMotor.setPower(vector.mag * sin((vector.angle - PI/4)) );  //xPower
        rightBackMotor.setPower(vector.mag * sin((vector.angle + PI/4)) );  //xPower
    }
    
    
    
    public double getHeading () {
        
        orientation = imu.getRobotYawPitchRollAngles();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        
        return -orientation.getYaw(AngleUnit.RADIANS);
        
    }
    
    
    public double getSetHeading () {
        return setHeading;
    }
    
    
    public int getPositionLF() {
        return leftFrontMotor.getCurrentPosition();
    }
    
    public int getPositionRF() {
        return rightFrontMotor.getCurrentPosition();
    }
    
    public int getPositionLB() {
        return leftBackMotor.getCurrentPosition();
    }
    
    public int getPositionRB() {
        return rightBackMotor.getCurrentPosition();
    }
    
    
    
    
    
    public boolean autonVector(DriveVector vector, int toPosition) {
        if (vector.mag < 0.05) {
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
        } else {
            while (abs(linearPosition(false)) < toPosition) {
                orientation = imu.getRobotYawPitchRollAngles();
                opModeTool.idle();
                botHeading = -orientation.getYaw(AngleUnit.RADIANS);
                headingOffset = (botHeading - setHeading);
                if (abs(headingOffset) < PI/4) {
                    turn = (kAuto * headingOffset) / (PI/4);
                } else if (abs(headingOffset) < PI/3) {
                    turn = kAuto * 1;
                } else {
                    leftFrontMotor.setPower(0.0);
                    rightFrontMotor.setPower(0.0);
                    leftBackMotor.setPower(0.0);
                    rightBackMotor.setPower(0.0);
                    warning = true;
                }  // end if-else turn < 0.05 case is not turning
                leftFrontMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/4)) - turn );  //yPower
                rightFrontMotor.setPower(vector.mag * sin((vector.angle - setHeading - PI/4)) - turn );  //yPower
                leftBackMotor.setPower(vector.mag * sin((vector.angle - setHeading - PI/4)) + turn );  //xPower
                rightBackMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/4)) + turn );  //xPower
                opModeTool.telemetry.addData("Position: ", linearPosition(false));
                opModeTool.telemetry.update();
            }  // end while
        }  // end if-else
        return warning;
    }  // end method autonVector
        
    public boolean distanceVector(DriveVector vector, int toCM){ //auton vector but with metric units.
        if (vector.mag < 0.05) {
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
        } else {
            while ((abs(linearPosition(false))*wheelD*PI)/(10*TPR) < toCM) {
                orientation = imu.getRobotYawPitchRollAngles();
                opModeTool.idle();
                botHeading = -orientation.getYaw(AngleUnit.RADIANS);
                headingOffset = (botHeading - setHeading);
                if (abs(headingOffset) < PI/4) {
                    turn = (kAuto * headingOffset) / (PI/4);
                } else if (abs(headingOffset) < PI/3) {
                    turn = kAuto * 1;
                } else {
                    leftFrontMotor.setPower(0.0);
                    rightFrontMotor.setPower(0.0);
                    leftBackMotor.setPower(0.0);
                    rightBackMotor.setPower(0.0);
                    warning = true;
                }  // end if-else turn < 0.05 case is not turning
                leftFrontMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/4)) - turn );  //yPower
                rightFrontMotor.setPower(vector.mag * sin((vector.angle - setHeading - PI/4)) - turn );  //yPower
                leftBackMotor.setPower(vector.mag * sin((vector.angle - setHeading - PI/4)) + turn );  //xPower
                rightBackMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/4)) + turn );  //xPower
                opModeTool.telemetry.addData("Position: ", linearPosition(false));
                opModeTool.telemetry.update();
            }  // end while
        }  // end if-else
        return warning;
    }
    public int linearPosition(boolean reset) {
        
        if (reset) {
            
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
            rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        
            leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
        } else {
            
            positionLF = leftFrontMotor.getCurrentPosition();
            positionRF = rightFrontMotor.getCurrentPosition();
            positionLB = leftBackMotor.getCurrentPosition();
            positionRB = rightBackMotor.getCurrentPosition();
            
            avgPosition = (int)((abs(positionLF) + abs(positionRF) + abs(positionLB) + abs(positionRB))/4);
            
        } // end if-else
        
        return avgPosition;
        
    }  // end method drivePosition
    
    
    // 
    // Stop the bot
    //
    
    public int stop() {
        
        leftFrontMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
        
        positionLF = leftFrontMotor.getCurrentPosition();
        positionRF = rightFrontMotor.getCurrentPosition();
        positionLB = leftBackMotor.getCurrentPosition();
        positionRB = rightBackMotor.getCurrentPosition();
            
        avgPosition = (int)((abs(positionLF) + abs(positionRF) + abs(positionLB) + abs(positionRB))/4);
        return avgPosition;
    }
    
    
    // 
    // Turn to heading
    //
    
    public double turnTo(double turnPower, double toHeading) {
        orientation = imu.getRobotYawPitchRollAngles();
        opModeTool.idle();
        botHeading = -orientation.getYaw(AngleUnit.RADIANS);
        while (abs(botHeading - toHeading) > (PI/180)) {
            leftFrontMotor.setPower(turnPower);
            rightFrontMotor.setPower(turnPower);
            leftBackMotor.setPower(-turnPower);
            rightBackMotor.setPower(-turnPower);
            orientation = imu.getRobotYawPitchRollAngles();
            opModeTool.idle();
            botHeading = -orientation.getYaw(AngleUnit.RADIANS);
        }  // end while
        leftFrontMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
        orientation = imu.getRobotYawPitchRollAngles();
        opModeTool.idle();
        setHeading = -orientation.getYaw(AngleUnit.RADIANS);
        return botHeading;
    }  // end method turn

    public double turnToDeg(double turnPower, double deg) { //A lot like turnTo, but made so a normal person can use it. (Takes degrees instead of radians)
        orientation = imu.getRobotYawPitchRollAngles();
        opModeTool.idle();
        botHeading = -orientation.getYaw(AngleUnit.RADIANS);
        deg = deg*(PI/180);
        while (abs(botHeading - deg) > (PI/180)) {
            leftFrontMotor.setPower(turnPower);
            rightFrontMotor.setPower(turnPower);
            leftBackMotor.setPower(-turnPower);
            rightBackMotor.setPower(-turnPower);
            orientation = imu.getRobotYawPitchRollAngles();
            opModeTool.idle();
            botHeading = -orientation.getYaw(AngleUnit.RADIANS);
        }  // end while
        leftFrontMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
        orientation = imu.getRobotYawPitchRollAngles();
        opModeTool.idle();
        setHeading = -orientation.getYaw(AngleUnit.RADIANS);
        return botHeading;
    }  // end method turn

    
    public boolean autonVector2(DriveVector vector, int toPosition) {
        if (vector.mag < 0.05) {
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
        } else {
            while (abs(linearPosition(false)) < toPosition) {
                leftFrontMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/4)) );  //yPower
                rightFrontMotor.setPower(vector.mag * sin((vector.angle - setHeading - PI/4)) );  //yPower
                leftBackMotor.setPower(vector.mag * sin((vector.angle - setHeading - PI/4)) );  //xPower
                rightBackMotor.setPower(vector.mag * sin((vector.angle - setHeading + PI/4)) );  //xPower
                opModeTool.telemetry.addData("Position: ", linearPosition(false));
                opModeTool.telemetry.update();
            }  // end while
        }  // end if-else
        return warning;
    }  // end method autonVector
    
    
    
    
}  // end class







