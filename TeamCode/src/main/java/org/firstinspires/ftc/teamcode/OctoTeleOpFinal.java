package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class OctoTeleOpFinal extends LinearOpMode{
   DcMotor rightBack, leftBack, leftFront, rightFront;
    DcMotor leftLift, rightLift;
    Blinker control_Hub, expansion_Hub_2;
    BNO055IMU c_imu, e_imu;
    Servo roller;
    DistanceSensor distL, distR, front;
    
    double angle, rawAngle, offset;
    double frontAngle, sideAngle;
    
    long headlessWait;
    boolean headless;
    
    Lift lift;

    @Override
    public void runOpMode() {
        motorSetup();
        gyroSetup();
        miscSetup();
        lift = new Lift(hardwareMap, this);
        lift.initialize(0.15,0.95,0.65); //end limit slow speed, max lift speed, manual speed
        
        waitForStart();
        lift.start();
        while (opModeIsActive()) { 
            servoPower();
            if (gamepad1.y) {
                offset();
            } else if (gamepad1.x) {
                if ((getTime()) - headlessWait >= 500) {
                    headless = !headless;
                    headlessWait = getTime();
                }
            }
            
            if (gamepad2.left_bumper){
                roller.setPosition(0.25);
                sleep(1300);
                roller.setPosition(0);
            }
            
            telemetry.addData("DistL", distL.getDistance(DistanceUnit.MM));
            telemetry.addData("DistR", distR.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance", front.getDistance(DistanceUnit.MM));
            telemetry.addData("Lift ", leftLift.getCurrentPosition());
            telemetry.update();
            
            getAngle();
            generalPower();
            
        }
        lift.terminateLift();

    }
    
    void generalPower() {
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double stickAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        
        final double turnPower = 0.40;//0.5
        double rightX = gamepad1.right_stick_x * turnPower;
        
        //double speedOffset = (gamepad1.right_trigger)+0.50;
        double speedOffset = (1-gamepad1.right_trigger)+0.2;
        if(gamepad1.right_bumper) {speedOffset = 0.2;}
        
        final double speedPower = 0.7;//0.65
        speedOffset *= speedPower;
        
        if (Math.abs(stickAngle - Math.PI/2) < Math.PI/12) {
            stickAngle = Math.PI/2;
        } else if (Math.abs(stickAngle + Math.PI/2) < Math.PI/12) {
            stickAngle = -Math.PI/2;
        }
        
        stickAngle += Math.PI/2;
        
        if (headless)
        {
            stickAngle += angle*(Math.PI/180);
            light(0xff0000); // red
        } else {
            light(0x000000); //black/nothing
        }
        
        rightFront.setPower((
            r * Math.sin(stickAngle - (Math.PI/4)) - rightX ) * speedOffset);
        rightBack.setPower((
            r * Math.sin(stickAngle + (Math.PI/4)) + rightX ) * speedOffset);
        leftBack.setPower((
            r * Math.sin(stickAngle - (Math.PI/4)) + rightX ) * speedOffset);
        leftFront.setPower((
            r * Math.sin(stickAngle + (Math.PI/4)) - rightX ) * speedOffset);
    }
    
    void servoPower(){
        if (gamepad2.right_bumper) {
            roller.setPosition(0.75); //0.5-0.75
        } else if (gamepad2.right_trigger > 0) {
            roller.setPosition(0.25);
        } 
        else{
            roller.setPosition(0);
        }
    }
    
    int leftLiftMinPosition = 0;
    int rightLiftMinPosition = 0;
    
    //The distance to the top of the lift from the bottom, in encoder units
    int liftLength = 4200;
    
    void liftPower() {
        double power = -gamepad2.right_stick_y;
    
        int leftLiftExtension = leftLift.getCurrentPosition() - leftLiftMinPosition;
        int rightLiftExtension = rightLift.getCurrentPosition() - rightLiftMinPosition;
        
        telemetry.addData("Left extention", leftLiftExtension);
        telemetry.addData("Right extention", rightLiftExtension);
        
        leftLift.setPower(-gamepad2.right_stick_y);
        rightLift.setPower(-gamepad2.right_stick_y);
    }
    
    void resetLiftMin() {
        leftLiftMinPosition = leftLift.getCurrentPosition();
        rightLiftMinPosition = rightLift.getCurrentPosition();
    }
    
    public void getAngle() {
        Orientation c_angles = c_imu.getAngularOrientation(
        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        Orientation e_angles = e_imu.getAngularOrientation(
        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        frontAngle = c_angles.secondAngle;
        sideAngle = c_angles.thirdAngle;
    
        rawAngle = (c_angles.firstAngle + e_angles.firstAngle)/2;
        //rawAngle = c_angles.firstAngle;
        angle = rawAngle + offset;
    }
    
    //set a new "0" heading for the robot
    public void offset() {
        offset = -rawAngle;
    }
    
    //in milliseconds
    public static long getTime() {
        return (System.nanoTime()) / 1000000;
    }
    
    //sets the color of the built in leds 
    public void light(int color) {
        control_Hub.setConstant(color);
        expansion_Hub_2.setConstant(color);
    }
    
    void motorSetup() {
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //leftBack.setDirection(DcMotor.Direction.REVERSE);
        
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        
        roller = hardwareMap.get(Servo.class, "roller");
    }
    
    public void gyroSetup() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled   = false;

        c_imu = hardwareMap.get(BNO055IMU.class, "imu");
        e_imu = hardwareMap.get(BNO055IMU.class, "imu2");

        c_imu.initialize(parameters);
        e_imu.initialize(parameters);
    }
    
    public void miscSetup() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        
        distL = hardwareMap.get(DistanceSensor.class, "distL");
        distR = hardwareMap.get(DistanceSensor.class, "distR");
        
        front = hardwareMap.get(DistanceSensor.class, "front");
        //claw = hardwareMap.get(Servo.class, "claw");
    }
}