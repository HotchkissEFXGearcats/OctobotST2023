package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Test for auton", group="Linear Opmode")

public class TestingAutonFunctions extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer;
    private Servo roller;
    private DriveVector vector;
    private DriveTrain drive;
    private Lift lift;
    private SensorNetwork sensors;
    private NormalizedRGBA cone;
    private int spot;


    int readCone(){
        int park = 2;
        cone = sensors.getForwardColors();
        timer.reset();

        while ((cone.alpha < 0.300)) {           // && opModeIsActive
            cone = sensors.getForwardColors();
            if (timer.milliseconds() > 2000.0) {break;}   // || !opModeTool.opModeIsActive()
        }

        if ((cone.blue > cone.green) && (cone.blue > cone.red)) {
            //1
            park = 1;
        } else if ((cone.red > cone.green) && (cone.red > cone.blue)) {
            //2
            park = 2;
        } else if ((cone.green > cone.red) && (cone.green > cone.blue)) {
            //3
            park = 3;
        }

        telemetry.addData("Zone  : ", spot);
        telemetry.update();

        return park;
    }

    @Override
    public void runOpMode() {
        
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        
        roller = hardwareMap.get(Servo.class, "roller");
        
        vector = new DriveVector();
        vector.initialize();
        
        drive = new DriveTrain(hardwareMap, this, 82, 560);
        drive.initialize();
        
        lift = new Lift(hardwareMap, this);
        lift.initialize(0.15, 0.5, 0.5);
        
        sensors = new SensorNetwork(hardwareMap, this);
        sensors.initialize(7, 7, 7);
        
        spot = 0;
        
        cone = sensors.getForwardColors();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addData("Red   : ", "%.3f", cone.red);
        telemetry.addData("Green : ", "%.3f", cone.green);
        telemetry.addData("Blue  : ", "%.3f", cone.blue);
        telemetry.update();
        
        telemetry.addData("Status", "Initialized Park w/ Color");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //Write Auton here:
        vector.mag = 0.2;
        vector.angle = 0.0;
        drive.autonVector(vector,100);
        sleep(100);
        vector.mag = 0.2;
        vector.angle = 0.0;
        drive.autonVector(vector,100);
        lift.pickUp();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine();
            //telemetry.addData("Encoder - Left Front  : ", drive.getPositionLF());
            //telemetry.addData("Encoder - Right Front : ", drive.getPositionRF());
            //telemetry.addData("Encoder - Left Back   : ", drive.getPositionLB());
            //telemetry.addData("Encoder - Right Back  : ", drive.getPositionRB());
            //telemetry.addLine();
            //telemetry.addData("Encoder - AVERAGE  : ", drive.getPositionRB());
            //telemetry.addLine();
            //telemetry.addData("Set Heading : ", "%.4f", drive.getSetHeading());
            //telemetry.addData("Bot Heading : ", "%.4f", drive.getHeading());
            
            telemetry.addLine();
            telemetry.addData("Alpha  : ", "%.3f", cone.alpha);
            telemetry.addLine();
            telemetry.addData("Red   : ", "%.3f", cone.red);
            telemetry.addData("Green : ", "%.3f", cone.green);
            telemetry.addData("Blue  : ", "%.3f", cone.blue);
            telemetry.addLine();
            telemetry.addData("Zone  : ", spot);
            
            telemetry.update();
        }
    }
}
