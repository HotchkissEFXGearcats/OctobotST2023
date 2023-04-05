package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

@TeleOp(name="Test IMU Orientation", group="Linear Opmode")
@Disabled

public class TestingImuOrientation extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    private IMU imu;
    private double heading;
    
    private YawPitchRollAngles orientation;
    private AngularVelocity angularVelocity;
    
    @Override
    public void runOpMode() {
        
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 45, 0, 0, 0));
        
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        orientation = imu.getRobotYawPitchRollAngles();
        idle();
        heading = orientation.getYaw(AngleUnit.RADIANS);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addData("Heading : ", "%.5f", heading);
        telemetry.update();
    
        waitForStart();
        runtime.reset();
    
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            orientation = imu.getRobotYawPitchRollAngles();
            idle();
            heading = orientation.getYaw(AngleUnit.RADIANS);
    
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine();
            telemetry.addData("Heading : ", "%.5f", heading);
            telemetry.update();
        }
    }
}
