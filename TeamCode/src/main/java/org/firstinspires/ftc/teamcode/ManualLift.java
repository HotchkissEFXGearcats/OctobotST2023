package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Manual Lift", group="Linear Opmode")
@Disabled

public class ManualLift extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    private double liftPower;
    

    @Override
    public void runOpMode() {
        
        Lift lift = new Lift(hardwareMap, this);
        lift.initialize(0.15, 0.50, 0.50);
        
        liftPower = 0.0;
        
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addData("Lift Position - LEFT  : ",  lift.getPositionL());
        telemetry.addData("Lift Position - RIGHT : ",  lift.getPositionR());
        telemetry.update();

        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            liftPower = gamepad2.right_stick_y;
            lift.moveManual(liftPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine();
            telemetry.addData("Lift Power : ", "%.2f",  liftPower);
            telemetry.addLine();
            telemetry.addData("Lift Position - LEFT  : ",  lift.getPositionL());
            telemetry.addData("Lift Position - RIGHT : ",  lift.getPositionR());
            telemetry.update();
        }  // end opmode while loop
        
    }  // end runopmode
}  // end class




