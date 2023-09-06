package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@TeleOp(name="'swerve'", group="Iterative Opmode")
public class MecanumTeleopTest extends OpMode{

    //create instances and variables for movement
    private MecanumDriverTest driver = new MecanumDriverTest();
    private ElapsedTime runtime = new ElapsedTime();
    double forward;
    double strafe;
    double turn;
    float desiredAngle = 0;
    
    
    //init hardware map
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        driver.init(hardwareMap);
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    
    @Override
    public void start(){
        runtime.reset();
    }
    
    @Override
    public void loop() {
        
        //assign movement variables
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        
        //set quadrant angles to follow
        if(gamepad1.a){
            desiredAngle = (float) Math.PI;
        }else if(gamepad1.b){
            desiredAngle = (float) - Math.PI / 2;
        }else if(gamepad1.x){
            desiredAngle = (float) Math.PI / 2;
        }else if(gamepad1.y){
            desiredAngle = (float) 0;
        }

        //turn correction doesn't interfere with intentional turning
        if(turn > 0){
            desiredAngle = (float) (driver.getHeading() - 0.5);
        }else if (turn < 0){
            desiredAngle = (float) (driver.getHeading() + 0.5);
        }

        //drive method
        driver.drive(forward, strafe, turn, driver.turnCorrection(desiredAngle));
        
        
        telemetry.addData("angle",driver.getHeading() * 180 / Math.PI);
        telemetry.addData("desired angle",desiredAngle * 180 / Math.PI);
        telemetry.addData("turn correction",driver.turnCorrection(desiredAngle));
        telemetry.addData("difference", desiredAngle - driver.getHeading());
        telemetry.addData("turn", turn);
    }

    @Override
    public void stop() {
    }
    
    
}


    