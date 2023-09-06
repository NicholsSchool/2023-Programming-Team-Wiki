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
            desiredAngle = Math.PI;
        }else if(gamepad1.b){
            desiredAngle = - Math.PI / 2;
        }else if(gamepad1.x){
            desiredAngle = Math.PI / 2;
        }else if(gamepad1.y){
            desiredAngle = 0;
        }

        //turn correction doesn't interfere with intentional turning
        if(turn != 0){
            desiredAngle = driver.getHeading();
        }

        //drive method
        driver.drive(forward, strafe, turn, driver.turnCorrection(desiredAngle));
        
        
        telemetry.addData("field oriented",fieldOriented);
        telemetry.addData("angle",driver.getHeading() * 180 / Math.PI);
        telemetry.addData("autocorrect",driver.autoCorrect(turn));
    
    }

    @Override
    public void stop() {
    }
    
    
}


    