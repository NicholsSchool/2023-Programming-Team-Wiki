package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@TeleOp(name="'swerve'", group="Iterative Opmode")
public class MecanumTeleopTest extends OpMode
{
    private MecanumDriverTest driver = new MecanumDriverTest();
    private ElapsedTime runtime = new ElapsedTime();
    double forward;
    double strafe;
    double turn;
    boolean fieldOriented = true;
    double lastTurn;
    private boolean autoAlign;
    private double desiredAngle;
    private double timeCounter;
    
    
    //init hardware map from XHardwareMap
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        driver.init(hardwareMap);
        
        desiredAngle = 0.0;
        autoAlign = false;
        lastTurn = 0.1;
        turn = 0;
        timeCounter = 0;
        
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
        
        
        if(gamepad1.x){
            fieldOriented = true;
        }
        if(gamepad1.y){
            fieldOriented = false;
        }
        
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        
        if(turn == 0.0){
            if(lastTurn != 0.0){
                timeCounter = runtime.time();
            }
            
        }else{
            timeCounter = 0.0;
            autoAlign = false;
        }
        
        if(timeCounter != 0.0 && runtime.time() - timeCounter >= 0.1){
            autoAlign = true;
            desiredAngle = driver.getHeading();
            timeCounter = 0.0;
            
            
        }

        driver.drive(forward, strafe, turn, autoAlign, desiredAngle);
        
        lastTurn = turn;
        
        if(gamepad1.a || fieldOriented == false){
            driver.resetIMU();
        }
        
        telemetry.addData("field oriented",fieldOriented);
        telemetry.addData("autoAlign",autoAlign);
        telemetry.addData("headingCorrect",driver.headingCorrect(desiredAngle));
        telemetry.addData("desiredAngle",desiredAngle);
        
    }

    @Override
    public void stop() {
    }
    
    
}


    