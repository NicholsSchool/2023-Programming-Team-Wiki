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
    
    
    //init hardware map from XHardwareMap
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
        
        
        if(gamepad1.dpad_up){
            fieldOriented = true;
        }
        if(gamepad1.dpad_down){
            fieldOriented = false;
        }
        
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        
        driver.drive(forward, strafe, turn, driver.autoCorrect(turn));
        
        if(gamepad1.a || fieldOriented == false){
            driver.resetIMU();
        }
        
        telemetry.addData("field oriented",fieldOriented);
        
    
    }

    @Override
    public void stop() {
    }
    
    
}


    