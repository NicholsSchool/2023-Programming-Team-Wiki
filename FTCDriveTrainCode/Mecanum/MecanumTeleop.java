package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@TeleOp(name="'swerve'", group="Iterative Opmode")
public class MecanumTeleop extends OpMode
{
    private MecanumDriver driver = new MecanumDriver();
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

        driver.drive(forward, strafe, turn);
        
        if(gamepad1.a || fieldOriented == false){
            driver.resetIMU();
        }
        
        telemetry.addData("field oriented",fieldOriented);
        
    }

    @Override
    public void stop() {
    }
    
    
}


    