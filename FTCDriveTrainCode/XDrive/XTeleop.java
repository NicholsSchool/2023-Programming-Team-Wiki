package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name="XTeleop", group="Iterative Opmode")
public class XTeleop extends OpMode
{
    // Declare OpMode members
    //make instances of elapsed time for acceleration and power up
    //also bring hardware map from XHardwareMap

    private XDriver driver = new XDriver();
    double angle;
    double power;
    double turn;
    
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
        
        angle = Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_x);
        power = Math.pow(Math.pow(gamepad1.left_stick_x,2) + Math.pow(gamepad1.left_stick_y,2),0.5); 
        turn = gamepad1.right_stick_x;
        driver.drive(angle, power, turn);
        
        telemetry.addData("angle",angle);
        if(gamepad1.a){
            driver.resetIMU();
        }
    }

    @Override
    public void stop() {
    }

}