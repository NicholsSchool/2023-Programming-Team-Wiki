package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@TeleOp(name="MecanumTeleopTest", group="Iterative Opmode")
public class MecanumTeleopTest extends OpMode
{
    private MecanumDriverTest driver = new MecanumDriverTest();
    double forward;
    double strafe;
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
        
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        driver.drive(forward, strafe, turn);
        
    }

    @Override
    public void stop() {
    }
    
    
}


    