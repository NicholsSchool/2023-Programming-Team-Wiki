package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.Range;


public class XDriver{
    //declaring variables, for tweaking use sensitivity and top speed
    public DcMotor redMotor , blueMotor, greenMotor, yellowMotor;

    double positionPower = 0.7;

    HardwareMap hwMap = null;

    private ElapsedTime runtime = new ElapsedTime();

    
    public void init( HardwareMap ahwMap ) 
    {
        // Save reference to Hardware map
        HardwareMap hwMap = ahwMap;  
        
        redMotor  = hwMap.get(DcMotor.class, "redMotor");
        greenMotor  = hwMap.get(DcMotor.class, "greenMotor");
        yellowMotor = hwMap.get(DcMotor.class, "yellowMotor");
        blueMotor = hwMap.get(DcMotor.class, "blueMotor");
        
        yellowMotor.setDirection(DcMotor.Direction.FORWARD);
        redMotor.setDirection(DcMotor.Direction.FORWARD);
        blueMotor.setDirection(DcMotor.Direction.REVERSE);
        greenMotor.setDirection(DcMotor.Direction.REVERSE);

        yellowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        redMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        greenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blueMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        yellowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        redMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        greenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blueMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    } 
        public void drive(double forward, double strafe, double turn){
            
            yellowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            redMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            greenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            blueMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double yellowPower = forward - turn;
            double greenPower = forward + turn;
            double redPower = strafe - turn;
            double bluePower = strafe + turn;
            blueMotor.setPower(bluePower);
            redMotor.setPower(redPower);
            yellowMotor.setPower(yellowPower);
            greenMotor.setPower(greenPower);
        }

        public void driveToPosition(int red, int green, int blue, int yellow){
            redMotor.setTargetPosition(red);
            greenMotor.setTargetPosition(green);
            blueMotor.setTargetPosition(blue);
            yellowMotor.setTargetPosition(yellow);

            yellowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            redMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            blueMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            blueMotor.setPower(positionPower);
            redMotor.setPower(positionPower);
            yellowMotor.setPower(positionPower);
            greenMotor.setPower(positionPower);


        }
        
}
