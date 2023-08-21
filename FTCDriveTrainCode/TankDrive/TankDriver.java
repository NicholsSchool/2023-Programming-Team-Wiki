package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.Range;

public class TankDriver{
    //declaring variables, for tweaking use sensitivity and top speed
    public DcMotor leftMotor, rightMotor;

    double positionPower = 0.7;

    HardwareMap hwMap = null;

    private ElapsedTime runtime = new ElapsedTime();

    
    public void init( HardwareMap ahwMap ){
        // Save reference to Hardware map
        HardwareMap hwMap = ahwMap;  
        
        leftMotor  = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor  = hwMap.get(DcMotor.class, "rightMotor");
        
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    } 
        public void drive(double forward, double turn){
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double rightPower = forward - turn;
            double leftPower = forward + turn;
            rightMotor.setPower(rightPower);
            leftMotor.setPower(leftPower);
        }

        public void driveToPosition(int left, int right){
            leftMotor.setTargetPosition(left);
            rightMotor.setTargetPosition(right);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            redMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftMotor.setPower(positionPower);
            rightMotor.setPower(positionPower);

        }
        
}
