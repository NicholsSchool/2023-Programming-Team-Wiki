package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;


public class MecanumDriverTest{
    //declaring variables, for tweaking use sensitivity and top speed
    public DcMotor frontLeftMotor , frontRightMotor, backLeftMotor, backRightMotor;
    public BNO055IMU imu;
    double positionPower = 0.7;
    double heading;

    HardwareMap hwMap = null;

    private ElapsedTime runtime = new ElapsedTime();
    
    public void init( HardwareMap ahwMap ) 
    {
        // Save reference to Hardware map
        HardwareMap hwMap = ahwMap;  
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        
        // Defining and Initializing IMU... Initializing it with the above Parameters...
        imu = hwMap.get( BNO055IMU.class, "IMU" );
        imu.initialize( parameters );
        
        
        frontLeftMotor  = hwMap.get(DcMotor.class, "leftMotorF");
        frontRightMotor  = hwMap.get(DcMotor.class, "rightMotorF");
        backLeftMotor = hwMap.get(DcMotor.class, "leftMotorB");
        backRightMotor = hwMap.get(DcMotor.class, "rightMotorB");
        
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        

    } 
        public void drive(double forward, double strafe, double turn){
            
            heading = imu.getAngularOrientation().firstAngle;
            
            double leftPowerF = Range.clip((Math.cos(heading) + Math.sin(heading)),-1,1) * (forward) - Range.clip((Math.cos(heading) - Math.sin(heading)),-1,1) * strafe - turn;
            double leftPowerB = Range.clip((Math.cos(heading) - Math.sin(heading)),-1,1) * (forward) + Range.clip((Math.cos(heading) + Math.sin(heading)),-1,1) * strafe - turn;
            double rightPowerF = Range.clip((Math.cos(heading) - Math.sin(heading)),-1,1) * (forward) + Range.clip((Math.cos(heading) + Math.sin(heading)),-1,1) * strafe + turn;
            double rightPowerB = Range.clip((Math.cos(heading) + Math.sin(heading)),-1,1) * (forward) - Range.clip((Math.cos(heading) - Math.sin(heading)),-1,1) * strafe + turn;
            frontLeftMotor.setPower(leftPowerF);
            frontRightMotor.setPower(rightPowerF);
            backLeftMotor.setPower(leftPowerB);
            backRightMotor.setPower(rightPowerB);
        }
        
        public float getHeading(){   
        return imu.getAngularOrientation().firstAngle;
    }
        
}
        
        
