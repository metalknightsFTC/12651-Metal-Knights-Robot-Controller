package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSystemControl {

    public DcMotor[] motors;

    public Servo[] servos;

    HardwareMap hardwareMap;

    String[] motorNames;
    String[] servoNames;

    public SubSystemControl(HardwareMap hardwareMap,String[] motorNames,String[] servoNames)
    {
        this.hardwareMap = hardwareMap;
        this.motorNames = motorNames;
        this.servoNames = servoNames;
        if(motorNames!=null){
            ConfigureMotors(motorNames);
        }
        if(servoNames!=null&&servoNames.length>0){
            ConfigureServos(servoNames);
        }
    }

    public  void  ConfigureMotors(String[] motorNames)
    {
        motors = new DcMotor[motorNames.length];
        for (int ii = 0; ii < motorNames.length; ii++)
        {
            motors[ii] = hardwareMap.get(DcMotor.class, motorNames[ii]);
        }
    }

    public  void  ConfigureServos(String[] servoNames)
    {
        servos = new Servo[servoNames.length];
        for (int ii = 0; ii < servoNames.length; ii++)
        {
            servos[ii] = hardwareMap.get(Servo.class, servoNames[ii]);
        }
    }

    public DcMotor ManipulateMotor(String motorName){

        DcMotor returnMotor = null;

        for(int ii = 0; ii < motorNames.length; ii++){
            if (motorName.equals(motorNames[ii]))
            {
                returnMotor = motors[ii];
            }
        }
        return returnMotor;
    }

    public Servo ManipulateServo(String servoName){

        Servo returnServo = null;

        for(int ii = 0; ii < servoNames.length; ii++){
            if (servoName.equals(servoNames[ii]))
            {
                returnServo = servos[ii];
            }
        }
        return returnServo;
    }

}
