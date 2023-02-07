package org.firstinspires.ftc.teamcode.Templates;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UtilityClasses.DriveTrainController;
import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.UtilityClasses.IMUController;
import org.firstinspires.ftc.teamcode.UtilityClasses.LiftManager;
import org.firstinspires.ftc.teamcode.UtilityClasses.NavigationManager;

@TeleOp
public class TeleOp_Template extends LinearOpMode {

    private IMUController imu;
    private DriveTrainController driveTrainController;
    private static final float slowSpeed = .3f;
    private static final float regularSpeed = .7f;
    private static final float fastSpeed = 1f;
    public static float currentSpeed = .6f;

    @Override
    public void runOpMode()
    {
        Initialize();
        waitForStart();
        RunLoop();
    }

    public void RunLoop()
    {
        while (opModeIsActive())
        {
            SpeedAndDrive();

            imu.GetAngle();
            telemetry.addData("Angle : ", imu.heading);
            telemetry.update();
        }
    }

    public void SpeedAndDrive()
    {
        if(gamepad1.right_bumper)
        {
            currentSpeed = slowSpeed;
        }else if(gamepad1.left_stick_button)
        {
            currentSpeed = fastSpeed;
        }else
        {
            currentSpeed = regularSpeed;
        }
        if(driveTrainController.RSX >= 0.001 || driveTrainController.RSX <= -0.001 ||
                !(driveTrainController.LSX > 0.02f || driveTrainController.LSX < -0.02f) && !gamepad1.right_stick_button)
        {
            imu.ResetAngle();
            driveTrainController.UpdateDriveTrain(currentSpeed, StrafeCorrection());
        }
        else
        {
            driveTrainController.UpdateDriveTrain(currentSpeed,0);
        }
    }

    //region Initialization Code
    public void Initialize(){
        Blinker expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Blinker expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        imu = new IMUController(hardwareMap);
        driveTrainController = new DriveTrainController(gamepad1,hardwareMap);
        driveTrainController.InvertMotorDirection(Motor.backLeft);
        driveTrainController.InvertMotorDirection(Motor.frontLeft);
        imu.ResetAngle();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    //endregion
    //region Drift Correction
    public float StrafeCorrection()
    {
        float turnMod = (float) imu.AngleDeviation(0) / 20;
        turnMod = Range(turnMod,-1,1);
        return -turnMod;
    }
    //endregion
    //region lock input between 2 numbers
    private float Range(float in,float lower, float upper){
        float out = in;
        if(out < lower){
            out = lower;
        }
        if(out > upper){
            out = upper;
        }
        return out;
    }
    //endregion
}