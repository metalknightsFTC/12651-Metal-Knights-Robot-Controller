package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Enums.Motor;

@TeleOp
public class ProjectAluminumKnight extends LinearOpMode {

    private Servo grabber;
    private IMUController imu;
    private DriveTrainCode driveTrainCode;
    private Servo verticalR;
    private Servo horizontalR;
    private LiftManager lift;
    private NavigationManager navSystem;
    private static final float slowSpeed = .3f;
    private static final float regularSpeed = .7f;
    private static final float fastSpeed = 1f;
    public static float currentSpeed = .6f;
    private float targetLockHeading;
    private boolean liftLimits = true;
    private boolean hasLocked = false;
    private float lastX = 0;
    private float currentX = 0;

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
            SetCameraAngle(0,.15);
            LiftRecovery();

            SpeedAndDrive();
            ManageLiftLevel();
            //Contact();
            //region Grabber Code
            if(gamepad1.left_bumper){
                grabber.setPosition(0.16f);
            }else{
                grabber.setPosition(.33f);
            }
            //endregion

            imu.GetAngle();
            telemetry.addData("Angle : ", imu.heading);
            telemetry.update();
            //endregion
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
        if(driveTrainCode.RSX >= 0.001 || driveTrainCode.RSX <= -0.001 ||
                !(driveTrainCode.LSX > 0.02f || driveTrainCode.LSX < -0.02f) && !gamepad1.right_stick_button)
        {
            imu.ResetAngle();
        }
        driveTrainCode.UpdateDriveTrain(currentSpeed, StrafeCorrection());
    }

    public void ManageLiftLevel()
    {
        //region lifter buttons
        //538
        if(liftLimits)
        {
            if (gamepad1.dpad_right) {
                lift.Lift(4);
            }
            if (gamepad1.dpad_left) {
                lift.Lift(2);
            }
            if (gamepad1.dpad_up) {
                lift.Lift(3);
            }
            if (gamepad1.dpad_down) {
                lift.Lift(1);
            }
            if (gamepad1.a) {
                lift.Lift(5);
            }
            if (gamepad1.x) {
                lift.Lift(6);
            }
            if (gamepad1.y) {
                lift.Lift(7);
            }
            if (gamepad1.b) {
                lift.Lift(0);
            }
        }
        //endregion
        //region lifter code
        lift.Lift((gamepad1.right_trigger - gamepad1.left_trigger)* 20f, liftLimits);
        //endregion
    }

    //region Initialization Code
    public void Initialize(){
        Blinker expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Blinker expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        imu = new IMUController(hardwareMap);
        lift = new LiftManager(hardwareMap);
        horizontalR = hardwareMap.get(Servo.class, "alignment");
        verticalR = hardwareMap.get(Servo.class, "pivot");
        grabber = hardwareMap.get(Servo.class,"grabber");
        //poleContact = hardwareMap.get(TouchSensor.class, "poleContact");
        driveTrainCode = new DriveTrainCode(gamepad1,hardwareMap);

        driveTrainCode.InvertMotorDirection(Motor.backLeft);
        driveTrainCode.InvertMotorDirection(Motor.frontLeft);
        imu.ResetAngle();
        navSystem = new NavigationManager(hardwareMap, imu, driveTrainCode);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    //endregion
    //region Drift Correction
    public float StrafeCorrection()
    {
        SoulsLikeTargetLock();
        float turnMod = (float) imu.AngleDeviation(targetLockHeading) / 20;
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

    //region Target Lock
    public void SoulsLikeTargetLock()
    {
        lastX = currentX;
        currentX = (navSystem.DeltaMovement().x);
        if(!gamepad1.right_stick_button)
        {
            hasLocked = false;
        }
        if(!hasLocked && gamepad1.right_stick_button) {
            targetLockHeading = imu.GetAngle();
            hasLocked = true;
        }
        if(hasLocked)
        {
            targetLockHeading = 0;
        }else
        {
            targetLockHeading = 0;
        }
    }
    //endregion
    //region Lift Recovery
    public void LiftRecovery()
    {
        telemetry.addData("Recover Lift: ","Hold back to unlock while holding back" +
                " press b to re-lock");
        liftLimits = !gamepad1.back;
        if(gamepad1.back && gamepad1.b)
        {
            telemetry.addData("Lift Recovery: ", "Re-locked");
            lift.Reset();
        }
    }
    //endregion

    private void SetCameraAngle(double x, double z){
        verticalR.setPosition(z);
        horizontalR.setPosition(x);
    }

}