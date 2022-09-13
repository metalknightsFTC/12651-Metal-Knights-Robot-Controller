package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enums.SelectedDrive;

@TeleOp
public class TeleopNeedle extends LinearOpMode {

    private String intake = "intake";
    private String duck = "duck";
    private String lift = "lifter";
    private String grabber = "grabber";
    private  SubSystemControl subSystemControl;
    private boolean automatics = false;
    private int targetRotations = 0;
    private double duckPower = .47;
    double totalLiftPercentage;

    @Override
    public void runOpMode(){
        /*intake = hardwareMap.get(DcMotor.class, "intake");
        duck = hardwareMap.get(DcMotor.class, "duck");
        lift = hardwareMap.get(DcMotor.class, "lifter");
        grabber = hardwareMap.get(Servo.class, "grabber");*/
        String[] motors = new String[3];
        String[] servos= new String[1];

        motors[0] = intake;
        motors[1] = duck;
        motors[2] = lift;
        servos[0] = grabber;

        DriveTrainCode driveTrainCode = new DriveTrainCode(gamepad1,hardwareMap);
        subSystemControl = new SubSystemControl(hardwareMap,motors, servos);

        //driveTrainCode.InvertMotorDirection(Motor.frontRight);
        //driveTrainCode.InvertMotorDirection(Motor.frontLeft);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            driveTrainCode.UpdateDriveTrain(SelectedDrive.mecanum);

            if(gamepad1.back&&automatics == false){
                automatics = true;
            }else if(gamepad1.back && automatics){
                automatics = false;
                targetRotations = 0;
            }
            if(gamepad1.left_bumper){
                subSystemControl.ManipulateServo(grabber).setPosition(1);
            }else{
                subSystemControl.ManipulateServo(grabber).setPosition(0);
            }

            if(gamepad1.a){
                targetRotations = 58;
            }
            if(gamepad1.x){
                targetRotations = 144;
            }
            if(gamepad1.y){
                targetRotations = 288;
            }
            if(gamepad1.b){
                targetRotations = 0;
            }

            telemetry.addData("AP = ", automatics);

            if(automatics){
                double liftDelta = (targetRotations-subSystemControl.ManipulateMotor(lift).getCurrentPosition())*.2;
                subSystemControl.ManipulateMotor(lift).setPower(liftDelta);

            }else{
                if(gamepad1.dpad_up)
                    subSystemControl.ManipulateMotor(lift).setPower(0.5);
                else if(gamepad1.dpad_down)
                    subSystemControl.ManipulateMotor(lift).setPower(-0.5);
                else
                    subSystemControl.ManipulateMotor(lift).setPower(0);
            }

            subSystemControl.ManipulateMotor(intake).setPower(gamepad1.right_trigger);

            if(gamepad1.dpad_left){
                duckPower = .47 + (gamepad1.left_trigger*2);
            }else if(gamepad1.dpad_right){
                duckPower = -.47 - (gamepad1.left_trigger*2);
            }else{
                duckPower = 0;
            }
            subSystemControl.ManipulateMotor(duck).setPower(duckPower);

            telemetry.addData("Lift Target: ", (double)((double)targetRotations));
            telemetry.addData("Current Lift Position: ", subSystemControl.ManipulateMotor(lift).getCurrentPosition());
            telemetry.update();

        }

    }
}
