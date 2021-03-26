package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "TeleOp")

public class ServoTest extends LinearOpMode {
    //All hardware
    protected DcMotor motorFwdLeft = null;
    protected DcMotor motorFwdRight = null;
    protected DcMotor motorBackLeft = null;
    protected DcMotor motorBackRight = null;
    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;
    public DcMotor intakeFwd = null;
    public DcMotor intakeBack = null;
    public Servo basketServo = null;
    public Servo unloadServo = null;
    public Servo wobbleGrabberServo = null;
    public Servo wobbleArmServo = null;


    double servo = 0.5;
@Override
    public void runOpMode() {
        //Prepares all the hardware
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        intakeFwd = hardwareMap.get(DcMotor.class, "intakeFwd");
        intakeBack = hardwareMap.get(DcMotor.class, "intakeBack");

        basketServo = hardwareMap.get(Servo.class,"basketServo");
        unloadServo = hardwareMap.get(Servo.class,"unloadServo");
        wobbleGrabberServo = hardwareMap.get(Servo.class, "wobbleGrabberServo");
        wobbleArmServo = hardwareMap.get(Servo.class, "wobbleArmServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

       // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                servo += 0.001;
            }
            else if(gamepad1.dpad_down){
                servo -= 0.001;
            }
            wobbleGrabberServo.setPosition(servo);

            telemetry.addData("unloadServo",basketServo.getPosition());
            telemetry.addData("servo", servo);
            telemetry.update(); 
        }
    }
}
