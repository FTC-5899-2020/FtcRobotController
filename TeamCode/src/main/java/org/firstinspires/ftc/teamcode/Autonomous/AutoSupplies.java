package org.firstinspires.ftc.teamcode.Autonomous;

//imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
import java.util.Locale;

public abstract class AutoSupplies extends LinearOpMode{
    //  Establish hardware
    protected DcMotor  motorFwdLeft   = null;
    protected DcMotor  motorFwdRight  = null;
    protected DcMotor  motorBackLeft  = null;
    protected DcMotor  motorBackRight = null;
    protected BNO055IMU imu;
    public DcMotor slideMotor=null;
    public DcMotor motorIntakeLeft = null;
    public DcMotor motorIntakeRight = null;
    public Servo claw1=null;
    public Servo claw2=null;//this one is further from the slides
    public Servo grabClaw1=null;
    public Servo grabClaw2=null;

    //  Declare OpMode Members
    protected ElapsedTime runtime = new ElapsedTime();
    abstract public void runOpMode() throws InterruptedException;

    //  Protected variables
    protected double globalAngle;
    protected double globalPitch;
    protected Orientation lastAngles = new Orientation();
    protected Orientation lastPitches = new Orientation();

    private static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    private static final double     WHEEL_DIAMETER_INCHES   = 3.93701;
    public static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.14159265358979323846264338);

    //callable methods

    //move
    public void move(long millis, double x, double y)
    {
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower - strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower + strafePower;
        double rightBackPower = fwdBackPower + strafePower;
        double maxPower;
        double max = 1.0;

        maxPower = Math.abs(leftFrontPower);
        if (Math.abs(rightFrontPower) > maxPower) {
            maxPower = Math.abs(rightFrontPower);
        }
        if (Math.abs(leftBackPower) > maxPower) {
            maxPower = Math.abs(leftBackPower);
        }
        if (Math.abs(rightBackPower) > maxPower) {
            maxPower = Math.abs(rightBackPower);
        }
        if (maxPower > 1) {
            leftFrontPower = leftFrontPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightBackPower = rightBackPower / maxPower;

        }
        //sets the power of the motors
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis) {
            motorFwdLeft.setPower(leftFrontPower*max);
            motorFwdRight.setPower(rightFrontPower*max);
            motorBackLeft.setPower(leftBackPower*max);
            motorBackRight.setPower(rightBackPower*max);
        }
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    public void setPower(double x, double y)
    {
        double fwdBackPower = -y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower - strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower + strafePower;
        double rightBackPower = fwdBackPower + strafePower;

        motorFwdLeft.setPower(leftFrontPower);
        motorFwdRight.setPower(rightFrontPower);
        motorBackLeft.setPower(leftBackPower);
        motorBackRight.setPower(rightBackPower);
    }
    public void turn(int degrees, double power){
        int left = 1;
        int right = 1;
        resetAngle();
        telemetry.addData("Angle",getAngle());
        telemetry.update();
        if(degrees >= 0){
            right *= -1;
        }
        else if(degrees < 0){
            left *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(power * left);
        motorBackRight.setPower(power * right);
        motorFwdRight.setPower(power* right);

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {telemetry.addData("Angle2",getAngle());telemetry.update();}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {telemetry.addData("Angle2",getAngle());telemetry.update();}

        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
    }
    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnToS(int degrees, double power, int loopnum){
        int left = 1;
        int right = 1;
        double distance = getAngle() - degrees;
        double startAngle = getAngle();
        telemetry.addData("Angle3",getAngle());
        telemetry.update();
        if(getAngle() >= degrees){
            left *= -1;
        }
        else if(getAngle() < degrees){
            right *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(power * left);
        motorBackRight.setPower(power * right);
        motorFwdRight.setPower(power* right);

        if (getAngle() > degrees)
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                telemetry.addData("Angle4",getAngle());
                telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                telemetry.addData("Angle4", getAngle());
                telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
        if(--loopnum > 0){
            turnToS(degrees, power/2, loopnum);
        }
    }
    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnTo(int degrees, double power){
        int left = 1;
        int right = 1;
        telemetry.addData("Angle3",getAngle());
        telemetry.update();
        if(getAngle() >= degrees){
            left *= -1;
        }
        else if(getAngle() < degrees){
            right *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(power * left);
        motorBackRight.setPower(power * right);
        motorFwdRight.setPower(power* right);

        if (getAngle() > degrees)
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                telemetry.addData("Angle4",getAngle());
                telemetry.update();
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                telemetry.addData("Angle4", getAngle());
                telemetry.update();
            }
        }
        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
    }
    //  Pause for the specified amount of time (time: mili secs)
    public void pause(long millis){
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis){

        }
    }
    //Resets gyro sensor bearing value to 0
    //commonly used to calibrate before a match as well
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    //Resets gyro sensor pitch value to 0
    //commonly used to calibrate before a match as well
    public void resetPitch()
    {
        lastPitches = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalPitch = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */


    //uses the imu to find the current angle
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    //uses the imu to get the current pitch of the robot
    public double getPitch() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.secondAngle - lastPitches.secondAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalPitch += deltaAngle;

        lastPitches = angles;

        return globalPitch;
    }

    public void claw1Down(){
        claw1.setPosition(0.789);
    }
    public void claw2Down(){
        claw2.setPosition(0.289);
    }
    public void claw1Up(){
        claw1.setPosition(0.186);
    }
    public void claw2Up(){
        claw2.setPosition(0.940);
    }
    public void initForAutonomous()
    {
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;

        //initialize hardware
        //main motors
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        //motorFwdRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        //motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        //motorFwdLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        //motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorIntakeLeft = hardwareMap.get(DcMotor.class, "motorIntakeLeft");
        motorIntakeRight = hardwareMap.get(DcMotor.class, "motorIntakeRight");
        slideMotor = hardwareMap.get(DcMotor.class, "motorSlide");
        motorIntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw1 = hardwareMap.get(Servo.class,"claw1");
        claw2 = hardwareMap.get(Servo.class,"claw2");
        claw1.setPosition(.186); //these are the preset up positions for the claws
        claw2.setPosition(.940);
        grabClaw1 = hardwareMap.get(Servo.class,"grabClaw1");
        grabClaw2 = hardwareMap.get(Servo.class,"grabClaw2");

        //initializes imu and calibrates it. Prepares lift motor to land using the encoder
        // Lights turn green when it is calibrated
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    /*public void encoderDrive(double xSpeed, double ySpeed,
                               double Inches) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFwdRightTarget;
        int newFwdLeftTarget;
        int dirX;
        int dirY;
        if(xSpeed <0){
            dirX = -1;
            dirY = 1;
        }
        if(xSpeed >0){
            dirX = 1;
            dirY = -1;
        }
        if(ySpeed <0){
            dirX = -1;
            dirY = -1;
        }
        if(ySpeed > 0){
            dirX = 1;
            dirY = 1;
        }
        else{
            dirX = 0;
            dirY = 0;
        }
        // Determine new target position, and pass to motor controller
        newBackLeftTarget = motorBackLeft.getCurrentPosition() + (int) ((Inches * COUNTS_PER_INCH)*dirX);
        newBackRightTarget =motorBackRight.getCurrentPosition() + (int) ((Inches * COUNTS_PER_INCH)*dirY);
        newFwdLeftTarget = motorFwdLeft.getCurrentPosition() + (int) ((Inches * COUNTS_PER_INCH)*dirY);
        newFwdRightTarget =  motorFwdRight.getCurrentPosition() + (int) ((Inches * COUNTS_PER_INCH)*dirX);
        motorBackLeft.setTargetPosition(newBackLeftTarget);
        motorBackRight.setTargetPosition(newBackRightTarget);
        motorFwdLeft.setTargetPosition(newFwdLeftTarget);
        motorFwdRight.setTargetPosition(newFwdRightTarget);
        // Turn On RUN_TO_POSITION
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFwdRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFwdLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        setPower(xSpeed, ySpeed);
    }*/
}
