package org.firstinspires.ftc.teamcode.Cheese;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Cheesy Justin", group="Cheese")
//@Disabled
public class CheeseRichard extends AutoSuppliesCheese {
    @Override
    public void runOpMode() {

        //  Establish all hardware
        initForAutonomous();

        //  Wait until start
        waitForStart();

        pause( 3000 );
        move(3000,.5,0);
        pause(1000);
        move(3000, 0, .5);
        pause(1000);
        move(1000,.5, .5);
        pause(1000);
        turnToS(90,.5,2);
        pause(500);
        turnToS(-90,.8,2);
        pause(100);

        //  Turn all motors off and sleep
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(1000);
    }
}
