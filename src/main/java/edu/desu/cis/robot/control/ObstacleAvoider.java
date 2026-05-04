package edu.desu.cis.robot.control;

public class ObstacleAvoider extends RobotController {

    public ObstacleAvoider(String robotName) {

        super(robotName);
    }

    @Override
    public void run() {
        mbot.avoidCrashing(15.0);
        // Start obstacle avoidance in the background.
        mbot.forward(20);


        while (true) {
            double distance = mbot.readUltrasonic();
            if (distance > 0 && distance <= 15.0) {
                mbot.stopAllBehaviors();
                mbot.moveAndTurnLeft(30,2,15);
                mbot.moveAndTurnRight(30,4,15);
                break;
            }
            // Flash the LEDs red every time we issue a "heartbeat" check.
        }
        mbot.stopAllBehaviors();
        mbot.stop();

    }

    public static void main(String[] args) {
        try (ObstacleAvoider robot = new ObstacleAvoider("Vulcans")) {
            robot.run();
        }
    }
}
