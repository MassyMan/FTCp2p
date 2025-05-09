package org.firstinspires.ftc.teamcode.drivetrain;

public class GotoPointAction implements Action {
    private DrivetrainActions dtAction;
    private double targetX, targetY, targetT, moveSpeed, distThreshold;
    private boolean conditionMet;

    public GotoPointAction(DrivetrainActions dtAction, double targetX, double targetY, double targetT, double moveSpeed, double distThreshold, boolean conditionMet) {
        this.dtAction = dtAction;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetT = targetT;
        this.moveSpeed = moveSpeed;
        this.distThreshold = distThreshold;
        this.conditionMet = conditionMet;
    }

    @Override
    public void run() {
        // This method runs the action's logic if the condition is met
        if (conditionMet) {
            dtAction.gotoPoint(targetX, targetY, targetT, moveSpeed, distThreshold, conditionMet);
        }
    }

    @Override
    public boolean isComplete() {
        // This method checks if the action is complete
        return dtAction.moveOn;  // moveOn is set to true when the robot has reached the target and the action is complete
    }
}
