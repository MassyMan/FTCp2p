package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.teamcode.utils.EncoderHandler;

public class MoveSlidesPlaceholderAction implements Action {
    private boolean isComplete = false;
    private boolean triggerCondition;  // Condition to trigger the action
    private int targetPosition;        // Target position (e.g., number of ticks)
    private EncoderHandler encoderHandler;

    // Constructor accepts a condition to trigger the action and a target position
    public MoveSlidesPlaceholderAction(int targetPosition) {

        this.targetPosition = targetPosition;
        this.triggerCondition = false; // Initially not triggered
    }

    @Override
    public void run() {
        // Only run if the condition is met and the action hasn't been completed yet
        if (!isComplete && triggerCondition) {
            // TODO: SLIDE MOVEMENT
            // Insert the logic for moving the slides to the target position
            // Example: Move slides to the target position (this should be replaced with actual movement code)
       //     encoderHandler.setSlideTargetPosition(targetPosition);

            // Once the slide reaches the target position, mark the action as complete
            // TODO: if (slides are at the right spot) {
            // TODO: isComplete = true; }
            // TODO: maybe pass it off to EncoderHandler for slide encoders, TBD

        }
    }

    @Override
    public boolean isComplete() {
        return isComplete; // Return true once the action is complete
    }

    // Set the condition to trigger the action
    public void setTriggerCondition(boolean condition) {
        this.triggerCondition = condition;
    }

    // Optionally, if you want to update the target position during runtime
    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }
}
