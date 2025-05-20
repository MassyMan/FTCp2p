package org.firstinspires.ftc.teamcode.drivetrain;

import java.util.List;
import java.util.ArrayList;


// TODO: Make robot class and make everything in an FSM

public class ActionSequencer {
    private List<Action> actions;
    private int currentActionIndex;

    public ActionSequencer() {
        this.actions = new ArrayList<>();
        this.currentActionIndex = 0;
    }

    // Add an action to the sequence
    public void addAction(Action action) {
        actions.add(action);
    }

    // Run the actions in sequence and concurrently
    public void run() {
        for (Action action : actions) {
            action.run();  // Run all actions in parallel
        }

        // Run sequential actions and check if they are complete
        if (currentActionIndex < actions.size()) {
            Action currentAction = actions.get(currentActionIndex);
            if (currentAction.isComplete()) {
                currentActionIndex++;  // If the action is complete, move to the next one
            }
        }
    }

    // Reset the sequence to the first action
    public void reset() { currentActionIndex = 0;}
    public void skipAction(){ currentActionIndex ++;}

    // Check if all actions are completed
    public boolean isComplete() {
        return currentActionIndex >= actions.size();
    }
}
