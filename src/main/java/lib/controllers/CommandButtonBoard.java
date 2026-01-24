package lib.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Wrapper over two generic HID boards for the button panel layout. */
public final class CommandButtonBoard {
    private final CommandGenericHID leftButtonBoard;
    private final CommandGenericHID rightButtonBoard;

    public CommandButtonBoard(int leftBoardPort, int rightBoardPort) {
        this.leftButtonBoard = new CommandGenericHID(leftBoardPort);
        this.rightButtonBoard = new CommandGenericHID(rightBoardPort);
    }

    /** Returns a trigger for a button on the left board. */
    public Trigger leftButton(int button) {
        return leftButtonBoard.button(button);
    }

    /** Returns a trigger for a button on the right board. */
    public Trigger rightButton(int button) {
        return rightButtonBoard.button(button);
    }

    /** Returns the action button (right board button 1). */
    public Trigger getActionButton() {
        return rightButtonBoard.button(1);
    }

    /** Returns the stow button (right board button 6). */
    public Trigger getStowButton() {
        return rightButtonBoard.button(6);
    }

    /** Returns the L1 button (right board button 5). */
    public Trigger getL1Button() {
        return rightButtonBoard.button(5);
    }

    /** Returns the L2 button (right board button 4). */
    public Trigger getL2Button() {
        return rightButtonBoard.button(4);
    }

    /** Returns the L3 button (right board button 3). */
    public Trigger getL3Button() {
        return rightButtonBoard.button(3);
    }

    /** Returns the L4 button (right board button 2). */
    public Trigger getL4Button() {
        return rightButtonBoard.button(2);
    }
}
