package org.firstinspires.ftc.teamcode.drive;

public class ToggleButton {
    public Boolean toggled;
    Boolean pressed;
    Boolean prevState;

    public interface OnToggle {
        void onToggle();
    }
    public OnToggle onToggle;

    public ToggleButton(Boolean toggled, Boolean pressed) {
        this.toggled = toggled;
        this.pressed = pressed;
        this.prevState = !pressed;
    }

    public ToggleButton() {
        this.toggled = false;
        this.pressed = false;
        this.prevState = false;
    }

    public void updateState(Boolean pressed) {
        this.pressed = pressed;
        if (pressed && !prevState) {
            toggled = !toggled;
            onToggle.onToggle();
        }
        prevState = pressed;
    }
}
