// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.firstinspires.ftc.teamcode.lib.mecanical_advantage;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Manages a number value published to the root table of NT. */
public class NetworkNumber extends SubsystemBase {
    private final String key;
    private final DoubleEntry entry;
    private double defaultValue = 0.0;
    private double value;

    /**
     * Creates a new LoggedNetworkNumber, for handling a number input sent via
     * NetworkTables.
     *
     * @param key The key for the number, published to the root table of NT or
     *            "/DashboardInputs/{key}" when logged.
     */
    public NetworkNumber(String key) {
        this.key = key;
        this.entry = NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(0.0);
        this.value = defaultValue;
    }

    /**
     * Creates a new LoggedNetworkNumber, for handling a number input sent via
     * NetworkTables.
     *
     * @param key          The key for the number, published to the root table of NT
     *                     or "/DashboardInputs/{key}" when logged.
     * @param defaultValue The default value if no value in NT is found.
     */
    public NetworkNumber(String key, double defaultValue) {
        this(key);
        setDefault(defaultValue);
        this.value = defaultValue;
    }

    /** Updates the default value, which is used if no value in NT is found. */
    public void setDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        entry.set(entry.get(defaultValue));
    }

    /**
     * Publishes a new value. Note that the value will not be returned by
     * {@link #get()} until the next cycle.
     */
    public void set(double value) {
        entry.set(value);
    }

    /** Returns the current value. */
    public double get() {
        return value;
    }

    @Override
    public void periodic() {
        value = entry.get(defaultValue);
    }
}
