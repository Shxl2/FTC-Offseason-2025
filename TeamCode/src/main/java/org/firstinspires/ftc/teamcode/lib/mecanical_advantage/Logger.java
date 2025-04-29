package org.firstinspires.ftc.teamcode.lib.mecanical_advantage;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class Logger {
    public <T extends StructSerializable> void putData(String key, T data) throws NoSuchFieldException, IllegalAccessException {
        data.getClass().getField("struct").get(Struct.class);
    }
}
