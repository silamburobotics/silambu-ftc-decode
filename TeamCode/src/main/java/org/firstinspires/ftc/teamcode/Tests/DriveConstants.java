package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {
    public static double transPositionX =0.0,
                  transPositionY=0.0,
                    transHeading =0.0;
    public static void resetConst()
    {
        transPositionX =0.0;
        transPositionY=0.0;
        transHeading =0.0;
    }
}
