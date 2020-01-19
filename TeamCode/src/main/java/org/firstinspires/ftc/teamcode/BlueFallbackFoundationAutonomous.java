package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue fallback (foundation) autonomous")
public class BlueFallbackFoundationAutonomous extends BaseFallbackFoundationAutonomous {
    public BlueFallbackFoundationAutonomous() {
        super(BaseAutonomous.Alliance.BLUE);
    }
}
