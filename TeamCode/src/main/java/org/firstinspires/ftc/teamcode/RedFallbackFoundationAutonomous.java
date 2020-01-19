package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red fallback (foundation) autonomous")
public class RedFallbackFoundationAutonomous extends BaseFallbackFoundationAutonomous {
    public RedFallbackFoundationAutonomous() {
        super(BaseAutonomous.Alliance.RED);
    }
}
