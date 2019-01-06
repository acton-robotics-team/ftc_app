package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous: facing depot (ONLY HITS GOLD MINERAL)")
class AutonomousFacingDepot : BaseAutonomous(){
    override val startLocation: AutonomousStartLocation = AutonomousStartLocation.FACING_DEPOT
}