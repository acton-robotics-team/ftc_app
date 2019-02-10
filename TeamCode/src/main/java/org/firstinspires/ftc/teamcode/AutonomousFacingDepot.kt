package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Facing depot")
class AutonomousFacingDepot : BaseAutonomous() {
    override val startLocation: AutonomousStartLocation = AutonomousStartLocation.FACING_DEPOT
}