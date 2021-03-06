/*
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 *
 */

@TeleOp(name="TeleOp: left beacon presser", group="Testing")
public class BeaconPresserOpMode extends VortexAutoOp {

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        super.start();
        beaconPresser.teamColor = 'r';
        fire2TurnDegree = 75;
        fire2WallDistance = 5420;
        wall2TurnDegree = -75;
        beacon2ParkTurnDegree = 45;
        VortexUtils.moveMotorByEncoder(robot.motorLeftArm, leftArmHomeParkingPosition, leftArmAutoMovePower);
    }

    @Override
    public void initBeaconPresser() {
        beaconPresser = new BeaconPresser(gyroTracker, leftBeaconArm);
        beaconPresser.setReporter(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.a) {
            beaconPresser.beaconArm.state = 0;
        }

        if (gamepad1.b) {
            beaconPresser.beaconArm.state = 1;
        }

        if (gamepad1.y) {
            beaconPresser.beaconArm.state = 2;
        }

        if (gamepad1.x) {
            beaconPresser.beaconArm.state = 3;
        }

        if (gamepad1.left_bumper) {
            beaconPresser.beaconArm.state = 4;
            beaconPresser.start(0);
        }

        float gain = 1.0f - gamepad1.left_trigger+0.01f;

        if (beaconPresser.beaconArm.state == 4) {
            beaconPresser.loop(0, 1);
        } else {
            beaconPresser.beaconArm.pressButton_loop(gain);
        }

        telemetry.addData("BeaconPresser State:", "%02d", beaconPresser.beaconArm.state);
        telemetry.addData("BeaconArm State:    ", beaconPresser.beaconArm.state);
        telemetry.addData("Speed gain:          ", gain);
        telemetry.addData("Upper Arm Pos:       ", "%f, (%f,%f)",
                beaconPresser.beaconArm.upperArm.getPosition(),
                beaconPresser.beaconArm.upperArmMin,
                beaconPresser.beaconArm.upperArmMax);
        telemetry.addData("Lower Arm Pos   ", "%f, (%f,%f)",
                beaconPresser.beaconArm.lowerArm.getPosition(),
                beaconPresser.beaconArm.lowerArmMin,
                beaconPresser.beaconArm.lowerArmMax);
        telemetry.addData("Upper Arm step size: ", beaconPresser.beaconArm.upperArmStepSize);
        telemetry.addData("Upper Arm step size: ", beaconPresser.beaconArm.lowerArmStepSize);
        telemetry.addData("Color sensor rgb     ", "%d,%d,%d",
                beaconPresser.beaconArm.colorSensor.red(),
                beaconPresser.beaconArm.colorSensor.green(),
                beaconPresser.beaconArm.colorSensor.blue());
        telemetry.addData("Color sensor ambient:", "%d,%d,%d",
                beaconPresser.beaconArm.ambientRGB.r,
                beaconPresser.beaconArm.ambientRGB.g,
                beaconPresser.beaconArm.ambientRGB.b);
        telemetry.addData("Color sensor intensity:", beaconPresser.beaconArm.getColorIntensity());
        telemetry.addData("Intensity threshold   :", beaconPresser.beaconArm.colorSensorForegroundThreshold);
        telemetry.addData("Color sensor color:   ", beaconPresser.beaconArm.getColorBlueOrRed());

        telemetry.addData("Near counts:        ", beaconPresser.beaconArm.nearCounts);
    }

}
