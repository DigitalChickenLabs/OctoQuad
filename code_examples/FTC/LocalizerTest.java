/*
 * Copyright (c) 2025 DigitalChickenLabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.OctoQuadFWv3;

/**
 * This OpMode demonstrates how to use the absolute localizer feature of the OctoQuad
 * FTC Edition MK2. It is STRONGLY recommended to read the Quick Start guide for the
 * localizer feature, located here:
 */
@TeleOp
public class LocalizerTest extends LinearOpMode
{
    // #####################################################################################
    // YOU MUST ADJUST THESE CONSTANTS FOR YOUR ROBOT! SEE THE QUICKSTART GUIDE.
    // #####################################################################################
    static final int DEADWHEEL_PORT_X = 1;
    static final int DEADWHEEL_PORT_Y = 2;
    static final OctoQuadFWv3.EncoderDirection DEADWHEEL_X_DIR = OctoQuadFWv3.EncoderDirection.FORWARD;
    static final OctoQuadFWv3.EncoderDirection DEADWHEEL_Y_DIR = OctoQuadFWv3.EncoderDirection.REVERSE;
    static final float X_TICKS_PER_MM = 12.66f;
    static final float Y_TICKS_PER_MM = 12.66f;
    static final float TCP_OFFSET_X_MM = -97.05f;
    static final float TCP_OFFSET_Y_MM = -156.70f;
    static final float IMU_SCALAR = 1.0323f;

    // Conversion factor for radians --> degrees
    static final double RAD2DEG = 180/Math.PI;

    // For tracking the number of CRC mismatches
    int badPacketCount = 0;
    int goodPacketCount = 0;

    // For tracking loop speed
    long prevTime = 0;
    MovingStatistics statistics = new MovingStatistics(200);

    // Data structure which will store the localizer data read from the OctoQuad
    OctoQuadFWv3.LocalizerDataBlock localizer = new OctoQuadFWv3.LocalizerDataBlock();

    /*
     * Main OpMode function
     */
    public void runOpMode()
    {
        // Begin by retrieving a handle to the device from the hardware map. Take care
        // to specify 'OctoQuadFWv3' as specifying 'OctoQuad' will load the FTC SDK's
        // built-in driver which only supports firmware v2.
        OctoQuadFWv3 oq = hardwareMap.get(OctoQuadFWv3.class, "octoquad");

        // Crank up the telemetry update frequency to make the data display a bit less laggy
        telemetry.setMsTransmissionInterval(50);

        // Configure a whole bunch of parameters for the absolute localizer
        // --> Read the quick start guide for an explanation of these!!
        // IMPORTANT: these parameter changes will not take effect until
        // the localizer is reset!
        oq.setSingleEncoderDirection(DEADWHEEL_PORT_X, DEADWHEEL_X_DIR);
        oq.setSingleEncoderDirection(DEADWHEEL_PORT_Y, DEADWHEEL_Y_DIR);
        oq.setLocalizerPortX(DEADWHEEL_PORT_X);
        oq.setLocalizerPortY(DEADWHEEL_PORT_Y);
        oq.setLocalizerCountsPerMM_X(X_TICKS_PER_MM);
        oq.setLocalizerCountsPerMM_Y(Y_TICKS_PER_MM);
        oq.setLocalizerTcpOffsetMM_X(TCP_OFFSET_X_MM);
        oq.setLocalizerTcpOffsetMM_Y(TCP_OFFSET_Y_MM);
        oq.setLocalizerImuHeadingScalar(IMU_SCALAR);
        oq.setLocalizerVelocityIntervalMS(25);
        oq.setI2cRecoveryMode(OctoQuadFWv3.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

        // Resetting the localizer will apply the parameters configured above.
        // This function will NOT block until calibration of the IMU is complete -
        // for that you need to look at the status returned by getLocalizerStatus()
        oq.resetLocalizerAndCalibrateIMU();

        /*
         * INIT LOOP
         */
        while (opModeInInit())
        {
            telemetry.addData("Firmware Version", oq.getFirmwareVersionString());
            telemetry.addData("Localizer Status", oq.getLocalizerStatus());
            telemetry.addData("Heading Axis Detection", oq.getLocalizerHeadingAxisChoice());
            telemetry.update();
            sleep(100);
        }

        /*
         * After the user presses play, the IMU might not have finished calibrating yet,
         */
        while (!isStopRequested())
        {
            OctoQuadFWv3.LocalizerStatus status = oq.getLocalizerStatus();
            if(status != OctoQuadFWv3.LocalizerStatus.RUNNING)
            {
                telemetry.addData("Status", status);
                telemetry.addLine("Waiting for localizer to report RUNNING status");
                telemetry.update();
                sleep(100);
            }
            else
            {
                // If we do have RUNNING status, we're good to go!
                break;
            }
        }

        /*
         * MAIN LOOP
         */
        while (opModeIsActive())
        {
            // Read updated data from the OctoQuad into the 'localizer' data structure
            oq.readLocalizerData(localizer);

            // #################################################################################
            // IMPORTANT! Check whether the received CRC for the data is correct. An incorrect
            //            CRC indicates that the data was corrupted in transit and cannot be
            //            trusted. This can be caused by ESD, but it has also been observed that
            //            there appears to be a hardware bug in the Control Hub / Expansion Hub
            //            where motor switching noise can confuse the I2C controller and result
            //            in malformed clock pulses which results in corrupted data transfer.
            //
            //            If the CRC is not reported as OK, you should throw away the data and
            //            try to read again.
            // #################################################################################
            if (localizer.crcOk)
            {
                long curTime = System.currentTimeMillis();
                if (prevTime != 0)
                {
                    long deltaTime = curTime - prevTime;
                    statistics.add(deltaTime);
                    telemetry.addData("Loop Hz", 1e3/statistics.getMean());
                }
                prevTime = curTime;

                telemetry.addData("Localizer Status", localizer.localizerStatus);
                telemetry.addData("Heading deg", localizer.heading_rad * RAD2DEG);
                telemetry.addData("Heading dps", localizer.velHeading_radS * RAD2DEG);
                telemetry.addData("X mm", localizer.posX_mm);
                telemetry.addData("Y mm", localizer.posY_mm);
                telemetry.addData("VX mm/s", localizer.velX_mmS);
                telemetry.addData("VY mm/s", localizer.velY_mmS);
                goodPacketCount++;
            }
            else
            {
                telemetry.addLine("Data CRC not valid");
                badPacketCount++;
            }

            // Print some statistics about CRC validation
            telemetry.addLine(String.format("CRC Mismatch Count: %d/%d (%.3f%%)", badPacketCount, goodPacketCount, ((float)badPacketCount/(float)goodPacketCount)*100.0f));

            // Demonstrate the ability to "teleport" the localizer position on demand.
            if (gamepad1.left_bumper)
            {
                // You can teleport position AND heading
                oq.setLocalizerPose(500, 750, (float)(Math.PI/4.0f));
            }
            else if (gamepad1.right_bumper)
            {
                // Or you can teleport just heading
                oq.setLocalizerHeading((float) Math.PI);
            }

            // Send updated telemetry to the Driver Station
            telemetry.update();
        }
    }
}
