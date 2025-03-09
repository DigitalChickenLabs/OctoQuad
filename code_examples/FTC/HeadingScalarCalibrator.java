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

import org.firstinspires.ftc.teamcode.OctoQuadFWv3;

/**
 * This OpMode helps calibrate the heading scalar for the IMU on the
 * OctoQuad FTC Edition MK2
 */
@TeleOp
public class HeadingScalarCalibrator extends LinearOpMode
{
    OctoQuadFWv3 oq;

    int wraps = 0;
    double integratedHeading;
    float lastNormalizedHeading = 0;

    static final int SETTLE_MS = 10000;
    static final int NUM_ROTATIONS = 5;

    @Override
    public void runOpMode()
    {
        telemetry.setMsTransmissionInterval(50);

        oq = hardwareMap.get(OctoQuadFWv3.class, "octoquad");

        telemetry.addLine("Place the robot on a level surface, against a solid edge. The baseboard of a wall usually works well for this. Press the right bumper to continue.");
        telemetry.update();

        while (!isStopRequested() && !bumperPress())
        {
            sleep(20);
        }

        telemetry.addLine("We now are going to perform the initial calibration on the IMU. DO NOT move the robot during this process!");
        telemetry.update();
        oq.setLocalizerImuHeadingScalar(1.0f);
        oq.resetLocalizerAndCalibrateIMU();

        while (!isStopRequested())
        {
            sleep(20);
            if (oq.getLocalizerStatus() == OctoQuadFWv3.LocalizerStatus.RUNNING)
            {
                break;
            }
        }

        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < SETTLE_MS && !isStopRequested())
        {
            int secRemaining = (int) Math.round((SETTLE_MS - (System.currentTimeMillis() - startTime)) / (1000.0));
            telemetry.addLine(String.format("Initial calibration complete. Waiting for the continuous calibration algorithm to settle... DO NOT MOVE THE ROBOT... (%d)", secRemaining));
            telemetry.update();
            sleep(20);
        }

        telemetry.addLine("We are now ready to measure the scale factor. Press the right bumper to continue.");
        telemetry.update();

        oq.setLocalizerHeading(0.0f);

        while (!isStopRequested() && !bumperPress())
        {
            OctoQuadFWv3.LocalizerDataBlock data = oq.readLocalizerData();

            if (data.crcOk)
            {
                if (data.heading_rad - lastNormalizedHeading > Math.PI / 2)
                {
                    wraps--;
                }
                else if (data.heading_rad - lastNormalizedHeading < -Math.PI / 2)
                {
                    wraps++;
                }

                integratedHeading = wraps*(2*Math.PI) + data.heading_rad;
                lastNormalizedHeading = data.heading_rad;

                telemetry.addLine(String.format("Gently rotate the robot %d turns COUNTER-CLOCKWISE, and return it to the same solid reference. DO NOT PITCH OR ROLL THE ROBOT WHILE ROTATING! Press the right bumper when this is complete.\n", NUM_ROTATIONS));
                telemetry.addData("Current uncalibrated heading (deg)", integratedHeading*180/Math.PI);
                telemetry.update();
            }
        }

        double expectedRad = Math.PI*2*NUM_ROTATIONS;
        double actualRad = integratedHeading;

        double scaleFactor = expectedRad / actualRad;

        if (scaleFactor < 0.9 || scaleFactor > 1.1)
        {
            while (!isStopRequested())
            {
                telemetry.addData("Your determined heading scalar is", scaleFactor);
                telemetry.addLine("This scale factor is outside the expected range. You may have performed the calibration incorrectly, or there may be a hardware problem.");
                telemetry.update();
                sleep(20);
            }
        }
        else
        {
            while (!isStopRequested())
            {
                telemetry.addData("Your determined heading scalar is", scaleFactor);
                telemetry.addLine("If desired, press the right bumper to save this to the OctoQuad's internal flash memory. Otherwise, take note of this scalar so you can load it from your OpMode.");
                telemetry.update();

                sleep(20);

                if (bumperPress())
                {
                    telemetry.addLine("Saving scalar to flash");
                    telemetry.update();

                    oq.setLocalizerImuHeadingScalar((float) scaleFactor);
                    oq.saveParametersToFlash();

                    while (!isStopRequested())
                    {
                        telemetry.addLine("Saved!");
                        telemetry.update();
                        sleep(20);
                    }

                    return;
                }
            }
        }
    }

    boolean lastBumper;

    boolean bumperPress()
    {
        boolean bmp = gamepad1.right_bumper;
        boolean trig = bmp && !lastBumper;

        lastBumper = bmp;
        return trig;
    }
}
