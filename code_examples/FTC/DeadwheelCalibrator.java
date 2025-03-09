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

/*
 * This OpMode helps calibrate the ticks/mm and directions for your deadwheels
 */
@TeleOp
public class DeadwheelCalibrator extends LinearOpMode
{
    static final int FEET_TO_PUSH = 8;
    static final double FT_TO_MM = 12 * 25.4;

    @Override
    public void runOpMode()
    {
        OctoQuadFWv3 oq = hardwareMap.get(OctoQuadFWv3.class, "octoquad");

        oq.resetEverything();

        while (!isStopRequested() && !bumperPress())
        {
            telemetry.addLine(String.format("Relocate the robot such that it can be pushed %dFT in the +X direction (refer to the coordinate system in the quick start guide). Press the right bumper when you are ready to continue.", FEET_TO_PUSH));
            telemetry.update();
            sleep(20);
        }

        oq.resetAllPositions();
        while (!isStopRequested() && !bumperPress())
        {
            telemetry.addLine(String.format("Push the robot exactly %dFT in the +X direction. Press the right bumper when complete.", FEET_TO_PUSH));
            telemetry.update();
            sleep(20);
        }

        OctoQuadFWv3.EncoderDataBlock block = oq.readAllEncoderData();

        int x = findAbsMax(block);
        int xCount = block.positions[x];

        while (!isStopRequested() && !bumperPress())
        {
            telemetry.addLine(String.format("Relocate the robot such that it can be pushed %dFT in the +Y direction (refer to the coordinate system in the quick start guide). Press the right bumper when you are ready to continue.", FEET_TO_PUSH));
            telemetry.update();
            sleep(20);
        }

        oq.resetAllPositions();
        while (!isStopRequested() && !bumperPress())
        {
            telemetry.addLine(String.format("Push the robot exactly %dFT in the +Y direction. Press the right bumper when complete.", FEET_TO_PUSH));
            telemetry.update();
            sleep(20);
        }

        block = oq.readAllEncoderData();
        int y = findAbsMax(block);
        int yCount = block.positions[y];

        boolean xNeedsReversed = xCount < 0;
        boolean yNeedsReversed = yCount < 0;

        while (!isStopRequested())
        {
            telemetry.addLine(String.format("It looks like your X wheel is plugged into port %d and your Y wheel is plugged into port %d. Please verify this.", x, y));
            telemetry.addData("X wheel port should be reversed", xNeedsReversed);
            telemetry.addData("Y wheel port should be reversed", yNeedsReversed);
            telemetry.addData("X wheel ticks/mm", Math.abs(xCount / (FEET_TO_PUSH*FT_TO_MM)));
            telemetry.addData("Y wheel ticks/mm", Math.abs(yCount / (FEET_TO_PUSH*FT_TO_MM)));
            telemetry.update();
            sleep(100);
        }
    }

    int findAbsMax(OctoQuadFWv3.EncoderDataBlock blk)
    {
        int absMax = 0;
        int port = 0;

        for (int i = 0; i < blk.positions.length; i++)
        {
            if (Math.abs(blk.positions[i]) > absMax)
            {
                absMax = Math.abs(blk.positions[i]);
                port = i;
            }
        }

        return port;
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
