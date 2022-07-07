/*
 * Copyright (c) 2022 DigitalChickenLabs
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

package org.digitalchickenlabs.octoquadandroid;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;

public class MainActivity extends Activity implements UsbCdcAcmSerialPortManager.Callback
{
    UsbCdcAcmSerialPortManager serialPortManager;

    TextView connectionStatus;
    LinearLayout layoutEncoders;
    TextView enc0, enc1, enc2, enc3, enc4, enc5, enc6, enc7;
    Button btnReset;

    UsbCdcAcmSerialPort octoquad;

    UiUpdaterThread uiUpdaterThread;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        connectionStatus = (TextView) findViewById(R.id.connectionStatus);
        layoutEncoders = (LinearLayout) findViewById(R.id.encoders);
        btnReset = (Button) findViewById(R.id.btnReset);
        enc0 = (TextView) findViewById(R.id.enc0);
        enc1 = (TextView) findViewById(R.id.enc1);
        enc2 = (TextView) findViewById(R.id.enc2);
        enc3 = (TextView) findViewById(R.id.enc3);
        enc4 = (TextView) findViewById(R.id.enc4);
        enc5 = (TextView) findViewById(R.id.enc5);
        enc6 = (TextView) findViewById(R.id.enc6);
        enc7 = (TextView) findViewById(R.id.enc7);

        layoutEncoders.setVisibility(View.INVISIBLE);

        serialPortManager = UsbCdcAcmSerialPortManager.getInstance(this);
        serialPortManager.registerCallback(this);

        ArrayList<UsbCdcAcmSerialPort> ports = serialPortManager.getCurrentlyKnownDevices();

        if(ports.size() > 0)
        {
            onDeviceConnected(ports.get(0));
        }

        btnReset.setOnClickListener(new View.OnClickListener()
        {
            @Override
            public void onClick(View v)
            {
                synchronized (MainActivity.this)
                {
                    if(uiUpdaterThread != null)
                    {
                        uiUpdaterThread.sendCommand('R');
                    }
                }
            }
        });
    }

    @Override
    protected void onNewIntent(Intent intent)
    {
        super.onNewIntent(intent);
        serialPortManager.onNewIntent(intent);
    }

    @Override
    protected void onDestroy()
    {
        super.onDestroy();
        serialPortManager.unregisterCallback(this);
    }

    @Override
    public synchronized void onDeviceConnected(UsbCdcAcmSerialPort dev)
    {
        if(octoquad == null)
        {
            octoquad = dev;
            uiUpdaterThread = new UiUpdaterThread(octoquad);
            uiUpdaterThread.start();

            layoutEncoders.setVisibility(View.VISIBLE);
            connectionStatus.setText("OctoQuad Connected");
            connectionStatus.setTextColor(getColor(R.color.colorConnected));
        }
    }

    @Override
    public synchronized void onDeviceDisconnected(UsbCdcAcmSerialPort dev)
    {
        if(dev == octoquad)
        {
            if(uiUpdaterThread != null)
            {
                uiUpdaterThread.shutdownAndAwaitTermination();
                uiUpdaterThread = null;
                octoquad = null;

                layoutEncoders.setVisibility(View.INVISIBLE);
                connectionStatus.setText("OctoQuad Not Connected");
                connectionStatus.setTextColor(getColor(R.color.colorNotConnected));
            }
        }
    }

    @Override
    public void onDeviceOpenFailed(UsbCdcAcmSerialPort dev, UsbCdcAcmSerialPort.OpenResultCode code)
    {

    }

    class UiUpdaterThread extends Thread
    {
        CountDownLatch latch = new CountDownLatch(1);
        byte[] readBuf = new byte[1];
        byte[] lineBuf = new byte[256];
        int lineBufIdx = 0;

        UsbCdcAcmSerialPort port;

        boolean uiUpdateRunning = false;

        public UiUpdaterThread(UsbCdcAcmSerialPort port)
        {
            this.port = port;
        }

        public void shutdownAndAwaitTermination()
        {
            UiUpdaterThread.this.interrupt();
            try {
                latch.await();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        void sendCommand(char cmd)
        {
            port.writeDisplacingIfNeeded(new byte[]{(byte)cmd}, 0, 1);
        }

        // Parse the CSV encoder string into an array
        public int[] parseEncoderString(String string)
        {
            String[] parts = string.split(",");
            int[] encoders = new int[8];

            for(int i = 0; i < 8; i++)
            {
                encoders[i] = Integer.parseInt(parts[i]);
            }

            return encoders;
        }

        @Override
        public void run()
        {
            try
            {
                // Synchronize to stream
                for(;;)
                {
                    port.read(readBuf, 0, 1);
                    if(readBuf[0] == '\n') break;
                }

                // Turn on fast mode
                sendCommand('F');

                while (!Thread.currentThread().isInterrupted())
                {
                    port.read(readBuf, 0, 1);
                    lineBuf[lineBufIdx] = readBuf[0];
                    lineBufIdx++;

                    // Did we hit the end of the line?
                    if(readBuf[0] == '\n')
                    {
                        String line = new String(lineBuf, 0,lineBufIdx-2, StandardCharsets.US_ASCII); // Trim off "\r\n"
                        lineBufIdx = 0;
                        final int[] encoders = parseEncoderString(line);

                        if(!uiUpdateRunning)
                        {
                            runOnUiThread(new Runnable()
                            {
                                @Override
                                public void run()
                                {
                                    try
                                    {
                                        enc0.setText("Encoder 0: " + encoders[0]);
                                        enc1.setText("Encoder 1: " + encoders[1]);
                                        enc2.setText("Encoder 2: " + encoders[2]);
                                        enc3.setText("Encoder 3: " + encoders[3]);
                                        enc4.setText("Encoder 4: " + encoders[4]);
                                        enc5.setText("Encoder 5: " + encoders[5]);
                                        enc6.setText("Encoder 6: " + encoders[6]);
                                        enc7.setText("Encoder 7: " + encoders[7]);
                                    }
                                    finally
                                    {
                                        uiUpdateRunning = false;
                                    }
                                }
                            });
                        }
                    }
                }
            }
            catch (ByteFIFO.ShutdownException | InterruptedException e)
            {
                e.printStackTrace();
            }
            finally
            {
                latch.countDown();
            }
        }
    }
}
