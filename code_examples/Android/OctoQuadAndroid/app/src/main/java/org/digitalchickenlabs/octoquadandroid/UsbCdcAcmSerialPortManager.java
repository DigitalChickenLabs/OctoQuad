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

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Looper;
import android.util.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.concurrent.CopyOnWriteArrayList;

public class UsbCdcAcmSerialPortManager
{
    /**
     * Interface to receive events on
     */
    interface Callback
    {
        void onDeviceConnected(UsbCdcAcmSerialPort dev);
        void onDeviceDisconnected(UsbCdcAcmSerialPort dev);
        void onDeviceOpenFailed(UsbCdcAcmSerialPort dev, UsbCdcAcmSerialPort.OpenResultCode code);
    }

    // -----------------------------------------------------------------------------------------------------------------------------------
    //                      -------------- PUBLIC API ------------------
    // -----------------------------------------------------------------------------------------------------------------------------------

    /**
     * Get the singleton instance of UsbCdcAcmSerialPortManager
     * @param context an Android context
     * @return the singleton instance of UsbCdcAcmSerialPortManager
     */
    public static synchronized UsbCdcAcmSerialPortManager getInstance(Context context)
    {
        if(instance == null)
        {
            instance = new UsbCdcAcmSerialPortManager(context);
            instance.initialize();
        }

        return instance;
    }

    /**
     * Register a callback to get attach and detach events.
     * Make sure to call {@link #unregisterCallback(Callback)}
     * when you no longer wish to receive events.
     * @param callback the callback to register
     */
    public void registerCallback(Callback callback)
    {
        callbacks.add(callback);
    }

    /**
     * Unregister a callback previously registered with {@link #registerCallback(Callback)}
     * @param callback a callback previously registered with {@link #registerCallback(Callback)}
     */
    public void unregisterCallback(Callback callback)
    {
        callbacks.remove(callback);
    }

    /**
     * Call this to get a list of connected serial devices on startup, that you would not
     * otherwise know about, because you would not receive a connection callback for
     * them. This MUST be called from the UI thread.
     * @return the list of connected serial devices on startup
     */
    public ArrayList<UsbCdcAcmSerialPort> getCurrentlyKnownDevices()
    {
        if(Looper.getMainLooper().getThread() == Thread.currentThread())
        {
            // Current Thread is Main Thread.
            // Since the attach/deteach events come on the main thread it is
            // safe to return this to our caller, since any iteration they may
            // be doing on it will also happen on the main thread.
            return trackedSerialPorts;
        }

        throw new RuntimeException("getCurrentlyKnownDevices() MUST be called from the UI thread!");
    }

    /**
     * Call this to pass through an ACTION_USB_DEVICE_ATTACHED intent from your activity.
     * It is required to do this, because the intent is what signals that the device has
     * been attached *and* that permission has been granted, whereas the ACTION_USB_DEVICE_ATTACHED
     * broadcast (which is listened for internally here) signals only that the device has
     * been attached and makes no guarantees about permission.
     * @param intent the ACTION_USB_DEVICE_ATTACHED intent received in your activity
     */
    public void onNewIntent(Intent intent)
    {
        if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(intent.getAction()))
        {
            UsbDevice dev = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            Log.d(TAG, String.format("ACTION_USB_DEVICE_ATTACHED_INTENT device id=%d vid=0x%X pid=0x%X %s", dev.getDeviceId(), dev.getVendorId(), dev.getProductId(), dev.getDeviceName()));
            if(UsbCdcAcmSerialPort.isCompatibleDevice(dev))
            {
                boolean alreadyTracked = false;

                for(UsbCdcAcmSerialPort port : trackedSerialPorts)
                {
                    if(port.getId() == dev.getDeviceId())
                    {
                        alreadyTracked = true;
                        break;
                    }
                }

                if(!alreadyTracked)
                {
                    connectDevice(dev, true);
                }
                else
                {
                    Log.d(TAG, String.format("Received USB attachment notification for device id=%d, but we already have that device open! Taking no action.", dev.getDeviceId()));
                }
            }
            else
            {
                Log.d(TAG, String.format("USB device id=%d is NOT a supported device", dev.getDeviceId()));
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------------------------------
    //                      -------------- INTERNAL ------------------
    // -----------------------------------------------------------------------------------------------------------------------------------

    private UsbManager usbManager;
    private Context context;
    private BroadcastReceiver usbHotplugReceiver;
    private static final String TAG = "SerialPortManager";
    private ArrayList<UsbCdcAcmSerialPort> trackedSerialPorts = new ArrayList<>();
    private CopyOnWriteArrayList<Callback> callbacks = new CopyOnWriteArrayList<>();
    private static UsbCdcAcmSerialPortManager instance = null;

    private UsbCdcAcmSerialPortManager(Context context)
    {
        this.usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        this.context = context;
    }

    private void initialize()
    {
        for(UsbDevice device : enumerateUsbBus())
        {
            if(UsbCdcAcmSerialPort.isCompatibleDevice(device))
            {
                connectDevice(device, false);
            }
        }

        registerUsbHotplugReceiver();
    }

    private void connectDevice(UsbDevice device, boolean shouldFireCallback)
    {
        UsbCdcAcmSerialPort port = new UsbCdcAcmSerialPort(device, usbManager);

        UsbCdcAcmSerialPort.OpenResultCode resultCode = port.openAssumingPermission();

        if(resultCode == UsbCdcAcmSerialPort.OpenResultCode.SUCCESS)
        {
            Log.d(TAG, String.format("Successfully opened USB device id=%d", device.getDeviceId()));

            trackedSerialPorts.add(port);

            if(shouldFireCallback)
            {
                for(Callback callback : callbacks)
                {
                    if(callback != null)
                    {
                        callback.onDeviceConnected(port);
                    }
                }
            }
        }
        else
        {
            for(Callback callback : callbacks)
            {
                if(callback != null)
                {
                    callback.onDeviceOpenFailed(port, resultCode);
                }
            }
            Log.d(TAG, String.format("Failed to open USB device id=%d, resultCode=%s", device.getDeviceId(), resultCode.toString()));
        }
    }

    private Collection<UsbDevice> enumerateUsbBus()
    {
        HashMap<String, UsbDevice> map = usbManager.getDeviceList();
        return map.values();
    }

    private void registerUsbHotplugReceiver()
    {
        IntentFilter filter = new IntentFilter();
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);

        usbHotplugReceiver = new BroadcastReceiver()
        {
            @Override
            public void onReceive(Context context, Intent intent)
            {
                String action = intent.getAction();

                if(UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action))
                {
                    final UsbDevice dev = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    Log.d(TAG, String.format("ACTION_USB_DEVICE_ATTACHED device id=%d vid=0x%X pid=0x%X %s", dev.getDeviceId(), dev.getVendorId(), dev.getProductId(), dev.getDeviceName()));
                }
                else if(UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action))
                {
                    UsbDevice dev = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    Log.d(TAG, String.format("ACTION_USB_DEVICE_DETACHED device id=%d vid=0x%X pid=0x%X %s", dev.getDeviceId(), dev.getVendorId(), dev.getProductId(), dev.getDeviceName()));

                    UsbCdcAcmSerialPort portToRemove = null;

                    for(UsbCdcAcmSerialPort port : trackedSerialPorts)
                    {
                        if(port.getId() == dev.getDeviceId())
                        {
                            portToRemove = port;
                            break;
                        }
                    }

                    if(portToRemove != null)
                    {
                        Log.d(TAG, String.format("USB device id=%d was hooked by userspace driver; unhooking", dev.getDeviceId()));
                        portToRemove.close();
                        trackedSerialPorts.remove(portToRemove);

                        for(Callback callback : callbacks)
                        {
                            if(callback != null)
                            {
                                callback.onDeviceDisconnected(portToRemove);
                            }
                        }
                    }
                    else
                    {
                        Log.d(TAG, String.format("Removal of USB device id=%d does not require any action", dev.getDeviceId()));
                    }
                }
            }
        };

        context.getApplicationContext().registerReceiver(usbHotplugReceiver, filter);
    }
}
