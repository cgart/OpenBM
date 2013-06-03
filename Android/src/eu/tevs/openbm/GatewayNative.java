package eu.tevs.openbm;

import android.widget.Toast;
import android.content.Context;
import android.util.Log;

public class GatewayNative
{    
    private static GatewayNative INSTANCE = null;
    private GatewayNative()
    {
        System.loadLibrary("openbmgateway");
        //setOnReceiveCallback("eu/tevs/openbm/GatewayNative", "onIBusReceive");    
    }

    // -------------------- IBus related ------------------------- 
    private IBusReceiver ibusReceiver = null;
    private static void onIBusReceive(int src, int dst, byte[] data, byte[] raw)
    {
        if (getInstance().ibusReceiver != null)
            getInstance().ibusReceiver.onReceiveIBusMessage(src, dst, data, raw);
    }
    
    /**
    * Thread class where the IBus application is executed.
    **/
    private class ConnectionThread extends Thread
    {
        GatewayNative mGw;
        ConnectionThread(GatewayNative gw) { mGw = gw; }        
        public void run() { try { mGw.run(); } catch (Exception e) { Log.v("BMW-IBus: ", e.toString()); }; }        
    }    

    private ConnectionThread mThread;
    
    private native void run();
    private native void setOnReceiveCallback(String className, String funcName);        
    
    // -------------------- Public interface ------------------ 
 
    /**
    * Get Instance of the singleton connection
    **/   
    public static GatewayNative getInstance()
    {
        if(INSTANCE == null)
            INSTANCE = new GatewayNative();
        return INSTANCE;
    }

    /**
    * Set ibus receiver, to which we pass received ibus messages
    **/    
    public void setIBusReceiver(IBusReceiver r)
    {
        ibusReceiver = r;
    }
                    
    //! Connect to the given serial port
    public native int connect(String device, int ctstype);

    //! Disconnect ibus        
    public native void disconnect();
    
    //! Execute ibus. This will open a new thread and perform any ibus operation in that thread
    public void execute()
    {
        mThread = new ConnectionThread(this);
        mThread.start();
    }

    //! Send a message over ibus
    public native void send(int src, int dst, byte[] data);
};

