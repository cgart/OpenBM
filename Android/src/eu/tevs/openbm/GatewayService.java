package eu.tevs.openbm;

import android.content.Intent;
import android.app.Service;
import android.os.Bundle;
import android.util.Log;
import android.os.IBinder;
import android.widget.Toast;
import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;

import android.content.Context;

/**
* GatewayService - provides gateway support for the BMW's IBus
**/
public class GatewayService extends Service implements IBusReceiver
{    
    static final String RECEIVE_INTENT = "ibus.bmw.RECEIVED";
    
    GatewayNative mGateway;
        
    public enum RunningMode
    {
        NO, NO_CONNECTION, CONNECTED
    };
    String mDevice; // connected to this device    
    RunningMode mRunningMode = RunningMode.NO;

    IBinder mBinder;      // interface for clients that bind
    boolean mAllowRebind; // indicates whether onRebind should be used

    //! Inform user about start of the gateway
    private void addStartedNotification()
    {
        NotificationManager notificationManager = (NotificationManager)getSystemService(Context.NOTIFICATION_SERVICE);
        
        int icon = R.drawable.icon;
        CharSequence text = "BMW IBus Gateway connected";
        CharSequence contentTitle = "BMW IBus Gateway";
        CharSequence contentText = "Provides connection to the BMW's IBus.";
        long when = System.currentTimeMillis();
        
        Intent intent = new Intent(this, Main.class);
        PendingIntent contentIntent = PendingIntent.getActivity(this, 0, intent, 0);
        Notification notification = new Notification(icon,text,when);
                
        notification.setLatestEventInfo(this, contentTitle, contentText, contentIntent);
        notification.flags |= Notification.FLAG_NO_CLEAR;
        
        notificationManager.notify(1, notification);        
    }
    private void removeStartedNotification()
    {
        NotificationManager notificationManager = (NotificationManager)getSystemService(Context.NOTIFICATION_SERVICE);
        notificationManager.cancel(1);
    }    

    //! Callback by the native gateway on new message
    public void onReceiveIBusMessage(int src, int dst, byte[] data, byte[] raw)
    {
        Intent intent = new Intent(RECEIVE_INTENT);
		intent.putExtra("src", src);
		intent.putExtra("dst", dst);
		intent.putExtra("data", data);
		intent.putExtra("raw", raw);
		sendBroadcast(intent);
    }
    
    /**
    * Try to establish connection to the IBus through COM-Port. Notify on success.
    **/
    @Override
    public void onCreate()
    {
        mGateway = GatewayNative.getInstance();
        mGateway.setIBusReceiver(this);
    }
    
    @Override
    public int onStartCommand(Intent intent, int flags, int startId)
    {        
        Context context = getApplicationContext();

        if (intent == null)
        {
            Toast.makeText(context, "Invalid service execution", Toast.LENGTH_SHORT).show();        
            return START_STICKY;
        }
        
        // if we want to send a message
        if (intent.hasExtra("send"))
        {
            if (mRunningMode != RunningMode.CONNECTED)
            {
                Toast.makeText(context, "Connection hasn't been established yet", Toast.LENGTH_SHORT).show();        
                return START_STICKY;
            }            
            int src = intent.getIntExtra("src",0);
            int dst = intent.getIntExtra("dst",0);
            byte[] data = intent.getByteArrayExtra("data");
            mGateway.send(src, dst, data);
            
            return START_STICKY;
        }
        
        // if the intent asked for a reconnection, then do so
        if (intent.hasExtra("reconnect"))
        {
            if (mRunningMode == RunningMode.CONNECTED)
                mGateway.disconnect();
            mRunningMode = RunningMode.NO;
        }
        
        if (mRunningMode == RunningMode.NO_CONNECTION)
        {
            Toast.makeText(context, "Connection hasn't been established yet", Toast.LENGTH_SHORT).show();        
        }else if (mRunningMode == RunningMode.NO)
        {
            // need device
            if (!intent.hasExtra("device"))
            {
                Toast.makeText(context, "No valid device specified. Cannot connect!", Toast.LENGTH_SHORT).show();            
                mRunningMode = RunningMode.NO_CONNECTION;
                return START_STICKY;                
            }
            
            // try to establish connection
            if (mGateway.connect(intent.getStringExtra("device"), intent.getIntExtra("flow",0)) != 0)
            {
                Toast.makeText(context, "Cannot connect to " + intent.getStringExtra("device"), Toast.LENGTH_SHORT).show();            
                mRunningMode = RunningMode.NO_CONNECTION;
                return START_STICKY;                            
            }else
            Toast.makeText(context, "Connected with " + intent.getStringExtra("device"), Toast.LENGTH_SHORT).show(); 
            addStartedNotification();   
            mRunningMode = RunningMode.CONNECTED;
            mGateway.execute();
        }
        
        // The service is starting, due to a call to startService()
        return START_STICKY;
    }
    
    @Override
    public IBinder onBind(Intent intent) {
        // A client is binding to the service with bindService()
        return mBinder;
    }
    
    @Override
    public boolean onUnbind(Intent intent) {
        // All clients have unbound with unbindService()
        return mAllowRebind;
    }
    
    @Override
    public void onRebind(Intent intent) {
        // A client is binding to the service with bindService(),
        // after onUnbind() has already been called
    }
    
    @Override
    public void onDestroy()
    {
        // The service is no longer used and is being destroyed
        if (mRunningMode == RunningMode.CONNECTED)
        {
            mGateway.disconnect();
            Context context = getApplicationContext();
            Toast.makeText(context, "BMW's IBus disconnected", Toast.LENGTH_SHORT).show(); 
            removeStartedNotification();
        }
        mRunningMode = RunningMode.NO;
    }
    
}

