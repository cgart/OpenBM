package eu.tevs.openbm;

import java.util.ArrayList;

import android.app.Activity;
import android.app.ActivityManager;
import android.app.ActivityManager.RunningServiceInfo;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.view.View;
import android.os.Bundle;
import android.content.Intent;
import android.widget.EditText;
import android.widget.TextView;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.widget.Toast;
import android.widget.SimpleAdapter;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.view.ViewGroup;
import android.view.LayoutInflater;
import android.widget.ImageView;
import android.text.InputType;

/**
* Main activity representing a client app to the IBus server
**/
public class Main extends Activity
{
    private TextView mDevice;
    private int mFlowControl = 0;
    private int mMaxHistorySize = 15;
    private String mLastSendMessage = "";
        
    /**
    * Element stored in an array list to be shown in the list of received messages
    **/
    private class MessageElement
    {
        public boolean wasTransmitted = false;
        public String msg = "";
        public MessageElement(String m)
        {
            msg = m;
        }
        public MessageElement(String m, boolean trans)
        {
            msg = m;
            wasTransmitted = trans;
        }
    };        
    private class MessageAdapter extends ArrayAdapter<MessageElement>
    {
        public int maxSize = -1;
        
        private ArrayList<MessageElement> items;

        public MessageAdapter(Context context, int textViewResourceId, ArrayList<MessageElement> items)
        {
                super(context, textViewResourceId, items);
                this.items = items;
        }
        ArrayList<MessageElement> getList()
        {
            return items;
        }
        
        public void push_front(MessageElement elem)
        {
            // add new element
            insert(elem, 0);
            
            // check if number of elements exceeds some constant, then remove one in the beginning
            if (maxSize > 0 && getCount() > maxSize) 
                remove(getItem(getCount()-1));
                
            notifyDataSetChanged();        
        }
        
        @Override
        public View getView(int position, View convertView, ViewGroup parent)
        {
                View v = convertView;
                if (v == null)
                {
                    LayoutInflater vi = (LayoutInflater)getSystemService(Context.LAYOUT_INFLATER_SERVICE);
                    v = vi.inflate(R.layout.msgrow, null);
                }
                MessageElement o = items.get(position);
                if (o != null)
                {
                        TextView tt = (TextView) v.findViewById(R.id.raw);
                        ImageView iv = (ImageView) v.findViewById(R.id.icon);
                        if (tt != null)
                            tt.setText(o.msg);
                        if (iv != null)
                        {
                            if (o.wasTransmitted)
                                iv.setImageResource(R.drawable.arrowleft);
                            else
                                iv.setImageResource(R.drawable.arrowright);
                        }
                }
                return v;
        }
    }    
    
    private MessageAdapter mMsgAdapter;
    private ListView mMsgHistory;
    
    static final int DIALOG_SELECT_FLOW = 0;
    static final int DIALOG_SET_DEVICE = 1;
    static final int DIALOG_TYPE_MESSAGE = 2;
    
    static final String RECEIVE_INTENT = "ibus.bmw.RECEIVED";

    private boolean isServiceRunning()
    {
        ActivityManager manager = (ActivityManager) getSystemService(ACTIVITY_SERVICE);
        for (RunningServiceInfo service : manager.getRunningServices(Integer.MAX_VALUE)) {
            if (GatewayService.class.getName().equals(service.service.getClassName()))
                return true;
        }
        return false;
    }
    
    //! Connect to the ibus. Send intent to the service to connect
    private void connect(String dev)
    {
        Intent intent = new Intent(this, GatewayService.class);
        intent.putExtra("reconnect", 1);
        intent.putExtra("device", dev);
        intent.putExtra("flow", mFlowControl);
        startService(intent);    
    }
    
    //! Disconnect from the ibus. 
    private void disconnect()
    {
        Intent intent = new Intent(this, GatewayService.class);
        intent.putExtra("disconnect", 1);
        stopService(intent);
    }
    
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        mDevice = (TextView) findViewById(R.id.deviceText);
        mDevice.setText("/dev/ttyS2");
        
        ArrayList<MessageElement> list = new ArrayList<MessageElement>();
        list.add(new MessageElement("iBus Logger started"));
        mMsgHistory = (ListView) findViewById(R.id.msgHistory);
        mMsgAdapter = new MessageAdapter(this, R.id.msgHistory, list);
        mMsgHistory.setAdapter(mMsgAdapter);
        mMsgAdapter.maxSize = mMaxHistorySize;
    }
    
    //! Show dialog to choose flow control
    public void onFlowTextClick(View arg0)
    {
        showDialog(DIALOG_SELECT_FLOW);
    }
    
    //! Show dialog to choose the device
    public void onSelectDeviceClick(View arg0)
    {
        showDialog(DIALOG_SET_DEVICE);
    }

    public void onClickSend(View arg0)
    {
        if (isServiceRunning())
            showDialog(DIALOG_TYPE_MESSAGE);
        else
        {        
            Toast.makeText(this, "No Connection to iBus, hence cannot send", Toast.LENGTH_SHORT).show();            
        }
    }
    
    //! Connect 
    public void onClickConnect(View view)
    {
        connect(mDevice.getText().toString());
	}
	
    public void onClickDisconnect(View view)
    {
        disconnect();
	}
	
	
	protected Dialog onCreateDialog(int id)
	{
	    if (id == DIALOG_SELECT_FLOW)
	    {
            final CharSequence[] items = {"No Flow Control", "Hardware CTS/RTS", "GPIO Control"};

            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setTitle("Select flow control");
            builder.setItems(items, new DialogInterface.OnClickListener()
            {
                public void onClick(DialogInterface dialog, int item)
                {
                    if (item == 0) mFlowControl = -1;
                    if (item == 1) mFlowControl = 0;
                    if (item == 2) mFlowControl = 1;
                    
                    TextView flowText = (TextView) findViewById(R.id.flowText);
                    flowText.setText(items[item]);
                }
            });
            return builder.create();            
        }else if (id == DIALOG_SET_DEVICE)
        {
            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setTitle("Select RS232 device");
            final EditText et = new EditText(this);
            et.setText(mDevice.getText().toString());  
            builder.setView(et);
            builder.setPositiveButton("OK", new DialogInterface.OnClickListener()
            {
                public void onClick(DialogInterface dialog, int which)
                {
                    mDevice.setText(et.getText().toString());
                } 
            });
            
            return builder.create();

        }else if (id == DIALOG_TYPE_MESSAGE)
        {
            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setTitle("Type iBus Message (SDM-Format)");
            final EditText et = new EditText(this);
            et.setText(mLastSendMessage);  
            et.setInputType(InputType.TYPE_TEXT_FLAG_CAP_CHARACTERS | InputType.TYPE_TEXT_FLAG_NO_SUGGESTIONS);
            et.setMaxLines(1);
            builder.setView(et);
            builder.setPositiveButton("OK", new DialogInterface.OnClickListener()
            {
                public void onClick(DialogInterface dialog, int which)
                {
                    // separate on commas
                    String[] sdm = et.getText().toString().split( ",\\s*" );
                    
                    // prepare intent parsing the data
                    if (sdm.length >= 3)
                    {
                        try
                        {
                            int src = Integer.parseInt(sdm[0],16);
                            int dst = Integer.parseInt(sdm[1],16);
                            byte[] data = new byte[sdm.length-2];
                            for (int i=0; i < data.length; i++)
                                data[i] = (byte)Integer.parseInt(sdm[i+2], 16);
                                
                            Intent intent = new Intent(Main.this,GatewayService.class);
                            intent.putExtra("send", 1);
                            intent.putExtra("src", src);
                            intent.putExtra("dst", dst);
                            intent.putExtra("data", data);
                            startService(intent);       
                            
                        }catch(Exception e)
                        {
                            Toast.makeText(Main.this, "Cannot parse given message", Toast.LENGTH_SHORT).show();
                            return;
                        }       
                        
                        mLastSendMessage = sdm[0]+","+sdm[1];
                        for (int i=2; i < sdm.length; i++)
                            mLastSendMessage += "," + sdm[i];
                            
                        mMsgAdapter.push_front(new MessageElement(mLastSendMessage.toUpperCase(), true));
                    }
                } 
            });
            
            
            return builder.create();
        }
        return null;
	}
	
    	
	// -------------------- Receive IBus messages ------------------------
    private final BroadcastReceiver msgReceiver = new BroadcastReceiver()
    {
        @Override
        public void onReceive(Context context, Intent intent)
        {
            if (intent == null || !intent.hasExtra("src") || !intent.hasExtra("dst") || !intent.hasExtra("data")|| !intent.hasExtra("raw")) return;
            
            int src = intent.getIntExtra("src",0);
            int dst = intent.getIntExtra("dst",0);
            byte[] data = intent.getByteArrayExtra("data");
            byte[] raw = intent.getByteArrayExtra("raw");
            
            String ds = "";
            for (int i=0; i < data.length; i++)
                ds += " " + Integer.toHexString(data[i]);
            
            String sdm = (Integer.toHexString(src) + "," + Integer.toHexString(dst) + "," + ds).toUpperCase();

            String rds = "";
            for (int i=0; i < raw.length; i++)
                rds += Integer.toHexString(data[i]).toUpperCase() + " ";
            
            // add new element
            mMsgAdapter.push_front(new MessageElement(rds, false));
            
            //Toast.makeText(context, Integer.toHexString(src) + " " + Integer.toHexString(dst) + ds, Toast.LENGTH_SHORT).show();            
        }
    };	
    
    @Override
    protected void onResume()
    {
        super.onResume();
        registerReceiver(msgReceiver, new IntentFilter(RECEIVE_INTENT));
    }
 
    @Override
    protected void onPause()
    {
        super.onPause();
        unregisterReceiver(msgReceiver);
    }
    
}
