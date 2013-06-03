package eu.tevs.openbm;

public interface IBusReceiver
{
    public void onReceiveIBusMessage(int src, int dst, byte[] data, byte[] raw);
}

