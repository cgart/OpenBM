#include "jni_native.h"
#include "Application.h"
#include "Log.h"
#include "GatewayProtocol.h"

#include <android/log.h>
#include <stdio.h>
#include <iostream>

static JavaVM* jvm = NULL;
//static std::string jReceiveCallbackClass = "";
//static std::string jReceiveCallbackFunc = "";

static jobject jReceiveCallbackObject;
const char *jReceiveCallbackClass = "eu/tevs/openbm/GatewayNative";

void initClassHelper(JNIEnv *env, const char *path, jobject *objptr)
{
    jclass cls = env->FindClass(path);
    if(!cls)
    {
        __android_log_print(ANDROID_LOG_ERROR, "BMW-IBus", "initClassHelper: failed to get %s class reference", path);
        return;
    }
    
    jmethodID constr = env->GetMethodID(cls, "<init>", "()V");
    if(!constr)
    {
        __android_log_print(ANDROID_LOG_ERROR, "BMW-IBus", "initClassHelper: failed to get %s constructor", path);
        return;
    }
    
    jobject obj = env->NewObject(cls, constr);
    if(!obj)
    {
        __android_log_print(ANDROID_LOG_ERROR, "BMW-IBus", "initClassHelper: failed to create a %s object", path);
        return;
    }
    
    (*objptr) = env->NewGlobalRef(obj);
}


JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void * reserved)
{
    jvm = vm;
    JNIEnv *env;
    if (vm->GetEnv((void**) &env, JNI_VERSION_1_6) != JNI_OK)
    {
        __android_log_print(ANDROID_LOG_ERROR, "BMW-IBus", "Failed to get environment");        
        return -1;   
    }
    
    initClassHelper(env, jReceiveCallbackClass, &jReceiveCallbackObject);

    return JNI_VERSION_1_6;
}
void JNI_OnUnload(JavaVM *vm, void *reserved)
{
    jvm = NULL;
}

void onStopIBusThread()
{
    if (jvm) jvm->DetachCurrentThread();
}

void onReceiveIBusMessage(const IBus::Message& msg)
{
    if (!jvm) return;// || !jReceiveCallbackClass.length() || !jReceiveCallbackFunc.length()) return;
 
    JNIEnv *env;
    jint attached = jvm->AttachCurrentThread(&env, NULL);
    //jclass jc = env->FindClass(jReceiveCallbackClass.c_str());
    jclass jc = env->GetObjectClass(jReceiveCallbackObject);
    if (env->ExceptionOccurred())
    {
        env->ExceptionClear();
        __android_log_print(ANDROID_LOG_ERROR, "BMW-IBus", "Cannot callback on receive");
        return;
    }    
    
    jmethodID mid = env->GetStaticMethodID(jc, "onIBusReceive","(II[B[B)V");
    if (env->ExceptionOccurred())
    {
        env->ExceptionClear();
        __android_log_print(ANDROID_LOG_ERROR, "BMW-IBus", "Cannot callback on receive, because function name is wrong");
        return;
    }
        
    jbyteArray jb = env->NewByteArray(msg.getData().size());
    env->SetByteArrayRegion(jb, 0, msg.getData().size(), (jbyte*)&(msg.getData()[0]));

    jbyteArray rb = env->NewByteArray(msg.getRaw().size());
    env->SetByteArrayRegion(rb, 0, msg.getRaw().size(), (jbyte*)&(msg.getRaw()[0]));

	jvalue jargs[4];
	memset(&jargs[0], 0, sizeof(jvalue) * 4);
	jargs[0].b = msg.getSource();
	jargs[1].b = msg.getDestination();
	jargs[2].l = jb;
	jargs[3].l = rb;
	env->CallStaticVoidMethodA(jc, mid, jargs);
	
	env->DeleteLocalRef(jb);
	env->DeleteLocalRef(rb);
   
    if (attached == JNI_EDETACHED)
        jvm->DetachCurrentThread();
}

void logCallback(const std::string& str)
{
    __android_log_print(ANDROID_LOG_DEBUG, "BMW-IBus", str.c_str());    
    
    /*
    if (str.length() <= 38)
    {
        IBus::Message msg(0xF0, 0x68, 0);
        std::vector<unsigned char> d(str.length());
        for (int i=0; i < str.length(); i++)
            d[i] = str[i];
        msg.setData(d);
        onReceiveIBusMessage(msg);
    }*/
}

JNIEXPORT void JNICALL Java_eu_tevs_openbm_GatewayNative_send(JNIEnv* env, jobject obj, jint src, jint dst, jbyteArray data)
{
    if (!Application::g_app) return;

    assert(sizeof(jbyte) == sizeof(unsigned char));

    jint len = env->GetArrayLength(data);
    jbyte* _d = env->GetByteArrayElements(data, 0);    
    
    std::vector<unsigned char> d(len,0);    
    memcpy(&d[0], _d, len);

    env->ReleaseByteArrayElements(data, _d, 0);
        
    IBus::Message* msg = new IBus::Message(src, dst, 0);
    msg->setData(d);
        
    Application::g_app->sendIBusMessage(msg, false);
}

JNIEXPORT void JNICALL Java_eu_tevs_openbm_GatewayNative_disconnect(JNIEnv* env, jobject obj)
{
    if (Application::g_app)
    {
        Application::g_app->stop();
        Application::g_app->disconnectIBus();
        delete Application::g_app;
        Application::g_app = NULL;
    }
}

JNIEXPORT void JNICALL Java_eu_tevs_openbm_GatewayNative_run(JNIEnv* env, jobject obj)
{
    if (Application::g_app)
    {
        try{
            if(env->GetJavaVM(&jvm) != 0) jvm = NULL;
            
            Application::g_app->setOnReceiveCallback(onReceiveIBusMessage);
            Application::g_app->setOnStopIBusThreadCallback(onStopIBusThread);
            Application::g_app->execute();                
        }catch(...)
        {
            LOG_ERROR("UNKNOWN ERROR CATCHED");
        }
    }
}

/*
JNIEXPORT void JNICALL Java_eu_tevs_openbm_GatewayNative_setOnReceiveCallback (JNIEnv* env, jobject obj, jstring className, jstring func)
{
    jReceiveCallbackClass = env->GetStringUTFChars(className, 0);
    jReceiveCallbackFunc = env->GetStringUTFChars(func, 0);
}
*/

/**
* Connect to the given device. Return error code if fails
**/
JNIEXPORT jint JNICALL Java_eu_tevs_openbm_GatewayNative_connect(JNIEnv* env, jobject obj, jstring _device, jint _cts)
{
    int loglevel = 0;
 
    // set log level and file
    if (loglevel < 0) loglevel = 0;
    if (loglevel > 4) loglevel = 4;
    Logger::instance()->setLevel((Logger::Level)loglevel);
    Logger::instance()->setCallback(logCallback);
    
    LOG_DEBUG("Connect to IBus...");
    
    // get arguments
    std::string device = env->GetStringUTFChars(_device, 0);
    int cts = static_cast<int>(_cts);
 
 #if 1 
     int fd, serial;
    fd = open("/dev/ttyS2", O_RDWR | O_NONBLOCK | O_NOCTTY);
  termios ios;
  errno = 0;
  tcgetattr(fd, &ios);
    ios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK
        | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    ios.c_oflag &= ~OPOST;
    ios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    ios.c_cflag &= ~(CSIZE | PARENB);
    ios.c_cflag |= CS8;
    ios.c_iflag |= IGNPAR;
    ios.c_cflag |= CREAD | CLOCAL;
    errno = 0;
    tcsetattr(fd, TCSANOW, &ios);
    
    ioctl(fd, TIOCMGET, &serial);
    if (serial & TIOCM_CTS)
        logCallback("TIOCM_CTS is not set");
    else
        logCallback("TIOCM_CTS is set");
    close(fd);      
#endif
    
    // try to connect 
    try
    {
        if (Application::g_app)
        {
            Application::g_app->disconnectIBus();
            delete Application::g_app;
            Application::g_app = NULL;
        }
        
        Application::g_app = new Application(openbm::Address, openbm::Port, 0, openbm::ClientAliveTimeout);
        
        IBus::CTSType ccc = IBus::NO;
        if (cts < 0) ccc = IBus::NO;
        if (cts == 0) ccc = IBus::HARDWARE;
        if (cts > 0) ccc = IBus::GPIO;
        
        if (!Application::g_app->connectIBus(device, ccc)) return jint(-1);
        
    }catch(...)
    {
        return jint(-1);    
    }
    
    Application::g_app->setUseWebserver(false);
 
    LOG_DEBUG("Connected...");
    
    return jint(0);
}
  
