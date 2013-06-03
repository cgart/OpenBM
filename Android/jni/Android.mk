LOCAL_PATH := $(call my-dir)
NDK := ../ndk-sources
# /var/tmp/develop/android/android-ndk-r6b/

include $(CLEAR_VARS)
LOCAL_MODULE:= boost_thread
LOCAL_SRC_FILES:= $(NDK)/boost/android/lib/libboost_thread.a
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE:= boost_iostreams
LOCAL_SRC_FILES:= $(NDK)/boost/android/lib/libboost_iostreams.a
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE:= boost_date_time
LOCAL_SRC_FILES:= $(NDK)/boost/android/lib/libboost_date_time.a
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE:= boost_program_options
LOCAL_SRC_FILES:= $(NDK)/boost/android/lib/libboost_program_options.a
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := boost_system
LOCAL_SRC_FILES := $(NDK)/boost/android/lib/libboost_system.a
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)
include $(PREBUILT_STATIC_LIBRARY)



include $(CLEAR_VARS)

#LOCAL_C_INCLUDES := $(LOCAL_PATH)/../../boost

LOCAL_C_INCLUDES := $(NDK)/boost/

LOCAL_MODULE    := openbmgateway
LOCAL_SRC_FILES := main.cpp \
		  mongoose.c \
                  Application.cpp \
                  ApplicationClient.cpp \
                  IBus.cpp \
                  Log.cpp \
		  jni_native.cpp

LOCAL_LDLIBS := -llog
LOCAL_STATIC_LIBRARIES := boost_thread boost_date_time boost_program_options boost_system

#$(call import-module,boost)
include $(BUILD_SHARED_LIBRARY)
