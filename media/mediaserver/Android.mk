LOCAL_PATH:= $(call my-dir)

ifneq ($(BOARD_USE_CUSTOM_MEDIASERVEREXTENSIONS),true)
include $(CLEAR_VARS)
LOCAL_SRC_FILES := register.cpp
LOCAL_MODULE := libregistermsext
LOCAL_MODULE_TAGS := optional
include $(BUILD_STATIC_LIBRARY)
endif

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	main_mediaserver.cpp 

LOCAL_SHARED_LIBRARIES := \
	libaudioflinger \
	libaudiopolicyservice \
	libcamera_metadata\
	libcameraservice \
	libmedialogservice \
	libcutils \
	libnbaio \
	libmedia \
	libmediaplayerservice \
	libutils \
	liblog \
	libbinder \
	libsoundtriggerservice

LOCAL_STATIC_LIBRARIES := \
	libregistermsext

<<<<<<< HEAD
ifeq ($(BOARD_USE_SECTVOUT),true)
	LOCAL_CFLAGS += -DSECTVOUT
	LOCAL_SHARED_LIBRARIES += libTVOut
endif

ifeq ($(TARGET_QCOM_AUDIO_VARIANT),caf)
	LOCAL_CFLAGS += -DQCOM_ENHANCED_AUDIO
endif

# FIXME The duplicate audioflinger is temporary
=======
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
LOCAL_C_INCLUDES := \
    frameworks/av/media/libmediaplayerservice \
    frameworks/av/services/medialog \
    frameworks/av/services/audioflinger \
    frameworks/av/services/audiopolicy \
    frameworks/av/services/camera/libcameraservice \
    $(call include-path-for, audio-utils) \
    frameworks/av/services/soundtrigger

ifeq ($(strip $(AUDIO_FEATURE_ENABLED_LISTEN)),true)
  LOCAL_SHARED_LIBRARIES += liblisten
  LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/audio-listen
  LOCAL_CFLAGS += -DAUDIO_LISTEN_ENABLED
endif

LOCAL_MODULE:= mediaserver
LOCAL_32_BIT_ONLY := true

include $(BUILD_EXECUTABLE)
