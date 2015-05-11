LOCAL_PATH:= $(call my-dir)

###########################################

include $(CLEAR_VARS)
LOCAL_MODULE := librotation

LOCAL_MODULE_TAGS := optional 

LOCAL_SRC_FILES := rotation.cpp 

LOCAL_SHARED_LIBRARIES := libutils libbinder

include $(BUILD_STATIC_LIBRARY)
###########################################

#
# libmediaplayerservice
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=               \
    ActivityManager.cpp         \
    Crypto.cpp                  \
    Drm.cpp                     \
    HDCP.cpp                    \
    MediaPlayerFactory.cpp      \
    MediaPlayerService.cpp      \
    MediaRecorderClient.cpp     \
    MetadataRetrieverClient.cpp \
    MidiFile.cpp                \
    MidiMetadataRetriever.cpp   \
    RemoteDisplay.cpp           \
<<<<<<< HEAD
    CedarPlayer.cpp       	\
    StagefrightPlayer.cpp       \
    StagefrightRecorder.cpp     \
    TestPlayerStub.cpp          \
    CedarAPlayerWrapper.cpp	\
    SimpleMediaFormatProbe.cpp	\
    MovAvInfoDetect.cpp         \
=======
    SharedLibrary.cpp           \
    StagefrightPlayer.cpp       \
    StagefrightRecorder.cpp     \
    TestPlayerStub.cpp          \
    VideoFrameScheduler.cpp     \
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e

LOCAL_SHARED_LIBRARIES :=       \
    libbinder                   \
    libcamera_client            \
    libcrypto                   \
    libcutils                   \
    libdrmframework             \
    liblog                      \
    libdl                       \
    libgui                      \
    libmedia                    \
    libsonivox                  \
    libCedarX           	\
    libCedarA           	\
    libstagefright              \
    libstagefright_foundation   \
    libstagefright_httplive     \
    libstagefright_omx          \
    libstagefright_wfd          \
    libutils                    \
    libvorbisidec               \
    libdrmframework             \

LOCAL_STATIC_LIBRARIES :=       \
    libstagefright_nuplayer     \
    libstagefright_rtsp         \
    librotation                 \

LOCAL_C_INCLUDES :=                                                 \
<<<<<<< HEAD
    $(call include-path-for, graphics corecg)                       \
    $(TOP)/frameworks/av/media/CedarX-Projects/CedarXAndroid/IceCreamSandwich \
    $(TOP)/frameworks/av/media/CedarX-Projects/CedarX/include/include_audio \
    $(TOP)/frameworks/av/media/CedarX-Projects/CedarX/include/include_cedarv \
    $(TOP)/frameworks/av/media/CedarX-Projects/CedarX/include \
    $(TOP)/frameworks/av/media/CedarX-Projects/CedarA \
    $(TOP)/frameworks/av/media/CedarX-Projects/CedarA/include \
=======
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    $(TOP)/frameworks/av/media/libstagefright/include               \
    $(TOP)/frameworks/av/media/libstagefright/rtsp                  \
    $(TOP)/frameworks/av/media/libstagefright/wifi-display          \
    $(TOP)/frameworks/av/media/libstagefright/webm                  \
    $(TOP)/frameworks/native/include/media/openmax                  \
    $(TOP)/external/tremolo/Tremolo

LOCAL_CFLAGS +=-DCEDARX_ANDROID_VERSION=9

ifeq ($(BOARD_USES_QCOM_HARDWARE),true)
    ifeq ($(TARGET_QCOM_MEDIA_VARIANT),caf)
    LOCAL_C_INCLUDES += \
            $(TOP)/hardware/qcom/media-caf/mm-core/inc
    else
    LOCAL_C_INCLUDES += \
            $(TOP)/hardware/qcom/media/mm-core/inc
    endif
endif

LOCAL_MODULE:= libmediaplayerservice

LOCAL_32_BIT_ONLY := true

ifeq ($(TARGET_ENABLE_QC_AV_ENHANCEMENTS),true)
    LOCAL_CFLAGS += -DENABLE_AV_ENHANCEMENTS
    LOCAL_C_INCLUDES += $(TOP)/frameworks/av/include/media
    LOCAL_C_INCLUDES += $(TOP)/$(call project-path-for,qcom-media)/mm-core/inc
endif

include $(BUILD_SHARED_LIBRARY)

include $(call all-makefiles-under,$(LOCAL_PATH))
