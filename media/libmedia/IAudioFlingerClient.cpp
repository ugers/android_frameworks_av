/*
<<<<<<< HEAD
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
 * Not a Contribution, Apache license notifications and license are retained
 * for attribution purposes only.
=======
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Not a Contribution.
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
 *
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "IAudioFlingerClient"
#include <utils/Log.h>

#include <stdint.h>
#include <sys/types.h>

#include <binder/Parcel.h>

#include <media/IAudioFlingerClient.h>
#include <media/AudioSystem.h>

namespace android {

enum {
    IO_CONFIG_CHANGED = IBinder::FIRST_CALL_TRANSACTION
};

class BpAudioFlingerClient : public BpInterface<IAudioFlingerClient>
{
public:
    BpAudioFlingerClient(const sp<IBinder>& impl)
        : BpInterface<IAudioFlingerClient>(impl)
    {
    }

    void ioConfigChanged(int event, audio_io_handle_t ioHandle, const void *param2)
    {
        Parcel data, reply;
        data.writeInterfaceToken(IAudioFlingerClient::getInterfaceDescriptor());
        data.writeInt32(event);
        data.writeInt32((int32_t) ioHandle);
        if (event == AudioSystem::STREAM_CONFIG_CHANGED) {
            uint32_t stream = *(const uint32_t *)param2;
            ALOGV("ioConfigChanged stream %d", stream);
            data.writeInt32(stream);
<<<<<<< HEAD
        } else if (event != AudioSystem::OUTPUT_CLOSED &&
#ifdef QCOM_HARDWARE
                        event != AudioSystem::EFFECT_CONFIG_CHANGED &&
#endif
                        event != AudioSystem::INPUT_CLOSED) {
=======
        } else if (event != AudioSystem::OUTPUT_CLOSED && event != AudioSystem::INPUT_CLOSED
#ifdef QCOM_DIRECTTRACK
                   && event != AudioSystem::EFFECT_CONFIG_CHANGED
#endif
            ) {
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
            const AudioSystem::OutputDescriptor *desc = (const AudioSystem::OutputDescriptor *)param2;
            data.writeInt32(desc->samplingRate);
            data.writeInt32(desc->format);
            data.writeInt32(desc->channelMask);
            data.writeInt64(desc->frameCount);
            data.writeInt32(desc->latency);
        }
        remote()->transact(IO_CONFIG_CHANGED, data, &reply, IBinder::FLAG_ONEWAY);
    }
};

IMPLEMENT_META_INTERFACE(AudioFlingerClient, "android.media.IAudioFlingerClient");

// ----------------------------------------------------------------------

status_t BnAudioFlingerClient::onTransact(
    uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags)
{
    switch (code) {
    case IO_CONFIG_CHANGED: {
            CHECK_INTERFACE(IAudioFlingerClient, data, reply);
            int event = data.readInt32();
            audio_io_handle_t ioHandle = (audio_io_handle_t) data.readInt32();
            const void *param2 = NULL;
            AudioSystem::OutputDescriptor desc;
            uint32_t stream;
            if (event == AudioSystem::STREAM_CONFIG_CHANGED) {
                stream = data.readInt32();
                param2 = &stream;
                ALOGV("STREAM_CONFIG_CHANGED stream %d", stream);
            } else if (event != AudioSystem::OUTPUT_CLOSED && event != AudioSystem::INPUT_CLOSED) {
                desc.samplingRate = data.readInt32();
                desc.format = (audio_format_t) data.readInt32();
                desc.channelMask = (audio_channel_mask_t) data.readInt32();
                desc.frameCount = data.readInt64();
                desc.latency = data.readInt32();
                param2 = &desc;
            }
            ioConfigChanged(event, ioHandle, param2);
            return NO_ERROR;
        } break;
        default:
            return BBinder::onTransact(code, data, reply, flags);
    }
}

// ----------------------------------------------------------------------------

}; // namespace android
