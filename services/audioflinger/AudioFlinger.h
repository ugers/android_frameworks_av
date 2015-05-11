/*
** Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
** Not a Contribution.
** Copyright 2007, The Android Open Source Project
** Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
**
** Not a Contribution, Apache license notifications and license are retained
** for attribution purposes only.
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#ifndef ANDROID_AUDIO_FLINGER_H
#define ANDROID_AUDIO_FLINGER_H

#include "Configuration.h"
#include <stdint.h>
#include <sys/types.h>
#include <limits.h>

#include <common_time/cc_helper.h>

#include <cutils/compiler.h>

#include <media/IAudioFlinger.h>
#include <media/IAudioFlingerClient.h>
<<<<<<< HEAD
#ifdef QCOM_HARDWARE
=======
#ifdef QCOM_DIRECTTRACK
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
#include <media/IDirectTrack.h>
#include <media/IDirectTrackClient.h>
#endif
#include <media/IAudioTrack.h>
#include <media/IAudioRecord.h>
#include <media/AudioSystem.h>
#include <media/AudioTrack.h>

#include <utils/Atomic.h>
#include <utils/Errors.h>
#include <utils/threads.h>
#include <utils/SortedVector.h>
#include <utils/TypeHelpers.h>
#include <utils/Vector.h>

#include <binder/BinderService.h>
#include <binder/MemoryDealer.h>

#include <system/audio.h>
#include <hardware/audio.h>
#include <hardware/audio_policy.h>

#include <media/AudioBufferProvider.h>
#include <media/ExtendedAudioBufferProvider.h>

#include "FastCapture.h"
#include "FastMixer.h"
#include <media/nbaio/NBAIO.h>
#include "AudioWatchdog.h"
#include "AudioMixer.h"

#include <powermanager/IPowerManager.h>
#include <utils/List.h>

#include <media/nbaio/NBLog.h>
#include <private/media/AudioTrackShared.h>
#include <utils/List.h>

namespace android {

struct audio_track_cblk_t;
struct effect_param_cblk_t;
class AudioMixer;
class AudioBuffer;
class AudioResampler;
class FastMixer;
class ServerProxy;

// ----------------------------------------------------------------------------

// AudioFlinger has a hard-coded upper limit of 2 channels for capture and playback.
// There is support for > 2 channel tracks down-mixed to 2 channel output via a down-mix effect.
// Adding full support for > 2 channel capture or playback would require more than simply changing
// this #define.  There is an independent hard-coded upper limit in AudioMixer;
// removing that AudioMixer limit would be necessary but insufficient to support > 2 channels.
// The macro FCC_2 highlights some (but not all) places where there is are 2-channel assumptions.
// Search also for "2", "left", "right", "[0]", "[1]", ">> 16", "<< 16", etc.
#define FCC_2 2     // FCC_2 = Fixed Channel Count 2

static const nsecs_t kDefaultStandbyTimeInNsecs = seconds(3);

#define INCLUDING_FROM_AUDIOFLINGER_H

class AudioFlinger :
    public BinderService<AudioFlinger>,
    public BnAudioFlinger
{
    friend class BinderService<AudioFlinger>;   // for AudioFlinger()
public:
    static const char* getServiceName() ANDROID_API { return "media.audio_flinger"; }

    virtual     status_t    dump(int fd, const Vector<String16>& args);

    // IAudioFlinger interface, in binder opcode order
    virtual sp<IAudioTrack> createTrack(
                                audio_stream_type_t streamType,
                                uint32_t sampleRate,
                                audio_format_t format,
                                audio_channel_mask_t channelMask,
                                size_t *pFrameCount,
                                IAudioFlinger::track_flags_t *flags,
                                const sp<IMemory>& sharedBuffer,
                                audio_io_handle_t output,
                                pid_t tid,
                                int *sessionId,
                                int clientUid,
                                status_t *status /*non-NULL*/);

#ifdef QCOM_DIRECTTRACK
    virtual sp<IDirectTrack> createDirectTrack(
                                pid_t pid,
                                uint32_t sampleRate,
                                audio_channel_mask_t channelMask,
                                audio_io_handle_t output,
                                int *sessionId,
                                IDirectTrackClient* client,
                                audio_stream_type_t streamType,
                                status_t *status);
<<<<<<< HEAD
#ifdef QCOM_HARDWARE
    virtual sp<IDirectTrack> createDirectTrack(
                                pid_t pid,
                                uint32_t sampleRate,
                                audio_channel_mask_t channelMask,
                                audio_io_handle_t output,
                                int *sessionId,
                                IDirectTrackClient* client,
                                audio_stream_type_t streamType,
                                status_t *status);

=======
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    virtual void deleteEffectSession();
#endif

    virtual sp<IAudioRecord> openRecord(
                                audio_io_handle_t input,
                                uint32_t sampleRate,
                                audio_format_t format,
                                audio_channel_mask_t channelMask,
                                size_t *pFrameCount,
                                IAudioFlinger::track_flags_t *flags,
                                pid_t tid,
                                int *sessionId,
                                size_t *notificationFrames,
                                sp<IMemory>& cblk,
                                sp<IMemory>& buffers,
                                status_t *status /*non-NULL*/);

    virtual     uint32_t    sampleRate(audio_io_handle_t output) const;
    virtual     audio_format_t format(audio_io_handle_t output) const;
    virtual     size_t      frameCount(audio_io_handle_t output) const;
    virtual     uint32_t    latency(audio_io_handle_t output) const;

    virtual     status_t    setMasterVolume(float value);
    virtual     status_t    setMasterMute(bool muted);

    virtual     float       masterVolume() const;
    virtual     bool        masterMute() const;

    virtual     status_t    setStreamVolume(audio_stream_type_t stream, float value,
                                            audio_io_handle_t output);
    virtual     status_t    setStreamMute(audio_stream_type_t stream, bool muted);

    virtual     float       streamVolume(audio_stream_type_t stream,
                                         audio_io_handle_t output) const;
    virtual     bool        streamMute(audio_stream_type_t stream) const;

    virtual     status_t    setMode(audio_mode_t mode);

    virtual     status_t    setMicMute(bool state);
    virtual     bool        getMicMute() const;

    virtual     status_t    setParameters(audio_io_handle_t ioHandle, const String8& keyValuePairs);
    virtual     String8     getParameters(audio_io_handle_t ioHandle, const String8& keys) const;

    virtual     void        registerClient(const sp<IAudioFlingerClient>& client);
<<<<<<< HEAD
#ifdef QCOM_HARDWARE
    virtual status_t deregisterClient(const sp<IAudioFlingerClient>& client);
=======
#ifdef QCOM_DIRECTTRACK
    virtual    status_t     deregisterClient(const sp<IAudioFlingerClient>& client);
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
#endif
    virtual     size_t      getInputBufferSize(uint32_t sampleRate, audio_format_t format,
                                               audio_channel_mask_t channelMask) const;

    virtual status_t openOutput(audio_module_handle_t module,
                                audio_io_handle_t *output,
                                audio_config_t *config,
                                audio_devices_t *devices,
                                const String8& address,
                                uint32_t *latencyMs,
                                audio_output_flags_t flags);

    virtual audio_io_handle_t openDuplicateOutput(audio_io_handle_t output1,
                                                  audio_io_handle_t output2);

    virtual status_t closeOutput(audio_io_handle_t output);

    virtual status_t suspendOutput(audio_io_handle_t output);

    virtual status_t restoreOutput(audio_io_handle_t output);

    virtual status_t openInput(audio_module_handle_t module,
                               audio_io_handle_t *input,
                               audio_config_t *config,
                               audio_devices_t *device,
                               const String8& address,
                               audio_source_t source,
                               audio_input_flags_t flags);

    virtual status_t closeInput(audio_io_handle_t input);

    virtual status_t invalidateStream(audio_stream_type_t stream);

    virtual status_t setVoiceVolume(float volume);

    virtual status_t getRenderPosition(uint32_t *halFrames, uint32_t *dspFrames,
                                       audio_io_handle_t output) const;

    virtual uint32_t getInputFramesLost(audio_io_handle_t ioHandle) const;

    virtual audio_unique_id_t newAudioUniqueId();

    virtual void acquireAudioSessionId(int audioSession, pid_t pid);

    virtual void releaseAudioSessionId(int audioSession, pid_t pid);

    virtual status_t queryNumberEffects(uint32_t *numEffects) const;

    virtual status_t queryEffect(uint32_t index, effect_descriptor_t *descriptor) const;

    virtual status_t getEffectDescriptor(const effect_uuid_t *pUuid,
                                         effect_descriptor_t *descriptor) const;

    virtual sp<IEffect> createEffect(
                        effect_descriptor_t *pDesc,
                        const sp<IEffectClient>& effectClient,
                        int32_t priority,
                        audio_io_handle_t io,
                        int sessionId,
                        status_t *status /*non-NULL*/,
                        int *id,
                        int *enabled);

    virtual status_t moveEffects(int sessionId, audio_io_handle_t srcOutput,
                        audio_io_handle_t dstOutput);

#ifdef QCOM_FM_ENABLED
    virtual status_t setFmVolume(float volume);
#endif

    virtual audio_module_handle_t loadHwModule(const char *name);

    virtual uint32_t getPrimaryOutputSamplingRate();
    virtual size_t getPrimaryOutputFrameCount();

    virtual status_t setLowRamDevice(bool isLowRamDevice);

    /* List available audio ports and their attributes */
    virtual status_t listAudioPorts(unsigned int *num_ports,
                                    struct audio_port *ports);

    /* Get attributes for a given audio port */
    virtual status_t getAudioPort(struct audio_port *port);

    /* Create an audio patch between several source and sink ports */
    virtual status_t createAudioPatch(const struct audio_patch *patch,
                                       audio_patch_handle_t *handle);

    /* Release an audio patch */
    virtual status_t releaseAudioPatch(audio_patch_handle_t handle);

    /* List existing audio patches */
    virtual status_t listAudioPatches(unsigned int *num_patches,
                                      struct audio_patch *patches);

    /* Set audio port configuration */
    virtual status_t setAudioPortConfig(const struct audio_port_config *config);

    /* Get the HW synchronization source used for an audio session */
    virtual audio_hw_sync_t getAudioHwSyncForSession(audio_session_t sessionId);

    virtual     status_t    onTransact(
                                uint32_t code,
                                const Parcel& data,
                                Parcel* reply,
                                uint32_t flags);

<<<<<<< HEAD
#ifdef QCOM_HARDWARE
=======
#ifdef QCOM_DIRECTTRACK
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    bool applyEffectsOn(void *token,
                        int16_t *buffer1,
                        int16_t *buffer2,
                        int size,
                        bool force);
#endif

    // end of IAudioFlinger interface

    sp<NBLog::Writer>   newWriter_l(size_t size, const char *name);
    void                unregisterWriter(const sp<NBLog::Writer>& writer);
private:
    static const size_t kLogMemorySize = 40 * 1024;
    sp<MemoryDealer>    mLogMemoryDealer;   // == 0 when NBLog is disabled
    // When a log writer is unregistered, it is done lazily so that media.log can continue to see it
    // for as long as possible.  The memory is only freed when it is needed for another log writer.
    Vector< sp<NBLog::Writer> > mUnregisteredWriters;
    Mutex               mUnregisteredWritersLock;
public:

    class SyncEvent;

    typedef void (*sync_event_callback_t)(const wp<SyncEvent>& event) ;

    class SyncEvent : public RefBase {
    public:
        SyncEvent(AudioSystem::sync_event_t type,
                  int triggerSession,
                  int listenerSession,
                  sync_event_callback_t callBack,
                  wp<RefBase> cookie)
        : mType(type), mTriggerSession(triggerSession), mListenerSession(listenerSession),
          mCallback(callBack), mCookie(cookie)
        {}

        virtual ~SyncEvent() {}

        void trigger() { Mutex::Autolock _l(mLock); if (mCallback) mCallback(this); }
        bool isCancelled() const { Mutex::Autolock _l(mLock); return (mCallback == NULL); }
        void cancel() { Mutex::Autolock _l(mLock); mCallback = NULL; }
        AudioSystem::sync_event_t type() const { return mType; }
        int triggerSession() const { return mTriggerSession; }
        int listenerSession() const { return mListenerSession; }
        wp<RefBase> cookie() const { return mCookie; }

    private:
          const AudioSystem::sync_event_t mType;
          const int mTriggerSession;
          const int mListenerSession;
          sync_event_callback_t mCallback;
          const wp<RefBase> mCookie;
          mutable Mutex mLock;
    };

    sp<SyncEvent> createSyncEvent(AudioSystem::sync_event_t type,
                                        int triggerSession,
                                        int listenerSession,
                                        sync_event_callback_t callBack,
                                        wp<RefBase> cookie);

private:
    class AudioHwDevice;    // fwd declaration for findSuitableHwDev_l

               audio_mode_t getMode() const { return mMode; }

                bool        btNrecIsOff() const { return mBtNrecIsOff; }

                            AudioFlinger() ANDROID_API;
    virtual                 ~AudioFlinger();

    // call in any IAudioFlinger method that accesses mPrimaryHardwareDev
    status_t                initCheck() const { return mPrimaryHardwareDev == NULL ?
                                                        NO_INIT : NO_ERROR; }

    // RefBase
    virtual     void        onFirstRef();

    AudioHwDevice*          findSuitableHwDev_l(audio_module_handle_t module,
                                                audio_devices_t devices);
    void                    purgeStaleEffects_l();

    // Set kEnableExtendedChannels to true to enable greater than stereo output
    // for the MixerThread and device sink.  Number of channels allowed is
    // FCC_2 <= channels <= AudioMixer::MAX_NUM_CHANNELS.
    static const bool kEnableExtendedChannels = true;

    // Returns true if channel mask is permitted for the PCM sink in the MixerThread
    static inline bool isValidPcmSinkChannelMask(audio_channel_mask_t channelMask) {
        switch (audio_channel_mask_get_representation(channelMask)) {
        case AUDIO_CHANNEL_REPRESENTATION_POSITION: {
            uint32_t channelCount = FCC_2; // stereo is default
            if (kEnableExtendedChannels) {
                channelCount = audio_channel_count_from_out_mask(channelMask);
                if (channelCount < FCC_2 // mono is not supported at this time
                        || channelCount > AudioMixer::MAX_NUM_CHANNELS) {
                    return false;
                }
            }
            // check that channelMask is the "canonical" one we expect for the channelCount.
            return channelMask == audio_channel_out_mask_from_count(channelCount);
            }
        default:
            return false;
        }
    }

    // Set kEnableExtendedPrecision to true to use extended precision in MixerThread
    static const bool kEnableExtendedPrecision = true;

    // Returns true if format is permitted for the PCM sink in the MixerThread
    static inline bool isValidPcmSinkFormat(audio_format_t format) {
        switch (format) {
        case AUDIO_FORMAT_PCM_16_BIT:
            return true;
        case AUDIO_FORMAT_PCM_FLOAT:
        case AUDIO_FORMAT_PCM_24_BIT_PACKED:
        case AUDIO_FORMAT_PCM_32_BIT:
        case AUDIO_FORMAT_PCM_8_24_BIT:
            return kEnableExtendedPrecision;
        default:
            return false;
        }
    }

    // standby delay for MIXER and DUPLICATING playback threads is read from property
    // ro.audio.flinger_standbytime_ms or defaults to kDefaultStandbyTimeInNsecs
    static nsecs_t          mStandbyTimeInNsecs;

    // incremented by 2 when screen state changes, bit 0 == 1 means "off"
    // AudioFlinger::setParameters() updates, other threads read w/o lock
    static uint32_t         mScreenState;

    // Internal dump utilities.
    static const int kDumpLockRetries = 50;
    static const int kDumpLockSleepUs = 20000;
    static bool dumpTryLock(Mutex& mutex);
    void dumpPermissionDenial(int fd, const Vector<String16>& args);
    void dumpClients(int fd, const Vector<String16>& args);
    void dumpInternals(int fd, const Vector<String16>& args);

    // --- Client ---
    class Client : public RefBase {
    public:
                            Client(const sp<AudioFlinger>& audioFlinger, pid_t pid);
        virtual             ~Client();
        sp<MemoryDealer>    heap() const;
        pid_t               pid() const { return mPid; }
        sp<AudioFlinger>    audioFlinger() const { return mAudioFlinger; }

        bool reserveTimedTrack();
        void releaseTimedTrack();

    private:
                            Client(const Client&);
                            Client& operator = (const Client&);
        const sp<AudioFlinger> mAudioFlinger;
        const sp<MemoryDealer> mMemoryDealer;
        const pid_t         mPid;

        Mutex               mTimedTrackLock;
        int                 mTimedTrackCount;
    };

    // --- Notification Client ---
    class NotificationClient : public IBinder::DeathRecipient {
    public:
                            NotificationClient(const sp<AudioFlinger>& audioFlinger,
                                                const sp<IAudioFlingerClient>& client,
                                                sp<IBinder> binder);
        virtual             ~NotificationClient();

                sp<IAudioFlingerClient> audioFlingerClient() const { return mAudioFlingerClient; }

                // IBinder::DeathRecipient
                virtual     void        binderDied(const wp<IBinder>& who);

    private:
                            NotificationClient(const NotificationClient&);
                            NotificationClient& operator = (const NotificationClient&);

        const sp<AudioFlinger>  mAudioFlinger;
        sp<IBinder>             mBinder;
        const sp<IAudioFlingerClient> mAudioFlingerClient;
    };

    class TrackHandle;
    class RecordHandle;
    class RecordThread;
    class PlaybackThread;
    class MixerThread;
    class DirectOutputThread;
    class OffloadThread;
    class DuplicatingThread;
    class AsyncCallbackThread;
    class Track;
    class RecordTrack;
    class EffectModule;
    class EffectHandle;
    class EffectChain;
<<<<<<< HEAD
#ifdef QCOM_HARDWARE
=======
#ifdef QCOM_DIRECTTRACK
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    struct AudioSessionDescriptor;
#endif
    struct AudioStreamOut;
    struct AudioStreamIn;

<<<<<<< HEAD
    class ThreadBase : public Thread {
    public:

        enum type_t {
            MIXER,              // Thread class is MixerThread
            DIRECT,             // Thread class is DirectOutputThread
            DUPLICATING,        // Thread class is DuplicatingThread
            RECORD              // Thread class is RecordThread
        };

        ThreadBase (const sp<AudioFlinger>& audioFlinger, audio_io_handle_t id,
                    audio_devices_t outDevice, audio_devices_t inDevice, type_t type);
        virtual             ~ThreadBase();

        void dumpBase(int fd, const Vector<String16>& args);
        void dumpEffectChains(int fd, const Vector<String16>& args);

        void clearPowerManager();

        // base for record and playback
        class TrackBase : public ExtendedAudioBufferProvider, public RefBase {

        public:
            enum track_state {
                IDLE,
                TERMINATED,
                FLUSHED,
                STOPPED,
                // next 2 states are currently used for fast tracks only
                STOPPING_1,     // waiting for first underrun
                STOPPING_2,     // waiting for presentation complete
                RESUMING,
                ACTIVE,
                PAUSING,
                PAUSED
            };

                                TrackBase(ThreadBase *thread,
                                        const sp<Client>& client,
                                        uint32_t sampleRate,
                                        audio_format_t format,
                                        audio_channel_mask_t channelMask,
                                        int frameCount,
#ifdef QCOM_ENHANCED_AUDIO
                                        uint32_t flags,
#endif
                                        const sp<IMemory>& sharedBuffer,
                                        int sessionId);
            virtual             ~TrackBase();

            virtual status_t    start(AudioSystem::sync_event_t event,
                                     int triggerSession) = 0;
            virtual void        stop() = 0;
                    sp<IMemory> getCblk() const { return mCblkMemory; }
                    audio_track_cblk_t* cblk() const { return mCblk; }
                    int         sessionId() const { return mSessionId; }
            virtual status_t    setSyncEvent(const sp<SyncEvent>& event);

        protected:
                                TrackBase(const TrackBase&);
                                TrackBase& operator = (const TrackBase&);

            // AudioBufferProvider interface
            virtual status_t getNextBuffer(AudioBufferProvider::Buffer* buffer, int64_t pts) = 0;
            virtual void releaseBuffer(AudioBufferProvider::Buffer* buffer);

            // ExtendedAudioBufferProvider interface is only needed for Track,
            // but putting it in TrackBase avoids the complexity of virtual inheritance
            virtual size_t  framesReady() const { return SIZE_MAX; }

            audio_format_t format() const {
                return mFormat;
            }

            int channelCount() const { return mChannelCount; }

            audio_channel_mask_t channelMask() const { return mChannelMask; }

            int sampleRate() const; // FIXME inline after cblk sr moved

            // Return a pointer to the start of a contiguous slice of the track buffer.
            // Parameter 'offset' is the requested start position, expressed in
            // monotonically increasing frame units relative to the track epoch.
            // Parameter 'frames' is the requested length, also in frame units.
            // Always returns non-NULL.  It is the caller's responsibility to
            // verify that this will be successful; the result of calling this
            // function with invalid 'offset' or 'frames' is undefined.
            void* getBuffer(uint32_t offset, uint32_t frames) const;

            bool isStopped() const {
                return (mState == STOPPED || mState == FLUSHED);
            }

            // for fast tracks only
            bool isStopping() const {
                return mState == STOPPING_1 || mState == STOPPING_2;
            }
            bool isStopping_1() const {
                return mState == STOPPING_1;
            }
            bool isStopping_2() const {
                return mState == STOPPING_2;
            }

            bool isTerminated() const {
                return mState == TERMINATED;
            }

            bool step();
            void reset();

            const wp<ThreadBase> mThread;
            /*const*/ sp<Client> mClient;   // see explanation at ~TrackBase() why not const
            sp<IMemory>         mCblkMemory;
            audio_track_cblk_t* mCblk;
            void*               mBuffer;    // start of track buffer, typically in shared memory
            void*               mBufferEnd; // &mBuffer[mFrameCount * frameSize], where frameSize
                                            //   is based on mChannelCount and 16-bit samples
            uint32_t            mFrameCount;
            // we don't really need a lock for these
            track_state         mState;
            const uint32_t      mSampleRate;    // initial sample rate only; for tracks which
                                // support dynamic rates, the current value is in control block
            const audio_format_t mFormat;
            bool                mStepServerFailed;
#ifdef QCOM_ENHANCED_AUDIO
            uint32_t            mFlags;
#endif
            const int           mSessionId;
            uint8_t             mChannelCount;
            audio_channel_mask_t mChannelMask;
            Vector < sp<SyncEvent> >mSyncEvents;
        };

        enum {
            CFG_EVENT_IO,
            CFG_EVENT_PRIO
        };

        class ConfigEvent {
        public:
            ConfigEvent(int type) : mType(type) {}
            virtual ~ConfigEvent() {}

                     int type() const { return mType; }

            virtual  void dump(char *buffer, size_t size) = 0;

        private:
            const int mType;
        };

        class IoConfigEvent : public ConfigEvent {
        public:
            IoConfigEvent(int event, int param) :
                ConfigEvent(CFG_EVENT_IO), mEvent(event), mParam(event) {}
            virtual ~IoConfigEvent() {}

                    int event() const { return mEvent; }
                    int param() const { return mParam; }

            virtual  void dump(char *buffer, size_t size) {
                snprintf(buffer, size, "IO event: event %d, param %d\n", mEvent, mParam);
            }

        private:
            const int mEvent;
            const int mParam;
        };

        class PrioConfigEvent : public ConfigEvent {
        public:
            PrioConfigEvent(pid_t pid, pid_t tid, int32_t prio) :
                ConfigEvent(CFG_EVENT_PRIO), mPid(pid), mTid(tid), mPrio(prio) {}
            virtual ~PrioConfigEvent() {}

                    pid_t pid() const { return mPid; }
                    pid_t tid() const { return mTid; }
                    int32_t prio() const { return mPrio; }

            virtual  void dump(char *buffer, size_t size) {
                snprintf(buffer, size, "Prio event: pid %d, tid %d, prio %d\n", mPid, mTid, mPrio);
            }

        private:
            const pid_t mPid;
            const pid_t mTid;
            const int32_t mPrio;
        };


        class PMDeathRecipient : public IBinder::DeathRecipient {
        public:
                        PMDeathRecipient(const wp<ThreadBase>& thread) : mThread(thread) {}
            virtual     ~PMDeathRecipient() {}

            // IBinder::DeathRecipient
            virtual     void        binderDied(const wp<IBinder>& who);

        private:
                        PMDeathRecipient(const PMDeathRecipient&);
                        PMDeathRecipient& operator = (const PMDeathRecipient&);

            wp<ThreadBase> mThread;
        };

        virtual     status_t    initCheck() const = 0;

                    // static externally-visible
                    type_t      type() const { return mType; }
                    audio_io_handle_t id() const { return mId;}

                    // dynamic externally-visible
                    uint32_t    sampleRate() const { return mSampleRate; }
                    int         channelCount() const { return mChannelCount; }
                    audio_channel_mask_t channelMask() const { return mChannelMask; }
                    audio_format_t format() const { return mFormat; }
                    // Called by AudioFlinger::frameCount(audio_io_handle_t output) and effects,
                    // and returns the normal mix buffer's frame count.
                    size_t      frameCount() const { return mNormalFrameCount; }
                    // Return's the HAL's frame count i.e. fast mixer buffer size.
                    size_t      frameCountHAL() const { return mFrameCount; }

        // Should be "virtual status_t requestExitAndWait()" and override same
        // method in Thread, but Thread::requestExitAndWait() is not yet virtual.
                    void        exit();
        virtual     bool        checkForNewParameters_l() = 0;
        virtual     status_t    setParameters(const String8& keyValuePairs);
        virtual     String8     getParameters(const String8& keys) = 0;
        virtual     void        audioConfigChanged_l(int event, int param = 0) = 0;
#ifdef QCOM_HARDWARE
                    void        effectConfigChanged();
#endif
                    void        sendIoConfigEvent(int event, int param = 0);
                    void        sendIoConfigEvent_l(int event, int param = 0);
                    void        sendPrioConfigEvent_l(pid_t pid, pid_t tid, int32_t prio);
                    void        processConfigEvents();

                    // see note at declaration of mStandby, mOutDevice and mInDevice
                    bool        standby() const { return mStandby; }
                    audio_devices_t outDevice() const { return mOutDevice; }
                    audio_devices_t inDevice() const { return mInDevice; }

        virtual     audio_stream_t* stream() const = 0;

                    sp<EffectHandle> createEffect_l(
                                        const sp<AudioFlinger::Client>& client,
                                        const sp<IEffectClient>& effectClient,
                                        int32_t priority,
                                        int sessionId,
                                        effect_descriptor_t *desc,
                                        int *enabled,
                                        status_t *status);
                    void disconnectEffect(const sp< EffectModule>& effect,
                                          EffectHandle *handle,
                                          bool unpinIfLast);

                    // return values for hasAudioSession (bit field)
                    enum effect_state {
                        EFFECT_SESSION = 0x1,   // the audio session corresponds to at least one
                                                // effect
                        TRACK_SESSION = 0x2     // the audio session corresponds to at least one
                                                // track
                    };

                    // get effect chain corresponding to session Id.
                    sp<EffectChain> getEffectChain(int sessionId);
                    // same as getEffectChain() but must be called with ThreadBase mutex locked
                    sp<EffectChain> getEffectChain_l(int sessionId) const;
                    // add an effect chain to the chain list (mEffectChains)
        virtual     status_t addEffectChain_l(const sp<EffectChain>& chain) = 0;
                    // remove an effect chain from the chain list (mEffectChains)
        virtual     size_t removeEffectChain_l(const sp<EffectChain>& chain) = 0;
                    // lock all effect chains Mutexes. Must be called before releasing the
                    // ThreadBase mutex before processing the mixer and effects. This guarantees the
                    // integrity of the chains during the process.
                    // Also sets the parameter 'effectChains' to current value of mEffectChains.
                    void lockEffectChains_l(Vector< sp<EffectChain> >& effectChains);
                    // unlock effect chains after process
                    void unlockEffectChains(const Vector< sp<EffectChain> >& effectChains);
                    // set audio mode to all effect chains
                    void setMode(audio_mode_t mode);
                    // get effect module with corresponding ID on specified audio session
                    sp<AudioFlinger::EffectModule> getEffect(int sessionId, int effectId);
                    sp<AudioFlinger::EffectModule> getEffect_l(int sessionId, int effectId);
                    // add and effect module. Also creates the effect chain is none exists for
                    // the effects audio session
                    status_t addEffect_l(const sp< EffectModule>& effect);
                    // remove and effect module. Also removes the effect chain is this was the last
                    // effect
                    void removeEffect_l(const sp< EffectModule>& effect);
                    // detach all tracks connected to an auxiliary effect
        virtual     void detachAuxEffect_l(int effectId) {}
                    // returns either EFFECT_SESSION if effects on this audio session exist in one
                    // chain, or TRACK_SESSION if tracks on this audio session exist, or both
                    virtual uint32_t hasAudioSession(int sessionId) const = 0;
                    // the value returned by default implementation is not important as the
                    // strategy is only meaningful for PlaybackThread which implements this method
                    virtual uint32_t getStrategyForSession_l(int sessionId) { return 0; }

                    // suspend or restore effect according to the type of effect passed. a NULL
                    // type pointer means suspend all effects in the session
                    void setEffectSuspended(const effect_uuid_t *type,
                                            bool suspend,
                                            int sessionId = AUDIO_SESSION_OUTPUT_MIX);
                    // check if some effects must be suspended/restored when an effect is enabled
                    // or disabled
                    void checkSuspendOnEffectEnabled(const sp<EffectModule>& effect,
                                                     bool enabled,
                                                     int sessionId = AUDIO_SESSION_OUTPUT_MIX);
                    void checkSuspendOnEffectEnabled_l(const sp<EffectModule>& effect,
                                                       bool enabled,
                                                       int sessionId = AUDIO_SESSION_OUTPUT_MIX);

                    virtual status_t    setSyncEvent(const sp<SyncEvent>& event) = 0;
                    virtual bool        isValidSyncEvent(const sp<SyncEvent>& event) const = 0;


        mutable     Mutex                   mLock;

    protected:

                    // entry describing an effect being suspended in mSuspendedSessions keyed vector
                    class SuspendedSessionDesc : public RefBase {
                    public:
                        SuspendedSessionDesc() : mRefCount(0) {}

                        int mRefCount;          // number of active suspend requests
                        effect_uuid_t mType;    // effect type UUID
                    };

                    void        acquireWakeLock();
                    void        acquireWakeLock_l();
                    void        releaseWakeLock();
                    void        releaseWakeLock_l();
                    void setEffectSuspended_l(const effect_uuid_t *type,
                                              bool suspend,
                                              int sessionId);
                    // updated mSuspendedSessions when an effect suspended or restored
                    void        updateSuspendedSessions_l(const effect_uuid_t *type,
                                                          bool suspend,
                                                          int sessionId);
                    // check if some effects must be suspended when an effect chain is added
                    void checkSuspendOnAddEffectChain_l(const sp<EffectChain>& chain);

        virtual     void        preExit() { }

        friend class AudioFlinger;      // for mEffectChains

                    const type_t            mType;

                    // Used by parameters, config events, addTrack_l, exit
                    Condition               mWaitWorkCV;

                    const sp<AudioFlinger>  mAudioFlinger;
                    uint32_t                mSampleRate;
                    size_t                  mFrameCount;       // output HAL, direct output, record
                    size_t                  mNormalFrameCount; // normal mixer and effects
                    audio_channel_mask_t    mChannelMask;
                    uint16_t                mChannelCount;
                    size_t                  mFrameSize;
                    audio_format_t          mFormat;

                    // Parameter sequence by client: binder thread calling setParameters():
                    //  1. Lock mLock
                    //  2. Append to mNewParameters
                    //  3. mWaitWorkCV.signal
                    //  4. mParamCond.waitRelative with timeout
                    //  5. read mParamStatus
                    //  6. mWaitWorkCV.signal
                    //  7. Unlock
                    //
                    // Parameter sequence by server: threadLoop calling checkForNewParameters_l():
                    // 1. Lock mLock
                    // 2. If there is an entry in mNewParameters proceed ...
                    // 2. Read first entry in mNewParameters
                    // 3. Process
                    // 4. Remove first entry from mNewParameters
                    // 5. Set mParamStatus
                    // 6. mParamCond.signal
                    // 7. mWaitWorkCV.wait with timeout (this is to avoid overwriting mParamStatus)
                    // 8. Unlock
                    Condition               mParamCond;
                    Vector<String8>         mNewParameters;
                    status_t                mParamStatus;

                    Vector<ConfigEvent *>     mConfigEvents;

                    // These fields are written and read by thread itself without lock or barrier,
                    // and read by other threads without lock or barrier via standby() , outDevice()
                    // and inDevice().
                    // Because of the absence of a lock or barrier, any other thread that reads
                    // these fields must use the information in isolation, or be prepared to deal
                    // with possibility that it might be inconsistent with other information.
                    bool                    mStandby;   // Whether thread is currently in standby.
                    audio_devices_t         mOutDevice;   // output device
                    audio_devices_t         mInDevice;    // input device
                    audio_source_t          mAudioSource; // (see audio.h, audio_source_t)

                    const audio_io_handle_t mId;
                    Vector< sp<EffectChain> > mEffectChains;

                    static const int        kNameLength = 16;   // prctl(PR_SET_NAME) limit
                    char                    mName[kNameLength];
                    sp<IPowerManager>       mPowerManager;
                    sp<IBinder>             mWakeLockToken;
                    const sp<PMDeathRecipient> mDeathRecipient;
                    // list of suspended effects per session and per type. The first vector is
                    // keyed by session ID, the second by type UUID timeLow field
                    KeyedVector< int, KeyedVector< int, sp<SuspendedSessionDesc> > >  mSuspendedSessions;
    };

=======
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    struct  stream_type_t {
        stream_type_t()
            :   volume(1.0f),
                mute(false)
        {
        }
        float       volume;
        bool        mute;
    };

    // --- PlaybackThread ---

#include "Threads.h"

#include "Effects.h"

#include "PatchPanel.h"

    // server side of the client's IAudioTrack
#ifdef QCOM_HARDWARE
    class DirectAudioTrack : public android::BnDirectTrack,
                             public AudioEventObserver
    {
    public:
                            DirectAudioTrack(const sp<AudioFlinger>& audioFlinger,
                                             int output, AudioSessionDescriptor *outputDesc,
                                             IDirectTrackClient* client, audio_output_flags_t outflag);
        virtual             ~DirectAudioTrack();
        virtual status_t    start();
        virtual void        stop();
        virtual void        flush();
        virtual void        mute(bool);
        virtual void        pause();
        virtual ssize_t     write(const void *buffer, size_t bytes);
        virtual void        setVolume(float left, float right);
        virtual int64_t     getTimeStamp();
        virtual void        postEOS(int64_t delayUs);

        virtual status_t    onTransact(
            uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags);
    private:

        IDirectTrackClient* mClient;
        AudioSessionDescriptor *mOutputDesc;
        int  mOutput;
        bool mIsPaused;
        audio_output_flags_t mFlag;

        class BufferInfo {
        public:
            BufferInfo(void *buf1, void *buf2, int32_t nSize) :
            localBuf(buf1), dspBuf(buf2), memBufsize(nSize)
            {}

            void *localBuf;
            void *dspBuf;
            uint32_t memBufsize;
            uint32_t bytesToWrite;
        };
        List<BufferInfo> mBufPool;
        List<BufferInfo> mEffectsPool;
        void *mEffectsThreadScratchBuffer;

        void allocateBufPool();
        void deallocateBufPool();

        //******Effects*************
        static void *EffectsThreadWrapper(void *me);
        void EffectsThreadEntry();
        // make sure the Effects thread also exited
        void requestAndWaitForEffectsThreadExit();
        void createEffectThread();
        Condition mEffectCv;
        Mutex mEffectLock;
        pthread_t mEffectsThread;
        bool mKillEffectsThread;
        bool mEffectsThreadAlive;
        bool mEffectConfigChanged;

        //Structure to recieve the Effect notification from the flinger.
        class AudioFlingerDirectTrackClient: public IBinder::DeathRecipient, public BnAudioFlingerClient {
        public:
            AudioFlingerDirectTrackClient(void *obj);

            DirectAudioTrack *pBaseClass;
            // DeathRecipient
            virtual void binderDied(const wp<IBinder>& who);

            // IAudioFlingerClient

            // indicate a change in the configuration of an output or input: keeps the cached
            // values for output/input parameters upto date in client process
            virtual void ioConfigChanged(int event, audio_io_handle_t ioHandle, const void *param2);

            friend class DirectAudioTrack;
        };
        // helper function to obtain AudioFlinger service handle
        sp<AudioFlinger> mAudioFlinger;
        sp<AudioFlingerDirectTrackClient> mAudioFlingerClient;

        void clearPowerManager();

        class PMDeathRecipient : public IBinder::DeathRecipient {
            public:
                            PMDeathRecipient(void *obj){parentClass = (DirectAudioTrack *)obj;}
                virtual     ~PMDeathRecipient() {}

                // IBinder::DeathRecipient
                virtual     void        binderDied(const wp<IBinder>& who);

            private:
                            DirectAudioTrack *parentClass;
                            PMDeathRecipient(const PMDeathRecipient&);
                            PMDeathRecipient& operator = (const PMDeathRecipient&);

            friend class DirectAudioTrack;
        };

        friend class PMDeathRecipient;

        Mutex pmLock;
        void        acquireWakeLock();
        void        releaseWakeLock();

        sp<IPowerManager>       mPowerManager;
        sp<IBinder>             mWakeLockToken;
        sp<PMDeathRecipient>    mDeathRecipient;
    };
#endif

    class TrackHandle : public android::BnAudioTrack {
    public:
                            TrackHandle(const sp<PlaybackThread::Track>& track);
        virtual             ~TrackHandle();
        virtual sp<IMemory> getCblk() const;
        virtual status_t    start();
        virtual void        stop();
        virtual void        flush();
        virtual void        pause();
        virtual status_t    attachAuxEffect(int effectId);
        virtual status_t    allocateTimedBuffer(size_t size,
                                                sp<IMemory>* buffer);
        virtual status_t    queueTimedBuffer(const sp<IMemory>& buffer,
                                             int64_t pts);
        virtual status_t    setMediaTimeTransform(const LinearTransform& xform,
                                                  int target);
        virtual status_t    setParameters(const String8& keyValuePairs);
        virtual status_t    getTimestamp(AudioTimestamp& timestamp);
        virtual void        signal(); // signal playback thread for a change in control block

        virtual status_t onTransact(
            uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags);
<<<<<<< HEAD
    private:
        const sp<PlaybackThread::Track> mTrack;
    };

                void        removeClient_l(pid_t pid);
                void        removeNotificationClient(sp<IBinder> binder);


    // record thread
    class RecordThread : public ThreadBase, public AudioBufferProvider
                            // derives from AudioBufferProvider interface for use by resampler
    {
    public:

        // record track
        class RecordTrack : public TrackBase {
        public:
                                RecordTrack(RecordThread *thread,
                                        const sp<Client>& client,
                                        uint32_t sampleRate,
                                        audio_format_t format,
                                        audio_channel_mask_t channelMask,
                                        int frameCount,
#ifdef QCOM_ENHANCED_AUDIO
                                        uint32_t flags,
#endif
                                        int sessionId);
            virtual             ~RecordTrack();

            virtual status_t    start(AudioSystem::sync_event_t event, int triggerSession);
            virtual void        stop();

                    void        destroy();

                    // clear the buffer overflow flag
                    void        clearOverflow() { mOverflow = false; }
                    // set the buffer overflow flag and return previous value
                    bool        setOverflow() { bool tmp = mOverflow; mOverflow = true; return tmp; }

            static  void        appendDumpHeader(String8& result);
                    void        dump(char* buffer, size_t size);

        private:
            friend class AudioFlinger;  // for mState

                                RecordTrack(const RecordTrack&);
                                RecordTrack& operator = (const RecordTrack&);

            // AudioBufferProvider interface
            virtual status_t getNextBuffer(AudioBufferProvider::Buffer* buffer, int64_t pts = kInvalidPTS);
            // releaseBuffer() not overridden

            bool                mOverflow;  // overflow on most recent attempt to fill client buffer
        };

                RecordThread(const sp<AudioFlinger>& audioFlinger,
                        AudioStreamIn *input,
                        uint32_t sampleRate,
                        audio_channel_mask_t channelMask,
                        audio_io_handle_t id,
                        audio_devices_t device);
                virtual     ~RecordThread();

        // no addTrack_l ?
        void        destroyTrack_l(const sp<RecordTrack>& track);
        void        removeTrack_l(const sp<RecordTrack>& track);

        void        dumpInternals(int fd, const Vector<String16>& args);
        void        dumpTracks(int fd, const Vector<String16>& args);

        // Thread virtuals
        virtual bool        threadLoop();
        virtual status_t    readyToRun();

        // RefBase
        virtual void        onFirstRef();

        virtual status_t    initCheck() const { return (mInput == NULL) ? NO_INIT : NO_ERROR; }
                sp<AudioFlinger::RecordThread::RecordTrack>  createRecordTrack_l(
                        const sp<AudioFlinger::Client>& client,
                        uint32_t sampleRate,
                        audio_format_t format,
                        audio_channel_mask_t channelMask,
                        int frameCount,
                        int sessionId,
                        IAudioFlinger::track_flags_t flags,
                        pid_t tid,
                        status_t *status);

                status_t    start(RecordTrack* recordTrack,
                                  AudioSystem::sync_event_t event,
                                  int triggerSession);

                // ask the thread to stop the specified track, and
                // return true if the caller should then do it's part of the stopping process
                bool        stop_l(RecordTrack* recordTrack);

                void        dump(int fd, const Vector<String16>& args);
                AudioStreamIn* clearInput();
                virtual audio_stream_t* stream() const;

        // AudioBufferProvider interface
        virtual status_t    getNextBuffer(AudioBufferProvider::Buffer* buffer, int64_t pts);
        virtual void        releaseBuffer(AudioBufferProvider::Buffer* buffer);

        virtual bool        checkForNewParameters_l();
        virtual String8     getParameters(const String8& keys);
        virtual void        audioConfigChanged_l(int event, int param = 0);
                void        readInputParameters();
        virtual unsigned int  getInputFramesLost();

        virtual status_t addEffectChain_l(const sp<EffectChain>& chain);
        virtual size_t removeEffectChain_l(const sp<EffectChain>& chain);
        virtual uint32_t hasAudioSession(int sessionId) const;

                // Return the set of unique session IDs across all tracks.
                // The keys are the session IDs, and the associated values are meaningless.
                // FIXME replace by Set [and implement Bag/Multiset for other uses].
                KeyedVector<int, bool> sessionIds() const;

        virtual status_t setSyncEvent(const sp<SyncEvent>& event);
        virtual bool     isValidSyncEvent(const sp<SyncEvent>& event) const;

        static void syncStartEventCallback(const wp<SyncEvent>& event);
               void handleSyncStartEvent(const sp<SyncEvent>& event);

    private:
                void clearSyncStartEvent();

                // Enter standby if not already in standby, and set mStandby flag
                void standby();

                // Call the HAL standby method unconditionally, and don't change mStandby flag
                void inputStandBy();

                AudioStreamIn                       *mInput;
                SortedVector < sp<RecordTrack> >    mTracks;
                // mActiveTrack has dual roles:  it indicates the current active track, and
                // is used together with mStartStopCond to indicate start()/stop() progress
                sp<RecordTrack>                     mActiveTrack;
                Condition                           mStartStopCond;
                AudioResampler                      *mResampler;
                int32_t                             *mRsmpOutBuffer;
                int16_t                             *mRsmpInBuffer;
                size_t                              mRsmpInIndex;
                size_t                              mInputBytes;
                const int                           mReqChannelCount;
                const uint32_t                      mReqSampleRate;
                ssize_t                             mBytesRead;
                // sync event triggering actual audio capture. Frames read before this event will
                // be dropped and therefore not read by the application.
                sp<SyncEvent>                       mSyncStartEvent;
                // number of captured frames to drop after the start sync event has been received.
                // when < 0, maximum frames to drop before starting capture even if sync event is
                // not received
                ssize_t                             mFramestoDrop;
                int16_t                             mInputSource;
=======

    private:
        const sp<PlaybackThread::Track> mTrack;
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    };

    // server side of the client's IAudioRecord
    class RecordHandle : public android::BnAudioRecord {
    public:
        RecordHandle(const sp<RecordThread::RecordTrack>& recordTrack);
        virtual             ~RecordHandle();
        virtual status_t    start(int /*AudioSystem::sync_event_t*/ event, int triggerSession);
        virtual void        stop();
        virtual status_t onTransact(
            uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags);
    private:
        const sp<RecordThread::RecordTrack> mRecordTrack;

        // for use from destructor
        void                stop_nonvirtual();
    };

<<<<<<< HEAD
    //--- Audio Effect Management

    // EffectModule and EffectChain classes both have their own mutex to protect
    // state changes or resource modifications. Always respect the following order
    // if multiple mutexes must be acquired to avoid cross deadlock:
    // AudioFlinger -> ThreadBase -> EffectChain -> EffectModule

    // The EffectModule class is a wrapper object controlling the effect engine implementation
    // in the effect library. It prevents concurrent calls to process() and command() functions
    // from different client threads. It keeps a list of EffectHandle objects corresponding
    // to all client applications using this effect and notifies applications of effect state,
    // control or parameter changes. It manages the activation state machine to send appropriate
    // reset, enable, disable commands to effect engine and provide volume
    // ramping when effects are activated/deactivated.
    // When controlling an auxiliary effect, the EffectModule also provides an input buffer used by
    // the attached track(s) to accumulate their auxiliary channel.
    class EffectModule: public RefBase {
    public:
        EffectModule(ThreadBase *thread,
                        const wp<AudioFlinger::EffectChain>& chain,
                        effect_descriptor_t *desc,
                        int id,
                        int sessionId);
        virtual ~EffectModule();

        enum effect_state {
            IDLE,
            RESTART,
            STARTING,
            ACTIVE,
            STOPPING,
            STOPPED,
            DESTROYED
        };

        int         id() const { return mId; }
        void process();
        void updateState();
        status_t command(uint32_t cmdCode,
                         uint32_t cmdSize,
                         void *pCmdData,
                         uint32_t *replySize,
                         void *pReplyData);

        void reset_l();
#ifdef QCOM_HARDWARE
        status_t configure(bool isForLPA = false,
                           int sampleRate = 0,
                           int channelCount = 0,
                           int frameCount = 0);
#else
        status_t configure();
#endif
        status_t init();
        effect_state state() const {
            return mState;
        }
        uint32_t status() {
            return mStatus;
        }
        int sessionId() const {
            return mSessionId;
        }
        status_t    setEnabled(bool enabled);
        status_t    setEnabled_l(bool enabled);
        bool isEnabled() const;
        bool isProcessEnabled() const;

        void        setInBuffer(int16_t *buffer) { mConfig.inputCfg.buffer.s16 = buffer; }
        int16_t     *inBuffer() { return mConfig.inputCfg.buffer.s16; }
        void        setOutBuffer(int16_t *buffer) { mConfig.outputCfg.buffer.s16 = buffer; }
        int16_t     *outBuffer() { return mConfig.outputCfg.buffer.s16; }
        void        setChain(const wp<EffectChain>& chain) { mChain = chain; }
        void        setThread(const wp<ThreadBase>& thread) { mThread = thread; }
        const wp<ThreadBase>& thread() { return mThread; }

        status_t addHandle(EffectHandle *handle);
        size_t disconnect(EffectHandle *handle, bool unpinIfLast);
        size_t removeHandle(EffectHandle *handle);

        const effect_descriptor_t& desc() const { return mDescriptor; }
        wp<EffectChain>&     chain() { return mChain; }

        status_t         setDevice(audio_devices_t device);
        status_t         setVolume(uint32_t *left, uint32_t *right, bool controller);
        status_t         setMode(audio_mode_t mode);
        status_t         setAudioSource(audio_source_t source);
        status_t         start();
        status_t         stop();
        void             setSuspended(bool suspended);
        bool             suspended() const;

        EffectHandle*    controlHandle_l();

        bool             isPinned() const { return mPinned; }
        void             unPin() { mPinned = false; }
        bool             purgeHandles();
        void             lock() { mLock.lock(); }
        void             unlock() { mLock.unlock(); }
#ifdef QCOM_HARDWARE
        bool             isOnLPA() { return mIsForLPA;}
        void             setLPAFlag(bool isForLPA) {mIsForLPA = isForLPA; }
#endif
        void             dump(int fd, const Vector<String16>& args);

    protected:
        friend class AudioFlinger;      // for mHandles
        bool                mPinned;

        // Maximum time allocated to effect engines to complete the turn off sequence
        static const uint32_t MAX_DISABLE_TIME_MS = 10000;

        EffectModule(const EffectModule&);
        EffectModule& operator = (const EffectModule&);

        status_t start_l();
        status_t stop_l();

mutable Mutex               mLock;      // mutex for process, commands and handles list protection
        wp<ThreadBase>      mThread;    // parent thread
        wp<EffectChain>     mChain;     // parent effect chain
        const int           mId;        // this instance unique ID
        const int           mSessionId; // audio session ID
        const effect_descriptor_t mDescriptor;// effect descriptor received from effect engine
        effect_config_t     mConfig;    // input and output audio configuration
        effect_handle_t  mEffectInterface; // Effect module C API
        status_t            mStatus;    // initialization status
        effect_state        mState;     // current activation state
        Vector<EffectHandle *> mHandles;    // list of client handles
                    // First handle in mHandles has highest priority and controls the effect module
        uint32_t mMaxDisableWaitCnt;    // maximum grace period before forcing an effect off after
                                        // sending disable command.
        uint32_t mDisableWaitCnt;       // current process() calls count during disable period.
        bool     mSuspended;            // effect is suspended: temporarily disabled by framework
#ifdef QCOM_HARDWARE
        bool     mIsForLPA;
#endif
    };

    // The EffectHandle class implements the IEffect interface. It provides resources
    // to receive parameter updates, keeps track of effect control
    // ownership and state and has a pointer to the EffectModule object it is controlling.
    // There is one EffectHandle object for each application controlling (or using)
    // an effect module.
    // The EffectHandle is obtained by calling AudioFlinger::createEffect().
    class EffectHandle: public android::BnEffect {
=======
#ifdef QCOM_DIRECTTRACK
    // server side of the client's IAudioTrack
    class DirectAudioTrack : public android::BnDirectTrack,
                             public AudioEventObserver
    {
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    public:
                            DirectAudioTrack(const sp<AudioFlinger>& audioFlinger,
                                             int output, AudioSessionDescriptor *outputDesc,
                                             IDirectTrackClient* client, audio_output_flags_t outflag);
        virtual             ~DirectAudioTrack();
        virtual status_t    start();
        virtual void        stop();
        virtual void        flush();
        virtual void        mute(bool);
        virtual void        pause();
        virtual ssize_t     write(const void *buffer, size_t bytes);
        virtual void        setVolume(float left, float right);
        virtual int64_t     getTimeStamp();
        virtual void        postEOS(int64_t delayUs);
        void                signalEffect();

        virtual status_t    onTransact(
            uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags);
    private:

        IDirectTrackClient* mClient;
        AudioSessionDescriptor *mOutputDesc;
        int  mOutput;
        bool mIsPaused;
        audio_output_flags_t mFlag;

        class BufferInfo {
        public:
            BufferInfo(void *buf1, void *buf2, int32_t nSize) :
            localBuf(buf1), dspBuf(buf2), memBufsize(nSize)
            {}

            void *localBuf;
            void *dspBuf;
            uint32_t memBufsize;
            uint32_t bytesToWrite;
        };
        List<BufferInfo> mBufPool;
        List<BufferInfo> mEffectsPool;
        void *mEffectsThreadScratchBuffer;

        void allocateBufPool();
        void deallocateBufPool();

        //******Effects*************
        static void *EffectsThreadWrapper(void *me);
        void EffectsThreadEntry();
        // make sure the Effects thread also exited
        void requestAndWaitForEffectsThreadExit();
        void createEffectThread();
        Condition mEffectCv;
        Mutex mEffectLock;
        pthread_t mEffectsThread;
        bool mKillEffectsThread;
        bool mEffectsThreadAlive;
        bool mEffectConfigChanged;

        //Structure to recieve the Effect notification from the flinger.
        class AudioFlingerDirectTrackClient: public IBinder::DeathRecipient, public BnAudioFlingerClient {
        public:
            AudioFlingerDirectTrackClient(void *obj);

            DirectAudioTrack *pBaseClass;
            // DeathRecipient
            virtual void binderDied(const wp<IBinder>& who);

            // IAudioFlingerClient

            // indicate a change in the configuration of an output or input: keeps the cached
            // values for output/input parameters upto date in client process
            virtual void ioConfigChanged(int event, audio_io_handle_t ioHandle, const void *param2);

<<<<<<< HEAD
        status_t addEffect_l(const sp<EffectModule>& handle);
        size_t removeEffect_l(const sp<EffectModule>& handle);
#ifdef QCOM_HARDWARE
        size_t getNumEffects() { return mEffects.size(); }
#endif
=======
            friend class DirectAudioTrack;
        };
        // helper function to obtain AudioFlinger service handle
        sp<AudioFlinger> mAudioFlinger;
        sp<AudioFlingerDirectTrackClient> mAudioFlingerClient;
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e

        void clearPowerManager();
        class PMDeathRecipient : public IBinder::DeathRecipient {
            public:
                            PMDeathRecipient(void *obj){parentClass = (DirectAudioTrack *)obj;}
                virtual     ~PMDeathRecipient() {}

<<<<<<< HEAD
        sp<EffectModule> getEffectFromDesc_l(effect_descriptor_t *descriptor);
        sp<EffectModule> getEffectFromId_l(int id);
#ifdef QCOM_HARDWARE
        sp<EffectModule> getEffectFromIndex_l(int idx);
#endif
        sp<EffectModule> getEffectFromType_l(const effect_uuid_t *type);
        bool setVolume_l(uint32_t *left, uint32_t *right);
        void setDevice_l(audio_devices_t device);
        void setMode_l(audio_mode_t mode);
        void setAudioSource_l(audio_source_t source);
=======
                // IBinder::DeathRecipient
                virtual     void        binderDied(const wp<IBinder>& who);
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e

            private:
                            DirectAudioTrack *parentClass;
                            PMDeathRecipient(const PMDeathRecipient&);
                            PMDeathRecipient& operator = (const PMDeathRecipient&);

            friend class DirectAudioTrack;
        };

        friend class PMDeathRecipient;

        Mutex pmLock;
        void        acquireWakeLock();
        void        releaseWakeLock();

        sp<IPowerManager>       mPowerManager;
        sp<IBinder>             mWakeLockToken;
        sp<PMDeathRecipient>    mDeathRecipient;
    };
#endif


              PlaybackThread *checkPlaybackThread_l(audio_io_handle_t output) const;
              MixerThread *checkMixerThread_l(audio_io_handle_t output) const;
              RecordThread *checkRecordThread_l(audio_io_handle_t input) const;
              sp<RecordThread> openInput_l(audio_module_handle_t module,
                                           audio_io_handle_t *input,
                                           audio_config_t *config,
                                           audio_devices_t device,
                                           const String8& address,
                                           audio_source_t source,
                                           audio_input_flags_t flags);
              sp<PlaybackThread> openOutput_l(audio_module_handle_t module,
                                              audio_io_handle_t *output,
                                              audio_config_t *config,
                                              audio_devices_t devices,
                                              const String8& address,
                                              audio_output_flags_t flags);

              void closeOutputFinish(sp<PlaybackThread> thread);
              void closeInputFinish(sp<RecordThread> thread);

<<<<<<< HEAD
        void dump(int fd, const Vector<String16>& args);
#ifdef QCOM_HARDWARE
        bool isForLPATrack() {return mIsForLPATrack; }
        void setLPAFlag(bool flag) {mIsForLPATrack = flag;}
#endif
=======
              // no range check, AudioFlinger::mLock held
              bool streamMute_l(audio_stream_type_t stream) const
                                { return mStreamTypes[stream].mute; }
              // no range check, doesn't check per-thread stream volume, AudioFlinger::mLock held
              float streamVolume_l(audio_stream_type_t stream) const
                                { return mStreamTypes[stream].volume; }
              void audioConfigChanged(int event, audio_io_handle_t ioHandle, const void *param2);

              // Allocate an audio_io_handle_t, session ID, effect ID, or audio_module_handle_t.
              // They all share the same ID space, but the namespaces are actually independent
              // because there are separate KeyedVectors for each kind of ID.
              // The return value is uint32_t, but is cast to signed for some IDs.
              // FIXME This API does not handle rollover to zero (for unsigned IDs),
              //       or from positive to negative (for signed IDs).
              //       Thus it may fail by returning an ID of the wrong sign,
              //       or by returning a non-unique ID.
              uint32_t nextUniqueId();
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e

              status_t moveEffectChain_l(int sessionId,
                                     PlaybackThread *srcThread,
                                     PlaybackThread *dstThread,
                                     bool reRegister);
              // return thread associated with primary hardware device, or NULL
              PlaybackThread *primaryPlaybackThread_l() const;
              audio_devices_t primaryOutputDevice_l() const;

              sp<PlaybackThread> getEffectThread_l(int sessionId, int EffectId);


<<<<<<< HEAD
        // get a list of effect modules to suspend when an effect of the type
        // passed is enabled.
        void                       getSuspendEligibleEffects(Vector< sp<EffectModule> > &effects);

        // get an effect module if it is currently enable
        sp<EffectModule> getEffectIfEnabled(const effect_uuid_t *type);
        // true if the effect whose descriptor is passed can be suspended
        // OEMs can modify the rules implemented in this method to exclude specific effect
        // types or implementations from the suspend/restore mechanism.
        bool isEffectEligibleForSuspend(const effect_descriptor_t& desc);

        void clearInputBuffer_l(sp<ThreadBase> thread);

        wp<ThreadBase> mThread;     // parent mixer thread
        Mutex mLock;                // mutex protecting effect list
        Vector< sp<EffectModule> > mEffects; // list of effect modules
        int mSessionId;             // audio session ID
        int16_t *mInBuffer;         // chain input buffer
        int16_t *mOutBuffer;        // chain output buffer

        // 'volatile' here means these are accessed with atomic operations instead of mutex
        volatile int32_t mActiveTrackCnt;    // number of active tracks connected
        volatile int32_t mTrackCnt;          // number of tracks connected

        int32_t mTailBufferCount;   // current effect tail buffer count
        int32_t mMaxTailBuffers;    // maximum effect tail buffers
        bool mOwnInBuffer;          // true if the chain owns its input buffer
        int mVolumeCtrlIdx;         // index of insert effect having control over volume
        uint32_t mLeftVolume;       // previous volume on left channel
        uint32_t mRightVolume;      // previous volume on right channel
        uint32_t mNewLeftVolume;       // new volume on left channel
        uint32_t mNewRightVolume;      // new volume on right channel
        uint32_t mStrategy; // strategy for this effect chain
#ifdef QCOM_HARDWARE
        bool     mIsForLPATrack;
#endif
        // mSuspendedEffects lists all effects currently suspended in the chain.
        // Use effect type UUID timelow field as key. There is no real risk of identical
        // timeLow fields among effect type UUIDs.
        // Updated by updateSuspendedSessions_l() only.
        KeyedVector< int, sp<SuspendedEffectDesc> > mSuspendedEffects;
    };
=======
                void        removeClient_l(pid_t pid);
                void        removeNotificationClient(pid_t pid);
                bool isNonOffloadableGlobalEffectEnabled_l();
                void onNonOffloadableGlobalEffectEnable();

                // Store an effect chain to mOrphanEffectChains keyed vector.
                // Called when a thread exits and effects are still attached to it.
                // If effects are later created on the same session, they will reuse the same
                // effect chain and same instances in the effect library.
                // return ALREADY_EXISTS if a chain with the same session already exists in
                // mOrphanEffectChains. Note that this should never happen as there is only one
                // chain for a given session and it is attached to only one thread at a time.
                status_t        putOrphanEffectChain_l(const sp<EffectChain>& chain);
                // Get an effect chain for the specified session in mOrphanEffectChains and remove
                // it if found. Returns 0 if not found (this is the most common case).
                sp<EffectChain> getOrphanEffectChain_l(audio_session_t session);
                // Called when the last effect handle on an effect instance is removed. If this
                // effect belongs to an effect chain in mOrphanEffectChains, the chain is updated
                // and removed from mOrphanEffectChains if it does not contain any effect.
                // Return true if the effect was found in mOrphanEffectChains, false otherwise.
                bool            updateOrphanEffectChains(const sp<EffectModule>& effect);
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e

    class AudioHwDevice {
    public:
        enum Flags {
            AHWD_CAN_SET_MASTER_VOLUME  = 0x1,
            AHWD_CAN_SET_MASTER_MUTE    = 0x2,
        };

        AudioHwDevice(audio_module_handle_t handle,
                      const char *moduleName,
                      audio_hw_device_t *hwDevice,
                      Flags flags)
            : mHandle(handle), mModuleName(strdup(moduleName))
            , mHwDevice(hwDevice)
            , mFlags(flags) { }
        /*virtual*/ ~AudioHwDevice() { free((void *)mModuleName); }

        bool canSetMasterVolume() const {
            return (0 != (mFlags & AHWD_CAN_SET_MASTER_VOLUME));
        }

        bool canSetMasterMute() const {
            return (0 != (mFlags & AHWD_CAN_SET_MASTER_MUTE));
        }

        audio_module_handle_t handle() const { return mHandle; }
        const char *moduleName() const { return mModuleName; }
        audio_hw_device_t *hwDevice() const { return mHwDevice; }
        uint32_t version() const { return mHwDevice->common.version; }

    private:
        const audio_module_handle_t mHandle;
        const char * const mModuleName;
        audio_hw_device_t * const mHwDevice;
        const Flags mFlags;
    };

    // AudioStreamOut and AudioStreamIn are immutable, so their fields are const.
    // For emphasis, we could also make all pointers to them be "const *",
    // but that would clutter the code unnecessarily.

    struct AudioStreamOut {
        AudioHwDevice* const audioHwDev;
        audio_stream_out_t* const stream;
        const audio_output_flags_t flags;

        audio_hw_device_t* hwDev() const { return audioHwDev->hwDevice(); }

        AudioStreamOut(AudioHwDevice *dev, audio_stream_out_t *out, audio_output_flags_t flags) :
            audioHwDev(dev), stream(out), flags(flags) {}
    };

    struct AudioStreamIn {
        AudioHwDevice* const audioHwDev;
        audio_stream_in_t* const stream;

        audio_hw_device_t* hwDev() const { return audioHwDev->hwDevice(); }

        AudioStreamIn(AudioHwDevice *dev, audio_stream_in_t *in) :
            audioHwDev(dev), stream(in) {}
    };
<<<<<<< HEAD
#ifdef QCOM_HARDWARE
=======

#ifdef QCOM_DIRECTTRACK
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    struct AudioSessionDescriptor {
        bool    mActive;
        int     mStreamType;
        float   mVolumeLeft;
        float   mVolumeRight;
        float   mVolumeScale;
        audio_hw_device_t   *hwDev;
        audio_stream_out_t  *stream;
        audio_output_flags_t flag;
        void *trackRefPtr;
        audio_devices_t device;
        AudioSessionDescriptor(audio_hw_device_t *dev, audio_stream_out_t *out, audio_output_flags_t outflag) :
            hwDev(dev), stream(out), flag(outflag)  {}
    };
#endif
<<<<<<< HEAD
=======

>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    // for mAudioSessionRefs only
    struct AudioSessionRef {
        AudioSessionRef(int sessionid, pid_t pid) :
            mSessionid(sessionid), mPid(pid), mCnt(1) {}
        const int   mSessionid;
        const pid_t mPid;
        int         mCnt;
    };

    mutable     Mutex                               mLock;
                // protects mClients and mNotificationClients.
                // must be locked after mLock and ThreadBase::mLock if both must be locked
                // avoids acquiring AudioFlinger::mLock from inside thread loop.
    mutable     Mutex                               mClientLock;
                // protected by mClientLock
                DefaultKeyedVector< pid_t, wp<Client> >     mClients;   // see ~Client()

                mutable     Mutex                   mHardwareLock;
                // NOTE: If both mLock and mHardwareLock mutexes must be held,
                // always take mLock before mHardwareLock

                // These two fields are immutable after onFirstRef(), so no lock needed to access
                AudioHwDevice*                      mPrimaryHardwareDev; // mAudioHwDevs[0] or NULL
                DefaultKeyedVector<audio_module_handle_t, AudioHwDevice*>  mAudioHwDevs;

    // for dump, indicates which hardware operation is currently in progress (but not stream ops)
    enum hardware_call_state {
        AUDIO_HW_IDLE = 0,              // no operation in progress
        AUDIO_HW_INIT,                  // init_check
        AUDIO_HW_OUTPUT_OPEN,           // open_output_stream
        AUDIO_HW_OUTPUT_CLOSE,          // unused
        AUDIO_HW_INPUT_OPEN,            // unused
        AUDIO_HW_INPUT_CLOSE,           // unused
        AUDIO_HW_STANDBY,               // unused
        AUDIO_HW_SET_MASTER_VOLUME,     // set_master_volume
        AUDIO_HW_GET_ROUTING,           // unused
        AUDIO_HW_SET_ROUTING,           // unused
        AUDIO_HW_GET_MODE,              // unused
        AUDIO_HW_SET_MODE,              // set_mode
        AUDIO_HW_GET_MIC_MUTE,          // get_mic_mute
        AUDIO_HW_SET_MIC_MUTE,          // set_mic_mute
        AUDIO_HW_SET_VOICE_VOLUME,      // set_voice_volume
        AUDIO_HW_SET_PARAMETER,         // set_parameters
#ifdef QCOM_FM_ENABLED
        AUDIO_SET_FM_VOLUME,
#endif
        AUDIO_HW_GET_INPUT_BUFFER_SIZE, // get_input_buffer_size
        AUDIO_HW_GET_MASTER_VOLUME,     // get_master_volume
        AUDIO_HW_GET_PARAMETER,         // get_parameters
        AUDIO_HW_SET_MASTER_MUTE,       // set_master_mute
        AUDIO_HW_GET_MASTER_MUTE,       // get_master_mute
    };

    mutable     hardware_call_state                 mHardwareStatus;    // for dump only


                DefaultKeyedVector< audio_io_handle_t, sp<PlaybackThread> >  mPlaybackThreads;
                stream_type_t                       mStreamTypes[AUDIO_STREAM_CNT];

                // member variables below are protected by mLock
                float                               mMasterVolume;
                bool                                mMasterMute;
                // end of variables protected by mLock

                DefaultKeyedVector< audio_io_handle_t, sp<RecordThread> >    mRecordThreads;

<<<<<<< HEAD
                DefaultKeyedVector< sp<IBinder>, sp<NotificationClient> >    mNotificationClients;
=======
                // protected by mClientLock
                DefaultKeyedVector< pid_t, sp<NotificationClient> >    mNotificationClients;

>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
                volatile int32_t                    mNextUniqueId;  // updated by android_atomic_inc
                // nextUniqueId() returns uint32_t, but this is declared int32_t
                // because the atomic operations require an int32_t

                audio_mode_t                        mMode;
                bool                                mBtNrecIsOff;
<<<<<<< HEAD
#ifdef QCOM_HARDWARE
                DefaultKeyedVector<audio_io_handle_t, AudioSessionDescriptor *> mDirectAudioTracks;
                int                                 mA2DPHandle; // Handle to notify A2DP connection status
#endif
                // protected by mLock
#ifdef QCOM_HARDWARE
                volatile bool                       mIsEffectConfigChanged;
#endif
                Vector<AudioSessionRef*> mAudioSessionRefs;
#ifdef QCOM_HARDWARE
                sp<EffectChain> mLPAEffectChain;
                int         mLPASessionId;
=======

#ifdef QCOM_DIRECTTRACK
                DefaultKeyedVector<audio_io_handle_t, AudioSessionDescriptor *> mDirectAudioTracks;

                // protected by mLock
                volatile bool                       mIsEffectConfigChanged;
#endif
                Vector<AudioSessionRef*> mAudioSessionRefs;
#ifdef QCOM_DIRECTTRACK
                sp<EffectChain> mLPAEffectChain;
                int         mLPASessionId;
                audio_devices_t mDirectDevice;//device for directTrack,used for effects
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
                int                                 mLPASampleRate;
                int                                 mLPANumChannels;
                volatile bool                       mAllChainsLocked;
#endif
<<<<<<< HEAD
=======

>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
                float       masterVolume_l() const;
                bool        masterMute_l() const;
                audio_module_handle_t loadHwModule_l(const char *name);

                Vector < sp<SyncEvent> > mPendingSyncEvents; // sync events awaiting for a session
                                                             // to be created

                // Effect chains without a valid thread
                DefaultKeyedVector< audio_session_t , sp<EffectChain> > mOrphanEffectChains;

private:
    sp<Client>  registerPid(pid_t pid);    // always returns non-0

    // for use from destructor
    status_t    closeOutput_nonvirtual(audio_io_handle_t output);
    void        closeOutputInternal_l(sp<PlaybackThread> thread);
    status_t    closeInput_nonvirtual(audio_io_handle_t input);
    void        closeInputInternal_l(sp<RecordThread> thread);

#ifdef TEE_SINK
    // all record threads serially share a common tee sink, which is re-created on format change
    sp<NBAIO_Sink>   mRecordTeeSink;
    sp<NBAIO_Source> mRecordTeeSource;
#endif

public:

#ifdef TEE_SINK
    // tee sink, if enabled by property, allows dumpsys to write most recent audio to .wav file
    static void dumpTee(int fd, const sp<NBAIO_Source>& source, audio_io_handle_t id = 0);

    // whether tee sink is enabled by property
    static bool mTeeSinkInputEnabled;
    static bool mTeeSinkOutputEnabled;
    static bool mTeeSinkTrackEnabled;

    // runtime configured size of each tee sink pipe, in frames
    static size_t mTeeSinkInputFrames;
    static size_t mTeeSinkOutputFrames;
    static size_t mTeeSinkTrackFrames;

    // compile-time default size of tee sink pipes, in frames
    // 0x200000 stereo 16-bit PCM frames = 47.5 seconds at 44.1 kHz, 8 megabytes
    static const size_t kTeeSinkInputFramesDefault = 0x200000;
    static const size_t kTeeSinkOutputFramesDefault = 0x200000;
    static const size_t kTeeSinkTrackFramesDefault = 0x200000;
#endif

    // This method reads from a variable without mLock, but the variable is updated under mLock.  So
    // we might read a stale value, or a value that's inconsistent with respect to other variables.
    // In this case, it's safe because the return value isn't used for making an important decision.
    // The reason we don't want to take mLock is because it could block the caller for a long time.
    bool    isLowRamDevice() const { return mIsLowRamDevice; }

private:
    bool    mIsLowRamDevice;
    bool    mIsDeviceTypeKnown;
    nsecs_t mGlobalEffectEnableTime;  // when a global effect was last enabled

    sp<PatchPanel> mPatchPanel;

    uint32_t    mPrimaryOutputSampleRate;   // sample rate of the primary output, or zero if none
                                            // protected by mHardwareLock
};

#undef INCLUDING_FROM_AUDIOFLINGER_H

const char *formatToString(audio_format_t format);

// ----------------------------------------------------------------------------

}; // namespace android

#endif // ANDROID_AUDIO_FLINGER_H

