/*
<<<<<<< HEAD
 * Copyright (C) 2010 The Android Open Source Project
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * Not a Contribution, Apache license notifications and license are retained
 * for attribution purposes only
=======
 * Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 * Copyright (C) 2010 The Android Open Source Project
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
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


//#define LOG_NDEBUG 0
#define LOG_TAG "FMA2DPWriter"
#include <utils/Log.h>

<<<<<<< HEAD

#include <media/stagefright/FMA2DPWriter.h>
#include <media/stagefright/MediaBuffer.h>
#include <media/stagefright/MediaDebug.h>
=======
#include <media/stagefright/foundation/ADebug.h>
#include <media/stagefright/FMA2DPWriter.h>
#include <media/stagefright/MediaBuffer.h>
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
#include <media/stagefright/MediaDefs.h>
#include <media/stagefright/MediaErrors.h>
#include <media/stagefright/MediaSource.h>
#include <media/stagefright/MetaData.h>
#include <media/mediarecorder.h>
#include <sys/prctl.h>
#include <sys/resource.h>

#include <media/AudioRecord.h>
#include <media/AudioTrack.h>
<<<<<<< HEAD
namespace android {

#define BUFFER_POOL_SIZE 5
static int kMaxBufferSize = 2048;
=======

namespace android {

#define BUFFER_POOL_SIZE 5
#define MAX_BUFFER_SIZE 2048
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e

FMA2DPWriter::FMA2DPWriter()
    :mStarted(false),
    mAudioChannels(0),
    mSampleRate(0),
    mAudioFormat(AUDIO_FORMAT_PCM_16_BIT),
    mAudioSource(AUDIO_SOURCE_FM_RX_A2DP),
<<<<<<< HEAD
    mBufferSize(0){
    sem_init(&mReaderThreadWakeupsem,0,0);
    sem_init(&mWriterThreadWakeupsem,0,0);
}



=======
    mBufferSize(0) {
}

>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
FMA2DPWriter::~FMA2DPWriter() {
    if (mStarted) {
        stop();
    }
<<<<<<< HEAD
    sem_destroy(&mReaderThreadWakeupsem);
    sem_destroy(&mWriterThreadWakeupsem);
}

status_t FMA2DPWriter::initCheck() const {
// API not need for FMA2DPWriter
    return OK;
}


status_t FMA2DPWriter::addSource(const sp<MediaSource> &source) {
// API not need for FMA2DPWriter
    return OK;
}

status_t FMA2DPWriter::allocateBufferPool()
{
    Mutex::Autolock lock(mFreeQLock);

    for (int i = 0; i < BUFFER_POOL_SIZE; ++i) {
        int *buffer = (int*)malloc(mBufferSize);
        if(buffer){
            audioBufferstruct audioBuffer(buffer,mBufferSize);
            mFreeQ.push_back(audioBuffer);
        }
        else{
            ALOGE("fatal:failed to alloate buffer pool");
=======
}

status_t FMA2DPWriter::initCheck() const {
    // API not need for FMA2DPWriter
    return OK;
}

status_t FMA2DPWriter::addSource(const sp<MediaSource> &source) {
   // API not need for FMA2DPWriter
   return OK;
}

status_t FMA2DPWriter::allocateBufferPool() {
    Mutex::Autolock lock(mLock);

    for (int i = 0; i < BUFFER_POOL_SIZE; ++i) {
        int *buffer = new int[mBufferSize];
        if (buffer) {
            FMData *fmBuffer = new FMData(buffer, mBufferSize);
            mFMDataPool.add(fmBuffer);
            mFreeList.push_back(i);
        } else {
            ALOGE("%s, fatal: failed to alloate buffer pool. Deleting partially created mFMDataPool", __func__);
            for ( Vector<FMData *>::iterator it = mFMDataPool.begin();
                  it != mFMDataPool.end();) {
                int *tempBuffer = (int *)((*it)->audioBuffer);
                it = mFMDataPool.erase(it);
                delete tempBuffer;
            }
            mFreeList.clear();
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
            return  NO_INIT;
        }
    }
    return OK;
}

status_t FMA2DPWriter::start(MetaData *params) {
<<<<<<< HEAD

=======
    ALOGV("%s Entered", __func__);
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    if (mStarted) {
        // Already started, does nothing
        return OK;
    }

<<<<<<< HEAD
    if(!mStarted){
        if(!params){
            ALOGE("fatal:params cannot be null");
            return NO_INIT;
        }
        CHECK( params->findInt32( kKeyChannelCount, &mAudioChannels ) );
        CHECK(mAudioChannels  == 1 || mAudioChannels  == 2);
        CHECK( params->findInt32( kKeySampleRate, &mSampleRate ) );

        if ( NO_ERROR != AudioSystem::getInputBufferSize(
                    mSampleRate, mAudioFormat, mAudioChannels, &mBufferSize) ){
            mBufferSize = kMaxBufferSize ;
        }
        ALOGV("mBufferSize = %d", mBufferSize);
=======
    if(!mStarted) {
        if(params) {
            params->findInt32(kKeyChannelCount, &mAudioChannels);
            params->findInt32(kKeySampleRate, &mSampleRate);
        }
        if (0 == mAudioChannels) {
             ALOGD("%s set default channel count:%", __func__, mAudioChannels);
             mAudioChannels = AUDIO_CHANNELS;
        }
        if (0 == mSampleRate) {
             ALOGD("%s set default sample rate:%", __func__, SAMPLING_RATE);
             mSampleRate = SAMPLING_RATE;
        }

        if ( NO_ERROR != AudioSystem::getInputBufferSize(
                    mSampleRate, mAudioFormat, mAudioChannels, &mBufferSize) ){
            mBufferSize = MAX_BUFFER_SIZE;
        }
        ALOGV("%s mBufferSize = %d", __func__, mBufferSize);
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    }

    status_t err = allocateBufferPool();

    if(err != OK)
        return err;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    mDone = false;

    pthread_create(&mReaderThread, &attr, ReaderThreadWrapper, this);
    pthread_create(&mWriterThread, &attr, WriterThreadWrapper, this);

    pthread_attr_destroy(&attr);

<<<<<<< HEAD

    mStarted = true;

=======
    mStarted = true;
    ALOGV("%s Exit", __func__);
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    return OK;
}

status_t FMA2DPWriter::pause() {
<<<<<<< HEAD
// API not need for FMA2DPWriter
=======
    // API not need for FMA2DPWriter
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    return OK;
}

status_t FMA2DPWriter::stop() {
<<<<<<< HEAD
=======
    ALOGV("%s Enter", __func__);
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    if (!mStarted) {
        return OK;
    }

<<<<<<< HEAD
    mDone = true;

    void *dummy;
    pthread_join(mReaderThread, &dummy);
    pthread_join(mWriterThread, &dummy);

    for ( List<audioBufferstruct>::iterator it = mDataQ.begin();
         it != mDataQ.end(); ++it){
            free(it->audioBuffer);
    }
    for ( List<audioBufferstruct>::iterator it = mFreeQ.begin();
         it != mFreeQ.end(); ++it){
            free(it->audioBuffer);
    }
    mStarted = false;

=======
    {
        Mutex::Autolock _l(mLock);
        ALOGV("Exiting");
        mDone = true;
        mCondVar.signal();
    }

    pthread_join(mReaderThread, NULL);
    pthread_join(mWriterThread, NULL);
    for (Vector<FMData *>::iterator it = mFMDataPool.begin();
                                            it != mFMDataPool.end();) {
        int *tempBuffer = (int *)((*it)->audioBuffer);
        it = mFMDataPool.erase(it);
        delete tempBuffer;
    }
    mFreeList.clear();
    mDataList.clear();
    mStarted = false;
    ALOGV("%s Exit", __func__);
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    return OK;
}

void *FMA2DPWriter::ReaderThreadWrapper(void *me) {
<<<<<<< HEAD
    return (void *) static_cast<FMA2DPWriter *>(me)->readerthread();
}

void *FMA2DPWriter::WriterThreadWrapper(void *me) {
    return (void *) static_cast<FMA2DPWriter *>(me)->writerthread();
=======
    return (void *) (uintptr_t)static_cast<FMA2DPWriter *>(me)->readerthread();
}

void *FMA2DPWriter::WriterThreadWrapper(void *me) {
    return (void *) (uintptr_t)static_cast<FMA2DPWriter *>(me)->writerthread();
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
}

status_t FMA2DPWriter::readerthread() {
    status_t err = OK;
<<<<<<< HEAD
    int framecount =((4*mBufferSize)/mAudioChannels)/sizeof(int16_t);
=======
    int framecount = ((4*mBufferSize)/mAudioChannels)/sizeof(int16_t);
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    //sizeof(int16_t) is frame size for PCM stream
    int inChannel =
        (mAudioChannels == 2) ? AUDIO_CHANNEL_IN_STEREO :
        AUDIO_CHANNEL_IN_MONO;

    prctl(PR_SET_NAME, (unsigned long)"FMA2DPReaderThread", 0, 0, 0);

<<<<<<< HEAD
    AudioRecord* record = new AudioRecord(
=======
    sp<AudioRecord> record;
    record = new AudioRecord(
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
                     mAudioSource,
                     mSampleRate,
                     mAudioFormat,
                     inChannel,
<<<<<<< HEAD
                     framecount);
    if(!record){
=======
                     framecount,
                     0);

    if(NULL == record.get()){
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
        ALOGE("fatal:Not able to open audiorecord");
        return UNKNOWN_ERROR;
    }

    status_t res = record->initCheck();
    if (res == NO_ERROR)
        res = record->start();
    else{
        ALOGE("fatal:record init check failure");
        return UNKNOWN_ERROR;
    }

<<<<<<< HEAD

    while (!mDone) {

        mFreeQLock.lock();
        if(mFreeQ.empty()){
            mFreeQLock.unlock();
            ALOGV("FreeQ empty");
            sem_wait(&mReaderThreadWakeupsem);
            ALOGV("FreeQ filled up");
            continue;
        }
        List<audioBufferstruct>::iterator it = mFreeQ.begin();
        audioBufferstruct buff ( it->audioBuffer,it->bufferlen);
        mFreeQ.erase(it);
        mFreeQLock.unlock();

        buff.bufferlen = record->read(buff.audioBuffer, mBufferSize);
        ALOGV("read %d bytes", buff.bufferlen);
        if (buff.bufferlen <= 0){
            ALOGE("error in reading from audiorecord..bailing out.");
            this ->notify(MEDIA_RECORDER_EVENT_ERROR, MEDIA_RECORDER_ERROR_UNKNOWN,
                           ERROR_MALFORMED);
            err = INVALID_OPERATION ;
            break;
        }

        mDataQLock.lock();
        if(mDataQ.empty()){
            ALOGV("waking up reader");
            sem_post(&mWriterThreadWakeupsem);
        }
        mDataQ.push_back(buff);
        mDataQLock.unlock();
    }
    record->stop();
    delete record;

    return err;
}


status_t FMA2DPWriter::writerthread(){
    status_t err = OK;
    int framecount =(16*mBufferSize)/sizeof(int16_t);
    //sizeof(int16_t) is frame size for PCM stream
    int outChannel = (mAudioChannels== 2) ? AUDIO_CHANNEL_OUT_STEREO :
        AUDIO_CHANNEL_OUT_MONO;

    prctl(PR_SET_NAME, (unsigned long)"FMA2DPWriterThread", 0, 0, 0);

    AudioTrack *audioTrack= new AudioTrack(
                AUDIO_STREAM_FM,
                mSampleRate,
                mAudioFormat,
                outChannel,
                framecount, 0);

    if(!audioTrack){
        ALOGE("fatal:Not able to open audiotrack");
        return UNKNOWN_ERROR;
    }
=======
    while (true) {
        {
            Mutex::Autolock _l (mLock);
            if(mDone)
                break;

            if(mFreeList.empty()){
                mCondVar.signal();
                continue;
            }

            int32_t index = *(mFreeList.begin());
            mFreeList.erase(mFreeList.begin());

            int len = record->read(mFMDataPool.editItemAt(index)->audioBuffer, mBufferSize);
            ALOGV("%s read %d bytes", __func__, len);
            if (len <= 0){
                ALOGE("%s error in reading from audiorecord..bailing out.", __func__);
                this ->notify(MEDIA_RECORDER_EVENT_ERROR, MEDIA_RECORDER_ERROR_UNKNOWN,
                              ERROR_MALFORMED);
                err = INVALID_OPERATION;
                break;
            }

            mDataList.push_back(index);
            mCondVar.signal();
        }
    }
    record->stop();
    return err;
}

status_t FMA2DPWriter::writerthread(){
    status_t err = OK;
    int framecount = (16*mBufferSize)/sizeof(int16_t);
    //sizeof(int16_t) is frame size for PCM stream
    int outChannel = (mAudioChannels== 2) ? AUDIO_CHANNEL_OUT_STEREO :
        AUDIO_CHANNEL_OUT_MONO ;

    prctl(PR_SET_NAME, (unsigned long)"FMA2DPWriterThread", 0, 0, 0);

    sp<AudioTrack> audioTrack;
    audioTrack = new AudioTrack(AUDIO_STREAM_MUSIC,
                                mSampleRate,
                                mAudioFormat,
                                outChannel,
                                framecount);

    if(audioTrack.get() == NULL) {
        ALOGE("fatal:Not able to open audiotrack");
        return UNKNOWN_ERROR;
    }

>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    status_t res = audioTrack->initCheck();
    if (res == NO_ERROR) {
        audioTrack->setVolume(1, 1);
        audioTrack->start();
<<<<<<< HEAD
    }
    else{
=======
    } else {
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
        ALOGE("fatal:audiotrack init check failure");
        return UNKNOWN_ERROR;
    }

<<<<<<< HEAD

    while (!mDone) {

        mDataQLock.lock();
        if(mDataQ.empty()){
            mDataQLock.unlock();
            ALOGV("dataQ empty");
            sem_wait(&mWriterThreadWakeupsem);
            ALOGV("dataQ filled up");
            continue;
        }
        List<audioBufferstruct>::iterator it = mDataQ.begin();
        audioBufferstruct buff ( it->audioBuffer,it->bufferlen);
        mDataQ.erase(it);
        mDataQLock.unlock();

       size_t retval = audioTrack->write(buff.audioBuffer, buff.bufferlen);
       if(!retval){
            ALOGE("audio track write failure..bailing out");
            this ->notify(MEDIA_RECORDER_EVENT_ERROR, MEDIA_RECORDER_ERROR_UNKNOWN,
                           ERROR_MALFORMED);
            err = INVALID_OPERATION ;
            break;
        }
        ALOGV("wrote %d bytes", buff.bufferlen);

        mFreeQLock.lock();
        if(mFreeQ.empty()){
            ALOGV("WAKING UP READER");
            sem_post(&mReaderThreadWakeupsem);
        }
        mFreeQ.push_back(buff);
        mFreeQLock.unlock();
    }
    audioTrack->stop();
    delete audioTrack;

=======
    while (true) {
        {
            Mutex::Autolock _l (mLock);
            if (mDone)
                break;

            if(mDataList.empty()){
                mCondVar.wait(mLock);
                continue;
            }

            int32_t index = *(mDataList.begin());
            mDataList.erase(mDataList.begin());

            size_t ret = audioTrack->write(mFMDataPool.editItemAt(index)->audioBuffer,
                                           mFMDataPool.editItemAt(index)->bufferLen);
            if (!ret) {
                ALOGE("%s audio track write failure.. bailing out", __func__);
                this->notify(MEDIA_RECORDER_EVENT_ERROR, MEDIA_RECORDER_ERROR_UNKNOWN,
                              ERROR_MALFORMED);
                err = INVALID_OPERATION;
                break;
            }
            ALOGV("%s wrote %d bytes", __func__, ret);

            mFreeList.push_back(index);
            mCondVar.signal();
        }
    }
    audioTrack->stop();
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    return err;
}

bool FMA2DPWriter::reachedEOS() {
<<<<<<< HEAD
// API not need for FMA2DPWriter
    return OK;
}


=======
    //API not need for FMA2DPWriter
    return OK;
}

>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
}  // namespace android
