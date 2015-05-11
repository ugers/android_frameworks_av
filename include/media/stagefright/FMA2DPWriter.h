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
 * Copyright (C) 2010 The Android Open Source Project *
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

#ifndef FM_A2DP_WRITER_H_

#define FM_A2DP_WRITER_H_

<<<<<<< HEAD
#include <stdio.h>
=======
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e

#include <media/stagefright/MediaWriter.h>
#include <utils/threads.h>
#include <media/AudioRecord.h>
#include <utils/List.h>
<<<<<<< HEAD
=======
#include <utils/Vector.h>
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
#include <semaphore.h>
#include <media/mediarecorder.h>

namespace android {

<<<<<<< HEAD
struct MediaSource;
struct MetaData;

struct audioBufferstruct {
   public:
   audioBufferstruct (void *buff, size_t bufflen)
      :audioBuffer(buff), bufferlen(bufflen){}

   void  *audioBuffer;
   size_t bufferlen;
 };
=======
#define SAMPLING_RATE 48000
#define AUDIO_CHANNELS 2

struct MediaSource;
struct MetaData;

struct FMData {
   FMData(void *buffer, size_t len) :audioBuffer(buffer), bufferLen(len) {}

   void  *audioBuffer;
   size_t bufferLen;
};
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e

struct FMA2DPWriter : public MediaWriter {
    FMA2DPWriter();

    status_t initCheck() const;
    virtual status_t addSource(const sp<MediaSource> &source);
    virtual bool reachedEOS();
    virtual status_t start(MetaData *params = NULL);
    virtual status_t stop();
    virtual status_t pause();
    virtual status_t allocateBufferPool();

protected:
    virtual ~FMA2DPWriter();

private:
<<<<<<< HEAD
    List<audioBufferstruct > mFreeQ,mDataQ;
    Mutex mFreeQLock,mDataQLock;
    sem_t mReaderThreadWakeupsem,mWriterThreadWakeupsem;
    pthread_t mReaderThread,mWriterThread;
=======
    List<int32_t> mFreeList, mDataList;
    Vector<FMData *> mFMDataPool;
    Mutex mLock;
    Condition mCondVar;
    pthread_t mReaderThread, mWriterThread;
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    bool mStarted;
    volatile bool mDone;
    int32_t mAudioChannels;
    int32_t mSampleRate;
    audio_format_t mAudioFormat;
    audio_source_t mAudioSource;
    size_t mBufferSize;
    static void *ReaderThreadWrapper(void *);
    static void *WriterThreadWrapper(void *);
    status_t readerthread();
    status_t writerthread();
    FMA2DPWriter(const FMA2DPWriter &);
    FMA2DPWriter &operator=(const FMA2DPWriter &);
};

}  // namespace android

#endif  // FM_A2DP_WRITER_H_
