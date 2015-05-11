<<<<<<< HEAD
/*Copyright (c) 2012, The Linux Foundation. All rights reserved.
=======
/*Copyright (c) 2012 - 2014, The Linux Foundation. All rights reserved.
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#define LOG_NDEBUG 0
#define LOG_TAG "ExtendedExtractor"
#include <utils/Log.h>
#include <dlfcn.h>  // for dlopen/dlclose
<<<<<<< HEAD

#include "include/ExtendedExtractor.h"

static const char* EXTENDED_PARSER_LIB = "libExtendedExtractor.so";

namespace android {

void* ExtendedParserLib() {
    static void* extendedParserLib = NULL;
    static bool alreadyTriedToOpenParsers = false;

    if(!alreadyTriedToOpenParsers) {
        alreadyTriedToOpenParsers = true;

        extendedParserLib = ::dlopen(EXTENDED_PARSER_LIB, RTLD_LAZY);

        if(extendedParserLib == NULL) {
            ALOGV("Failed to open EXTENDED_PARSER_LIB, dlerror = %s \n", dlerror());
        }
    }

    return extendedParserLib;
}

MediaExtractor* ExtendedExtractor::CreateExtractor(const sp<DataSource> &source, const char *mime) {
    static MediaExtractorFactory mediaFactoryFunction = NULL;
    static bool alreadyTriedToFindFactoryFunction = false;

    MediaExtractor* extractor = NULL;

    if(!alreadyTriedToFindFactoryFunction) {

        void *extendedParserLib = ExtendedParserLib();
        if (extendedParserLib != NULL) {

            mediaFactoryFunction = (MediaExtractorFactory) dlsym(extendedParserLib, MEDIA_CREATE_EXTRACTOR);
            alreadyTriedToFindFactoryFunction = true;
        }
    }

    if(mediaFactoryFunction==NULL) {
        ALOGE(" dlsym for ExtendedExtractor factory function failed, dlerror = %s \n", dlerror());
        return NULL;
    }

    extractor = mediaFactoryFunction(source, mime);
    if(extractor==NULL) {
        ALOGE(" ExtendedExtractor failed to instantiate extractor \n");
=======
#include "include/ExtendedExtractor.h"

#define ARG_TOUCH(x) (void)x

#ifdef ENABLE_AV_ENHANCEMENTS

namespace android {

static const char* EXTENDED_EXTRACTOR_LIB = "libExtendedExtractor.so";
static const char* EXTENDED_EXTRACTOR_CREATE = "CreateExtractor";
static const char* EXTENDED_EXTRACTOR_SNIFF = "SniffExtendedExtractor";

typedef MediaExtractor* (*ExtendedExtractorCreate)
                            (const sp<DataSource> &source, const char* mime);

typedef bool (*ExtendedExtractorSniff)
                 (const sp<DataSource> &source, String8 *mimeType,
                 float *confidence,sp<AMessage> *meta);

static void* loadExtendedExtractorLib() {
    static void* extendedExtractorLib = NULL;
    static bool alreadyTriedToLoadLib = false;

    if(!alreadyTriedToLoadLib) {
        alreadyTriedToLoadLib = true;

        extendedExtractorLib = ::dlopen(EXTENDED_EXTRACTOR_LIB, RTLD_LAZY);

        if(extendedExtractorLib == NULL) {
            ALOGV("Failed to load %s, dlerror = %s \n",
                EXTENDED_EXTRACTOR_LIB, dlerror());
        }
    }

    return extendedExtractorLib;
}

MediaExtractor* ExtendedExtractor::Create (
        const sp<DataSource> &source, const char *mime) {
    static ExtendedExtractorCreate create = NULL;
    static bool alreadyTriedToFindCreateFunction = false;
    MediaExtractor* extractor = NULL;

    if (!alreadyTriedToFindCreateFunction) {
        void *extendedExtractorLib = loadExtendedExtractorLib();

        if (extendedExtractorLib != NULL) {
            create = (ExtendedExtractorCreate) dlsym (
                    extendedExtractorLib, EXTENDED_EXTRACTOR_CREATE);
            alreadyTriedToFindCreateFunction = true;
        }
    }

    if (create == NULL) {
        ALOGE ("Failed to find symbol : %s, dlerror = %s",
            EXTENDED_EXTRACTOR_CREATE, dlerror());
        return NULL;
    }

    extractor = create (source, mime);
    if (extractor == NULL) {
        ALOGE("Failed to instantiate extractor \n");
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
    }

    return extractor;
}

<<<<<<< HEAD
bool SniffExtendedExtractor(const sp<DataSource> &source, String8 *mimeType,
                            float *confidence,sp<AMessage> *meta) {
    void *extendedParserLib = ExtendedParserLib();
    bool retVal = false;
    if (extendedParserLib != NULL) {
       ExtendedExtractorSniffers extendedExtractorSniffers=
           (ExtendedExtractorSniffers) dlsym(extendedParserLib, EXTENDED_EXTRACTOR_SNIFFERS);

       if(extendedExtractorSniffers == NULL) {
           ALOGE(" dlsym for extendedExtractorSniffers function failed, dlerror = %s \n", dlerror());
           return retVal;
       }

       retVal = extendedExtractorSniffers(source, mimeType, confidence, meta);

       if(!retVal) {
           ALOGV("ExtendedExtractor:: ExtendedExtractorSniffers  Failed");
=======
bool ExtendedExtractor::Sniff (
        const sp<DataSource> &source, String8 *mimeType,
        float *confidence,sp<AMessage> *meta) {
    void *extendedExtractorLib = loadExtendedExtractorLib();
    bool retVal = false;

    if (extendedExtractorLib != NULL) {
       ExtendedExtractorSniff sniff = (ExtendedExtractorSniff) dlsym (
               extendedExtractorLib, EXTENDED_EXTRACTOR_SNIFF);

        if (sniff == NULL) {
            ALOGE ("Failed to find symbol : %s, dlerror = %s",
                EXTENDED_EXTRACTOR_SNIFF, dlerror());
            return retVal;
        }

       retVal = sniff (source, mimeType, confidence, meta);

       if(!retVal) {
           ALOGV("Sniff Failed");
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e
       }
    }
    return retVal;
}

}  // namespace android

<<<<<<< HEAD
=======
#else //ENABLE_AV_ENHANCEMENTS

namespace android {

MediaExtractor* ExtendedExtractor::Create (
        const sp<DataSource> &source, const char *mime) {
    ARG_TOUCH(source);
    ARG_TOUCH(mime);
    return NULL;
}
bool ExtendedExtractor::Sniff (
        const sp<DataSource> &source, String8 *mimeType,
        float *confidence, sp<AMessage> *meta) {
    ARG_TOUCH(source);
    ARG_TOUCH(mimeType);
    ARG_TOUCH(confidence);
    ARG_TOUCH(meta);
    *confidence = 0.0;
    return false;
}

}  // namespace android

#endif //ENABLE_AV_ENHANCEMENTS
>>>>>>> 8b8d02886bd9fb8d5ad451c03e486cfad74aa74e

