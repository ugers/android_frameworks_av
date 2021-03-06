/*******************************************************************************
--                                                                            --
--                    CedarX Multimedia Framework                             --
--                                                                            --
--          the Multimedia Framework for Linux/Android System                 --
--                                                                            --
--       This software is confidential and proprietary and may be used        --
--        only as expressly authorized by a licensing agreement from          --
--                         Softwinner Products.                               --
--                                                                            --
--                   (C) COPYRIGHT 2011 SOFTWINNER PRODUCTS                   --
--                            ALL RIGHTS RESERVED                             --
--                                                                            --
--                 The entire notice above must be reproduced                 --
--                  on all copies and should not be removed.                  --
--                                                                            --
*******************************************************************************/

#ifndef CEDARX_STREAM_FILE_H_
#define CEDARX_STREAM_FILE_H_
#include <cedarx_stream.h>

int cdx_seek_stream_file(struct cdx_stream_info *stream, cdx_off_t offset, int whence);
cdx_off_t cdx_tell_stream_file(struct cdx_stream_info *stream);
int cdx_read_stream_file(void *ptr, size_t size, size_t nmemb, struct cdx_stream_info *stream);
int cdx_write_stream_file(const void *ptr, size_t size, size_t nmemb, struct cdx_stream_info *stream);
long long cdx_get_stream_size_file(struct cdx_stream_info *stream);
int create_outstream_handle_file(struct cdx_stream_info *stm_info, CedarXDataSourceDesc *datasource_desc);

#endif
