/* Copyright (c) 2012 - 2020, The Linux Foundation. All rights reserved.
 *
 * redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * this software is provided "as is" and any express or implied
 * warranties, including, but not limited to, the implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement
 * are disclaimed.  in no event shall the copyright owner or contributors
 * be liable for any direct, indirect, incidental, special, exemplary, or
 * consequential damages (including, but not limited to, procurement of
 * substitute goods or services; loss of use, data, or profits; or
 * business interruption) however caused and on any theory of liability,
 * whether in contract, strict liability, or tort (including negligence
 * or otherwise) arising in any way out of the use of this software, even
 * if advised of the possibility of such damage.
 *
 */

#include <C2DColorConverter.h>

void swap(size_t &x, size_t &y)
{
    x = x ^ y;
    y = x ^ y;
    x = x ^ y;
}

C2DColorConverter::C2DColorConverter()
    : mC2DLibHandle(NULL),
      mAdrenoUtilsHandle(NULL)
{

    enabled = true;
    mSrcSurface = 0;
    mDstSurface = 0;

    mSrcWidth = 0;
    mSrcHeight = 0;
    mSrcStride = 0;
    mDstWidth = 0;
    mDstHeight = 0;
    mRotation = C2D_TARGET_ROTATE_0;
    mSrcFormat = NO_COLOR_FORMAT;
    mDstFormat = NO_COLOR_FORMAT;
    mSrcSurfaceDef = NULL;
    mDstSurfaceDef = NULL;

    mConversionNeeded = false;

    pthread_mutex_init(&mLock, NULL);

    mC2DLibHandle = dlopen("libC2D2.so", RTLD_NOW);
    if (!mC2DLibHandle) {
        ALOGE("%s: ERROR: could not dlopen libc2d2.so: %s. C2D is disabled.",
                                                    __FUNCTION__, dlerror());
        enabled = false;
        return;
    }
    mAdrenoUtilsHandle = dlopen("libadreno_utils.so", RTLD_NOW);
    if (!mAdrenoUtilsHandle) {
        ALOGE("%s: ERROR: could not dlopen libadreno_utils.so: %s.. C2D is disabled.",
                                                    __FUNCTION__, dlerror());
        enabled = false;
        return;
    }

    mC2DCreateSurface = (LINK_c2dCreateSurface)dlsym(mC2DLibHandle, "c2dCreateSurface");
    mC2DUpdateSurface = (LINK_c2dUpdateSurface)dlsym(mC2DLibHandle, "c2dUpdateSurface");
    mC2DReadSurface = (LINK_c2dReadSurface)dlsym(mC2DLibHandle, "c2dReadSurface");
    mC2DDraw = (LINK_c2dDraw)dlsym(mC2DLibHandle, "c2dDraw");
    mC2DFlush = (LINK_c2dFlush)dlsym(mC2DLibHandle, "c2dFlush");
    mC2DFinish = (LINK_c2dFinish)dlsym(mC2DLibHandle, "c2dFinish");
    mC2DWaitTimestamp = (LINK_c2dWaitTimestamp)dlsym(mC2DLibHandle, "c2dWaitTimestamp");
    mC2DDestroySurface = (LINK_c2dDestroySurface)dlsym(mC2DLibHandle, "c2dDestroySurface");
    mC2DMapAddr = (LINK_c2dMapAddr)dlsym(mC2DLibHandle, "c2dMapAddr");
    mC2DUnMapAddr = (LINK_c2dUnMapAddr)dlsym(mC2DLibHandle, "c2dUnMapAddr");

    if (!mC2DCreateSurface || !mC2DUpdateSurface || !mC2DReadSurface
        || !mC2DDraw || !mC2DFlush || !mC2DFinish || !mC2DWaitTimestamp
        || !mC2DDestroySurface || !mC2DMapAddr || !mC2DUnMapAddr) {
        ALOGE("%s: dlsym ERROR. C2D is disabled. mC2DCreateSurface[%p] mC2DUpdateSurface[%p] "
              "mC2DReadSurface[%p] mC2DDraw[%p] mC2DFlush[%p] mC2DFinish[%p] mC2DWaitTimestamp[%p] "
              "mC2DDestroySurface[%p] mC2DMapAddr[%p] mC2DUnMapAddr[%p]", __FUNCTION__,
              mC2DCreateSurface, mC2DUpdateSurface, mC2DReadSurface, mC2DDraw, mC2DFlush, mC2DFinish,
              mC2DWaitTimestamp, mC2DDestroySurface, mC2DMapAddr, mC2DUnMapAddr);
        enabled = false;
        return;
    }

    mAdrenoComputeFmtAlignedWidthAndHeight = (LINK_adreno_compute_fmt_aligned_width_and_height)dlsym(mAdrenoUtilsHandle, "compute_fmt_aligned_width_and_height");
    if (!mAdrenoComputeFmtAlignedWidthAndHeight) {
        ALOGE("%s: dlsym compute_aligned_width_and_height ERROR. C2D is disabled.", __FUNCTION__);
        enabled = false;
        return;
    }
}

C2DColorConverter::~C2DColorConverter()
{
    if (enabled) {

        if (mDstSurface) {
            mC2DDestroySurface(mDstSurface);
            mDstSurface = 0;
        }
        if (mSrcSurface) {
            mC2DDestroySurface(mSrcSurface);
            mSrcSurface = 0;
        }

        if (mSrcSurfaceDef) {
            if (isYUVSurface(mSrcFormat)) {
                delete ((C2D_YUV_SURFACE_DEF *)mSrcSurfaceDef);
            } else {
                delete ((C2D_RGB_SURFACE_DEF *)mSrcSurfaceDef);
            }
        }

        if (mDstSurfaceDef) {
            if (isYUVSurface(mDstFormat)) {
                delete ((C2D_YUV_SURFACE_DEF *)mDstSurfaceDef);
            } else {
                delete ((C2D_RGB_SURFACE_DEF *)mDstSurfaceDef);
            }
        }
        mSrcSurfaceDef = NULL;
        mDstSurfaceDef = NULL;
    }

    if (mC2DLibHandle) {
        dlclose(mC2DLibHandle);
        mC2DLibHandle = NULL;
    }
    if (mAdrenoUtilsHandle) {
        dlclose(mAdrenoUtilsHandle);
        mAdrenoUtilsHandle = NULL;
    }

}

bool C2DColorConverter::isPropChanged(size_t srcWidth, size_t srcHeight, size_t dstWidth,
                                      size_t dstHeight, ColorConvertFormat srcFormat,
                                      ColorConvertFormat dstFormat, int32_t flags,
                                      size_t srcStride)
{
    return (mSrcWidth  != srcWidth   ||
            mSrcHeight != srcHeight  ||
            mDstWidth  != dstWidth   ||
            mDstHeight != dstHeight  ||
            mSrcFormat != srcFormat  ||
            mDstFormat != dstFormat  ||
            mSrcStride != srcStride  ||
            (mFlags & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED)  !=
                      (flags  & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED));
}

bool C2DColorConverter::setResolution(size_t srcWidth, size_t srcHeight,
                                      size_t dstWidth, size_t dstHeight,
                                      ColorConvertFormat srcFormat,
                                      ColorConvertFormat dstFormat,
                                      int32_t flags, size_t srcStride)
{
    int32_t retval = -1;
    if (enabled) {
        pthread_mutex_lock(&mLock);

        ClearSurfaces();

        mSrcWidth = srcWidth;
        mSrcHeight = srcHeight;
        mSrcStride = srcStride;
        mDstWidth = dstWidth;
        mDstHeight = dstHeight;
        mSrcFormat = srcFormat;
        mDstFormat = dstFormat;

        if (mRotation == C2D_TARGET_ROTATE_90 ||
            mRotation == C2D_TARGET_ROTATE_270) {
                swap(mDstWidth, mDstHeight);
        }
        mSrcSize = calcSize(srcFormat, mSrcWidth, mSrcHeight);
        mDstSize = calcSize(dstFormat, mDstWidth, mDstHeight);
        mSrcYSize = calcYSize(srcFormat, mSrcWidth, mSrcHeight);
        mDstYSize = calcYSize(dstFormat, mDstWidth, mDstHeight);
        mFlags = flags;

        retval = getDummySurfaceDef(srcFormat, mSrcWidth, mSrcHeight, true);
        retval |= getDummySurfaceDef(dstFormat, mDstWidth, mDstHeight, false);

        if (retval == 0) {
            memset((void*)&mBlit,0,sizeof(C2D_OBJECT));
            mBlit.source_rect.x = 0 << 16;
            mBlit.source_rect.y = 0 << 16;
            mBlit.source_rect.width = srcWidth << 16;
            mBlit.source_rect.height = srcHeight << 16;
            mBlit.target_rect.x = 0 << 16;
            mBlit.target_rect.y = 0 << 16;
            mBlit.target_rect.width = dstWidth << 16;
            mBlit.target_rect.height = dstHeight << 16;
            mBlit.config_mask = C2D_ALPHA_BLEND_NONE    |
                C2D_NO_BILINEAR_BIT     |
                C2D_NO_ANTIALIASING_BIT |
                C2D_TARGET_RECT_BIT;
            mBlit.surface_id = mSrcSurface;
        }

        pthread_mutex_unlock(&mLock);
    }

    return retval == 0? true:false;
}

void C2DColorConverter::setRotation(int32_t rotation) {
    // C2D does rotation in anticlock wise, where as VPE rotates in clockwise
    // Hence swapping the 90 and 270 angles to rotate in clockwise
    switch (rotation) {
        case 90:
            mRotation  = C2D_TARGET_ROTATE_270;
            break;
        case 180:
            mRotation  = C2D_TARGET_ROTATE_180;
            break;
        case 270:
            mRotation  = C2D_TARGET_ROTATE_90;
            break;
        default:
            mRotation = C2D_TARGET_ROTATE_0;
            break;
    }
}
bool C2DColorConverter::convertC2D(int srcFd, void *srcBase, void * srcData,
                                   int dstFd, void *dstBase, void * dstData)
{
  C2D_STATUS ret;
  uint8_t *srcMappedGpuAddr = nullptr;
  uint8_t *dstMappedGpuAddr = nullptr;
  bool status = false;

  if (enabled) {
    pthread_mutex_lock(&mLock);
    if (srcFd < 0 || dstFd < 0
                || srcData == NULL || dstData == NULL
                || srcBase == NULL || dstBase == NULL) {
      ALOGE("%s: Color conversion failed. Incorrect input parameters FD (%d:%d) Data (%p:%p) Base (%p:%p)",
            __FUNCTION__, srcFd, dstFd, srcData, dstData, srcBase, dstBase);
      status = false;
    } else {

      srcMappedGpuAddr = (uint8_t *)getMappedGPUAddr(srcFd, srcData, mSrcSize);
      if (!srcMappedGpuAddr) {
          pthread_mutex_unlock(&mLock);
          return false;
      }

      if (isYUVSurface(mSrcFormat)) {
        ret = updateYUVSurfaceDef(srcMappedGpuAddr, srcBase, srcData, true);
      } else {
        ret = updateRGBSurfaceDef(srcMappedGpuAddr, srcData, true);
      }

      if (ret == C2D_STATUS_OK) {

        dstMappedGpuAddr = (uint8_t *)getMappedGPUAddr(dstFd, dstData, mDstSize);
        if (!dstMappedGpuAddr) {
          unmapGPUAddr((unsigned long)srcMappedGpuAddr);
          pthread_mutex_unlock(&mLock);
          return false;
        }

        if (isYUVSurface(mDstFormat)) {
          ret = updateYUVSurfaceDef(dstMappedGpuAddr, dstBase, dstData, false);
        } else {
          ret = updateRGBSurfaceDef(dstMappedGpuAddr, dstData, false);
        }

        if (ret == C2D_STATUS_OK) {

          mBlit.surface_id = mSrcSurface;
          ret = mC2DDraw(mDstSurface, mRotation, 0, 0, 0, &mBlit, 1);
          mC2DFinish(mDstSurface);

          if (ret == C2D_STATUS_OK) {
            bool unmappedSrcSuccess;
            unmappedSrcSuccess = unmapGPUAddr((unsigned long)srcMappedGpuAddr);

            bool unmappedDstSuccess;
            unmappedDstSuccess = unmapGPUAddr((unsigned long)dstMappedGpuAddr);

            if (!unmappedSrcSuccess || !unmappedDstSuccess) {
              ALOGE("%s: unmapping GPU address failed (%d:%d)", __FUNCTION__,
                    unmappedSrcSuccess, unmappedDstSuccess);
              status = false;
            } else {
              status = true;
            }
          } else {
            ALOGE("%s: C2D Draw failed (%d)", __FUNCTION__, ret);
            status = false;
          }
        } else {
          ALOGE("%s: Update dst surface def failed (%d)", __FUNCTION__, ret);
          unmapGPUAddr((unsigned long)srcMappedGpuAddr);
          unmapGPUAddr((unsigned long)dstMappedGpuAddr);
          status = false;
        }
      } else {
        ALOGE("%s: Update src surface def failed (%d)", __FUNCTION__, ret);
        unmapGPUAddr((unsigned long)srcMappedGpuAddr);
        status = false;
      }
    }

    pthread_mutex_unlock(&mLock);
  }

  return status;
}

bool C2DColorConverter::isYUVSurface(ColorConvertFormat format)
{
    switch (format) {
        case YCbCr420Tile:
        case YCbCr420SP:
        case YCbCr420P:
        case YCrCb420P:
        case NV12_2K:
        case NV12_512:
        case NV12_128m:
        case NV12_UBWC:
        case TP10_UBWC:
        case P010:
        case VENUS_P010:
            return true;
        default:
            return false;
    }
}

void C2DColorConverter::ClearSurfaces()
{
        if (mSrcSurface) {
            mC2DDestroySurface(mSrcSurface);
            mSrcSurface = 0;
        }

         if (mSrcSurfaceDef) {
            if (isYUVSurface(mSrcFormat)) {
                delete ((C2D_YUV_SURFACE_DEF *)mSrcSurfaceDef);
            } else {
                delete ((C2D_RGB_SURFACE_DEF *)mSrcSurfaceDef);
            }
            mSrcSurfaceDef = NULL;
        }

        if (mDstSurface) {
            mC2DDestroySurface(mDstSurface);
            mDstSurface = 0;
        }

        if (mDstSurfaceDef) {
            if (isYUVSurface(mDstFormat)) {
                delete ((C2D_YUV_SURFACE_DEF *)mDstSurfaceDef);
            } else {
                delete ((C2D_RGB_SURFACE_DEF *)mDstSurfaceDef);
            }
            mDstSurfaceDef = NULL;
        }
}

int32_t C2DColorConverter::getDummySurfaceDef(ColorConvertFormat format,
                                            size_t width, size_t height,
                                            bool isSource)
{
    void *surfaceDef = NULL;
    C2D_SURFACE_TYPE hostSurfaceType;
    C2D_STATUS ret;

    if (isYUVSurface(format)) {
        C2D_YUV_SURFACE_DEF **surfaceYUVDef = (C2D_YUV_SURFACE_DEF **)
                                  (isSource ? &mSrcSurfaceDef : &mDstSurfaceDef);
        if (*surfaceYUVDef == NULL) {
            *surfaceYUVDef = (C2D_YUV_SURFACE_DEF *)
                                  calloc(1, sizeof(C2D_YUV_SURFACE_DEF));
            if (*surfaceYUVDef == NULL) {
                ALOGE("%s: surfaceYUVDef allocation failed", __FUNCTION__);
                return -1;
            }
        } else {
            memset(*surfaceYUVDef, 0, sizeof(C2D_YUV_SURFACE_DEF));
        }
        (*surfaceYUVDef)->format = getC2DFormat(format, isSource);
        (*surfaceYUVDef)->width = width;
        (*surfaceYUVDef)->height = height;
        (*surfaceYUVDef)->plane0 = (void *)0xaaaaaaaa;
        (*surfaceYUVDef)->phys0 = (void *)0xaaaaaaaa;
        (*surfaceYUVDef)->stride0 = calcStride(format, width);
        (*surfaceYUVDef)->plane1 = (void *)0xaaaaaaaa;
        (*surfaceYUVDef)->phys1 = (void *)0xaaaaaaaa;
        (*surfaceYUVDef)->stride1 = calcStride(format, width);
        (*surfaceYUVDef)->stride2 = calcStride(format, width);
        (*surfaceYUVDef)->phys2 = NULL;
        (*surfaceYUVDef)->plane2 = NULL;

        if (mFlags & private_handle_t::PRIV_FLAGS_ITU_R_601_FR) {
            (*surfaceYUVDef)->format |= C2D_FORMAT_BT601_FULLRANGE;
        }

        if (format == YCbCr420P ||
            format == YCrCb420P) {
          ALOGI("%s: half stride for Cb Cr planes \n", __FUNCTION__);
          (*surfaceYUVDef)->stride1 = calcStride(format, width) / 2;
          (*surfaceYUVDef)->phys2 = (void *)0xaaaaaaaa;
          (*surfaceYUVDef)->stride2 = calcStride(format, width) / 2;
        }

        surfaceDef = *surfaceYUVDef;
        hostSurfaceType = C2D_SURFACE_YUV_HOST;
    } else {
        C2D_RGB_SURFACE_DEF **surfaceRGBDef = (C2D_RGB_SURFACE_DEF **)
                                  (isSource ? &mSrcSurfaceDef : &mDstSurfaceDef);
        if (*surfaceRGBDef == NULL) {
            *surfaceRGBDef = (C2D_RGB_SURFACE_DEF *)
                                  calloc(1, sizeof(C2D_RGB_SURFACE_DEF));
            if (*surfaceRGBDef == NULL) {
                ALOGE("%s: surfaceRGBDef allocation failed", __FUNCTION__);
                return -1;
            }
        } else {
            memset(*surfaceRGBDef, 0, sizeof(C2D_RGB_SURFACE_DEF));
        }
        (*surfaceRGBDef)->format = getC2DFormat(format, isSource);

        if (mFlags & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED ||
            mFlags & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED_PI)
            (*surfaceRGBDef)->format |= C2D_FORMAT_UBWC_COMPRESSED;
        (*surfaceRGBDef)->width = width;
        (*surfaceRGBDef)->height = height;
        (*surfaceRGBDef)->buffer = (void *)0xaaaaaaaa;
        (*surfaceRGBDef)->phys = (void *)0xaaaaaaaa;
        (*surfaceRGBDef)->stride = calcStride(format, width);

        surfaceDef = *surfaceRGBDef;
        hostSurfaceType = C2D_SURFACE_RGB_HOST;
    }

    ret = mC2DCreateSurface(isSource ? &mSrcSurface :
                      &mDstSurface,
                      isSource ? C2D_SOURCE : C2D_TARGET,
                      (C2D_SURFACE_TYPE)(hostSurfaceType
                                         | C2D_SURFACE_WITH_PHYS
                                         | C2D_SURFACE_WITH_PHYS_DUMMY),
                      surfaceDef);
    return (int32_t) ret;
}

C2D_STATUS C2DColorConverter::updateYUVSurfaceDef(uint8_t *gpuAddr, void *base,
                                                  void *data, bool isSource)
{
    if (isSource) {
        C2D_YUV_SURFACE_DEF * srcSurfaceDef = (C2D_YUV_SURFACE_DEF *)mSrcSurfaceDef;
        srcSurfaceDef->plane0 = data;
        srcSurfaceDef->phys0  = gpuAddr + ((uint8_t *)data - (uint8_t *)base);
        srcSurfaceDef->plane1 = (uint8_t *)data + mSrcYSize;
        srcSurfaceDef->phys1  = (uint8_t *)srcSurfaceDef->phys0 + mSrcYSize;
        if (srcSurfaceDef->format & C2D_COLOR_FORMAT_420_I420 ||
               srcSurfaceDef->format & C2D_COLOR_FORMAT_420_YV12) {
            srcSurfaceDef->plane2 = (uint8_t *)srcSurfaceDef->plane1 + mSrcYSize/4;
            srcSurfaceDef->phys2  = (uint8_t *)srcSurfaceDef->phys1 + mSrcYSize/4;
        }
        return mC2DUpdateSurface(mSrcSurface, C2D_SOURCE,
                        (C2D_SURFACE_TYPE)(C2D_SURFACE_YUV_HOST | C2D_SURFACE_WITH_PHYS),
                        &(*srcSurfaceDef));
    } else {
        C2D_YUV_SURFACE_DEF * dstSurfaceDef = (C2D_YUV_SURFACE_DEF *)mDstSurfaceDef;
        dstSurfaceDef->plane0 = data;
        dstSurfaceDef->phys0  = gpuAddr + ((uint8_t *)data - (uint8_t *)base);
        dstSurfaceDef->plane1 = (uint8_t *)data + mDstYSize;
        dstSurfaceDef->phys1  = (uint8_t *)dstSurfaceDef->phys0 + mDstYSize;
        if (dstSurfaceDef->format & C2D_COLOR_FORMAT_420_I420 ||
               dstSurfaceDef->format & C2D_COLOR_FORMAT_420_YV12) {
            dstSurfaceDef->plane2 = (uint8_t *)dstSurfaceDef->plane1 + mDstYSize/4;
            dstSurfaceDef->phys2  = (uint8_t *)dstSurfaceDef->phys1 + mDstYSize/4;
        }

        return mC2DUpdateSurface(mDstSurface, C2D_TARGET,
                        (C2D_SURFACE_TYPE)(C2D_SURFACE_YUV_HOST | C2D_SURFACE_WITH_PHYS),
                        &(*dstSurfaceDef));
    }
}

C2D_STATUS C2DColorConverter::updateRGBSurfaceDef(uint8_t *gpuAddr, void * data, bool isSource)
{
    if (isSource) {
        C2D_RGB_SURFACE_DEF * srcSurfaceDef = (C2D_RGB_SURFACE_DEF *)mSrcSurfaceDef;
        srcSurfaceDef->buffer = data;
        srcSurfaceDef->phys = gpuAddr;
        return  mC2DUpdateSurface(mSrcSurface, C2D_SOURCE,
                        (C2D_SURFACE_TYPE)(C2D_SURFACE_RGB_HOST | C2D_SURFACE_WITH_PHYS),
                        &(*srcSurfaceDef));
    } else {
        C2D_RGB_SURFACE_DEF * dstSurfaceDef = (C2D_RGB_SURFACE_DEF *)mDstSurfaceDef;
        dstSurfaceDef->buffer = data;
        ALOGV("%s: dstSurfaceDef->buffer = %p", __FUNCTION__, data);
        dstSurfaceDef->phys = gpuAddr;
        return mC2DUpdateSurface(mDstSurface, C2D_TARGET,
                        (C2D_SURFACE_TYPE)(C2D_SURFACE_RGB_HOST | C2D_SURFACE_WITH_PHYS),
                        &(*dstSurfaceDef));
    }
}

uint32_t C2DColorConverter::getC2DFormat(ColorConvertFormat format, bool isSource)
{
    uint32_t C2DFormat;
    switch (format) {
        case RGB565:
            return C2D_COLOR_FORMAT_565_RGB;
        case RGBA8888:
            C2DFormat = C2D_COLOR_FORMAT_8888_RGBA | C2D_FORMAT_SWAP_ENDIANNESS;
            if (isSource)
                C2DFormat |= C2D_FORMAT_PREMULTIPLIED;
            return C2DFormat;
        case RGBA8888_UBWC:
            C2DFormat = C2D_COLOR_FORMAT_8888_RGBA |
                        C2D_FORMAT_SWAP_ENDIANNESS |
                        C2D_FORMAT_UBWC_COMPRESSED;
            if (isSource)
                C2DFormat |= C2D_FORMAT_PREMULTIPLIED;
            return C2DFormat;
        case YCbCr420Tile:
            return (C2D_COLOR_FORMAT_420_NV12 | C2D_FORMAT_MACROTILED);
        case YCbCr420SP:
        case NV12_2K:
        case NV12_512:
        case NV12_128m:
            return C2D_COLOR_FORMAT_420_NV12;
        case YCbCr420P:
            return C2D_COLOR_FORMAT_420_I420;
        case YCrCb420P:
            return C2D_COLOR_FORMAT_420_YV12;
        case NV12_UBWC:
            return C2D_COLOR_FORMAT_420_NV12 | C2D_FORMAT_UBWC_COMPRESSED;
        case TP10_UBWC:
            return C2D_COLOR_FORMAT_420_TP10 | C2D_FORMAT_UBWC_COMPRESSED;
        case P010:
        case VENUS_P010:
            return C2D_COLOR_FORMAT_420_P010;
        default:
            ALOGW("%s: Format not supported , %d", __FUNCTION__, format);
            return -1;
    }
}

size_t C2DColorConverter::calcStride(ColorConvertFormat format, size_t width)
{
    switch (format) {
        case RGB565:
            return ALIGN(width, ALIGN32) * 2; // RGB565 has width as twice
        case RGBA8888:
            if (mSrcStride)
                return mSrcStride * 4;
            else
                return ALIGN(width, ALIGN32) * 4;
        case YCbCr420Tile:
            return ALIGN(width, ALIGN128);
        case YCbCr420SP:
            return ALIGN(width, ALIGN16);
        case NV12_2K:
            return ALIGN(width, ALIGN16);
        case NV12_512:
            return ALIGN(width, ALIGN512);
        case NV12_128m: {
            int32_t stride_alignment = VENUS_Y_STRIDE(COLOR_FMT_NV12, 1);
            return ALIGN(width, stride_alignment);
        }
        case YCbCr420P:
            return ALIGN(width, ALIGN16);
        case YCrCb420P:
            return ALIGN(width, ALIGN16);
        case NV12_UBWC:
            return VENUS_Y_STRIDE(COLOR_FMT_NV12_UBWC, width);
        case TP10_UBWC:
            return VENUS_Y_STRIDE(COLOR_FMT_NV12_BPP10_UBWC, width);
        case P010:
            return ALIGN(width*2, ALIGN64);
        case VENUS_P010:
            return ALIGN(width*2, ALIGN256);
        default:
            ALOGW("%s: Format not supported , %d", __FUNCTION__, format);
            return 0;
    }
}

size_t C2DColorConverter::calcYSize(ColorConvertFormat format, size_t width, size_t height)
{
    switch (format) {
        case YCbCr420SP:
            return (ALIGN(width, ALIGN16) * height);
        case YCbCr420P:
            return ALIGN(width, ALIGN16) * height;
        case YCrCb420P:
            return ALIGN(width, ALIGN16) * height;
        case YCbCr420Tile:
            return ALIGN(ALIGN(width, ALIGN128) * ALIGN(height, ALIGN32), ALIGN8K);
        case NV12_2K: {
            size_t alignedw = ALIGN(width, ALIGN16);
            size_t lumaSize = ALIGN(alignedw * height, ALIGN2K);
            return lumaSize;
        }
        case NV12_512:
            return ALIGN(width, ALIGN512) * ALIGN(height, ALIGN512);
        case NV12_128m: {
            int32_t stride_alignment = VENUS_Y_STRIDE(COLOR_FMT_NV12, 1);
            int32_t scanline_alignment = VENUS_Y_SCANLINES(COLOR_FMT_NV12, 1);
            return ALIGN(width, stride_alignment) * ALIGN(height, scanline_alignment);
        }
        case NV12_UBWC:
            return ALIGN( VENUS_Y_STRIDE(COLOR_FMT_NV12_UBWC, width) *
                    VENUS_Y_SCANLINES(COLOR_FMT_NV12_UBWC, height), ALIGN4K) +
                 ALIGN( VENUS_Y_META_STRIDE(COLOR_FMT_NV12_UBWC, width) *
                   VENUS_Y_META_SCANLINES(COLOR_FMT_NV12_UBWC, height), ALIGN4K);
        case TP10_UBWC:
            return ALIGN( VENUS_Y_STRIDE(COLOR_FMT_NV12_BPP10_UBWC, width) *
                          VENUS_Y_SCANLINES(COLOR_FMT_NV12_BPP10_UBWC, height), ALIGN4K) +
                ALIGN( VENUS_Y_META_STRIDE(COLOR_FMT_NV12_BPP10_UBWC, width) *
                       VENUS_Y_META_SCANLINES(COLOR_FMT_NV12_BPP10_UBWC, height), ALIGN4K);
        case P010:
            return (ALIGN(width*2, ALIGN64) * height);
        case VENUS_P010:
            return (ALIGN(width*2, ALIGN256) * ALIGN(height, ALIGN32));
        default:
            ALOGW("%s: Format not supported , %d", __FUNCTION__, format);
            return 0;
    }
}

size_t C2DColorConverter::calcSize(ColorConvertFormat format, size_t width, size_t height)
{
    int32_t alignedw = 0;
    int32_t alignedh = 0;
    int32_t size = 0;
    int32_t tile_mode = 0;
    int32_t raster_mode = 0;
    int32_t padding_threshold = 512; /* hardcode for RGB formats */
    int32_t bpp = 0;

    switch (format) {
        case RGB565:
            bpp = 2;
            mAdrenoComputeFmtAlignedWidthAndHeight(width, height,
                                                   0, ADRENO_PIXELFORMAT_B5G6R5,
                                                   1, tile_mode, raster_mode,
                                                   padding_threshold,
                                                   &alignedw, &alignedh);
            ALOGV("%s: alignedw %d alignedh %d", __FUNCTION__,alignedw, alignedh);
            size = alignedw * alignedh * bpp;
            size = ALIGN(size, ALIGN4K);
            break;
        case RGBA8888:
            bpp = 4;
            mAdrenoComputeFmtAlignedWidthAndHeight(width, height,
                                                   0, ADRENO_PIXELFORMAT_R8G8B8A8 ,
                                                   1, tile_mode, raster_mode,
                                                   padding_threshold,
                                                   &alignedw, &alignedh);
            ALOGV("%s: alignedw %d alignedh %d", __FUNCTION__,alignedw, alignedh);
            if (mSrcStride)
              size = mSrcStride *  alignedh * bpp;
            else
              size = alignedw * alignedh * bpp;
            size = ALIGN(size, ALIGN4K);
            break;
        case YCbCr420SP:
            alignedw = ALIGN(width, ALIGN16);
            size = ALIGN((alignedw * height) + (ALIGN((width+1)/2, ALIGN32) * ((height+1)/2) * 2), ALIGN4K);
            break;
        case YCbCr420P:
            alignedw = ALIGN(width, ALIGN16);
            size = ALIGN((alignedw * height) + (ALIGN((width+1)/2, ALIGN16) * ((height+1)/2) * 2), ALIGN4K);
            break;
        case YCrCb420P:
            alignedw = ALIGN(width, ALIGN16);
            size = ALIGN((alignedw * height) + (ALIGN((width+1)/2, ALIGN16) * ((height+1)/2) * 2), ALIGN4K);
            break;
        case YCbCr420Tile:
            alignedw = ALIGN(width, ALIGN128);
            alignedh = ALIGN(height, ALIGN32);
            size = ALIGN(alignedw * alignedh, ALIGN8K) + ALIGN(alignedw * ALIGN((height+1)/2, ALIGN32), ALIGN8K);
            break;
        case NV12_2K: {
            alignedw = ALIGN(width, ALIGN16);
            size_t lumaSize = ALIGN(alignedw * height, ALIGN2K);
            size_t chromaSize = ALIGN((alignedw * height)/2, ALIGN2K);
            size = ALIGN(lumaSize + chromaSize, ALIGN4K);
            ALOGV("%s: NV12_2k, width = %zu, height = %zu, size = %d",
                                                   __FUNCTION__, width, height, size);
            }
            break;
        case NV12_512:
            alignedw = ALIGN(width, ALIGN512);
            alignedh = ALIGN(height, ALIGN512);
            size = ALIGN(alignedw * alignedh + (alignedw * ALIGN(height/2, ALIGN256)), ALIGN4K);
            break;
        case NV12_128m:
            alignedw = VENUS_Y_STRIDE(COLOR_FMT_NV12, width);
            alignedh = VENUS_Y_SCANLINES(COLOR_FMT_NV12, height);
            size = ALIGN(alignedw * alignedh + (alignedw * ALIGN((height+1)/2, VENUS_Y_SCANLINES(COLOR_FMT_NV12, 1)/2)), ALIGN4K);
            break;
        case NV12_UBWC:
            size = VENUS_BUFFER_SIZE(COLOR_FMT_NV12_UBWC, width, height);
            break;
        case TP10_UBWC:
            size = VENUS_BUFFER_SIZE(COLOR_FMT_NV12_BPP10_UBWC, width, height);
            break;
        case P010:
            alignedw = ALIGN(width*2, ALIGN64);
            size = (alignedw * height) + (alignedw * height / 2);
            break;
        case VENUS_P010:
            // Y plane
            alignedw = ALIGN(width*2, ALIGN256);
            alignedh = ALIGN(height, ALIGN32);
            size = (alignedw * alignedh);
            // UV plane
            alignedw = ALIGN(width*2, ALIGN256);
            alignedh = ALIGN(height/2, ALIGN16);
            size = ALIGN(size + (alignedw * alignedh), ALIGN4K);
            break;
        default:
            ALOGW("%s: Format not supported , %d", __FUNCTION__, format);
            break;
    }
    return size;
}
/*
 * Tells GPU to map given buffer and returns a physical address of mapped buffer
 */
void * C2DColorConverter::getMappedGPUAddr(int bufFD, void *bufPtr, size_t bufLen)
{
    C2D_STATUS status;
    void *gpuaddr = NULL;
    status = mC2DMapAddr(bufFD, bufPtr, bufLen, 0, KGSL_USER_MEM_TYPE_ION,
                         &gpuaddr);
    if (status != C2D_STATUS_OK) {
        ALOGE("%s: c2dMapAddr failed: status %d fd %d ptr %p len %zu flags %d",
              __FUNCTION__, status, bufFD, bufPtr, bufLen, KGSL_USER_MEM_TYPE_ION);
        return NULL;
    }
    ALOGV("%s: c2d mapping created: gpuaddr %p fd %d ptr %p len %zu",
          __FUNCTION__, gpuaddr, bufFD, bufPtr, bufLen);
    return gpuaddr;
}

bool C2DColorConverter::unmapGPUAddr(unsigned long gAddr)
{

    C2D_STATUS status = mC2DUnMapAddr((void*)gAddr);

    if (status != C2D_STATUS_OK)
        ALOGE("%s: c2dUnMapAddr failed: status %d gpuaddr %08lx",
                                     __FUNCTION__, status, gAddr);

    return (status == C2D_STATUS_OK);
}

int32_t C2DColorConverter::getBuffSize(int32_t port)
{
  if (enabled) {
    if (port == C2D_INPUT) {
      return calcSize(mSrcFormat, mSrcWidth, mSrcHeight);
    } else if (port == C2D_OUTPUT) {
      return calcSize(mDstFormat, mDstWidth, mDstHeight);
    }
  }
  return 0;
}

bool C2DColorConverter::getBuffFilledLen(int32_t port, unsigned int &filled_length)
{
  bool ret = false;
  C2DBuffReq req;
  if (enabled) {
    ret = getBuffReq(port, &req);
    if (ret) {
      filled_length = req.size;
    }
  }

  return ret;
}

bool C2DColorConverter::getBuffReq(int32_t port, C2DBuffReq *req) {
    if (!req
        || (port != C2D_INPUT
            && port != C2D_OUTPUT)) return false;

    memset(req, 0, sizeof(C2DBuffReq));
    if (port == C2D_INPUT) {
        req->width = mSrcWidth;
        req->height = mSrcHeight;
        req->stride = calcStride(mSrcFormat, mSrcWidth);
        req->sliceHeight = mSrcHeight;
        req->lumaAlign = calcLumaAlign(mSrcFormat);
        req->sizeAlign = calcSizeAlign(mSrcFormat);
        req->size = calcSize(mSrcFormat, mSrcWidth, mSrcHeight);
        req->bpp = calcBytesPerPixel(mSrcFormat);
        ALOGV("%s: input req->size = %d", __FUNCTION__, req->size);
    } else if (port == C2D_OUTPUT) {
        req->width = mDstWidth;
        req->height = mDstHeight;
        req->stride = calcStride(mDstFormat, mDstWidth);
        req->sliceHeight = mDstHeight;
        req->lumaAlign = calcLumaAlign(mDstFormat);
        req->sizeAlign = calcSizeAlign(mDstFormat);
        req->size = calcSize(mDstFormat, mDstWidth, mDstHeight);
        req->bpp = calcBytesPerPixel(mDstFormat);
        ALOGV("%s: output req->size = %d", __FUNCTION__, req->size);
    }
    return true;
}

size_t C2DColorConverter::calcLumaAlign(ColorConvertFormat format) {
    if (!isYUVSurface(format)) return 1; //no requirement

    switch (format) {
        case NV12_2K:
          return ALIGN2K;
        case NV12_512:
          return ALIGN512;
        case NV12_128m:
          return 1;
        case NV12_UBWC:
        case TP10_UBWC:
        case P010:
        case VENUS_P010:
          return ALIGN4K;
        default:
          ALOGW("%s: unknown format (%d) passed for luma alignment number.",
                                                       __FUNCTION__, format);
          return 1;
    }
}

size_t C2DColorConverter::calcSizeAlign(ColorConvertFormat format) {
    if (!isYUVSurface(format)) return 1; //no requirement

    switch (format) {
        case YCbCr420SP: //OR NV12
        case YCbCr420P:
        case NV12_2K:
        case NV12_512:
        case NV12_128m:
        case NV12_UBWC:
        case TP10_UBWC:
        case P010:
        case VENUS_P010:
          return ALIGN4K;
        default:
          ALOGW("%s: unknown format (%d) passed for size alignment number",
                                                      __FUNCTION__, format);
          return 1;
    }
}

C2DBytesPerPixel C2DColorConverter::calcBytesPerPixel(ColorConvertFormat format) {
    C2DBytesPerPixel bpp;
    bpp.numerator = 0;
    bpp.denominator = 1;

    switch (format) {
        case RGB565:
            bpp.numerator = 2;
            break;
        case RGBA8888:
        case RGBA8888_UBWC:
            bpp.numerator = 4;
            break;
        case YCbCr420SP:
        case YCbCr420P:
        case YCrCb420P:
        case YCbCr420Tile:
        case NV12_2K:
        case NV12_512:
        case NV12_128m:
        case NV12_UBWC:
        case TP10_UBWC:
        case P010:
        case VENUS_P010:
            bpp.numerator = 3;
            bpp.denominator = 2;
            break;
        default:
            ALOGW("%s: unknown format (%d) passed.", __FUNCTION__, format);
            break;
    }
    return bpp;
}

int32_t C2DColorConverter::dumpOutput(char * filename, char mode) {
    int fd;
    size_t stride, sliceHeight;
    if (!filename) return -1;

    int flags = O_RDWR | O_CREAT;
    if (mode == 'a') {
      flags |= O_APPEND;
    }

    if ((fd = open(filename, flags)) < 0) {
        ALOGE("%s: open dump file failed w/ errno %s", __FUNCTION__, strerror(errno));
        return -1;
    }

    int ret = 0;
    if (isYUVSurface(mDstFormat)) {
      C2D_YUV_SURFACE_DEF * dstSurfaceDef = (C2D_YUV_SURFACE_DEF *)mDstSurfaceDef;
      uint8_t * base = (uint8_t *)dstSurfaceDef->plane0;
      stride = dstSurfaceDef->stride0;
      sliceHeight = dstSurfaceDef->height;
      /* dump luma */
      for (size_t i = 0; i < sliceHeight; i++) {
        ret = write(fd, base, mDstWidth); //will work only for the 420 ones
        if (ret < 0) goto cleanup;
        base += stride;
      }

      if (mDstFormat == YCbCr420P ||
          mDstFormat == YCrCb420P) {
          ALOGI("%s: Dump Cb and Cr separately for Planar\n", __FUNCTION__);
          //dump Cb/Cr
          base = (uint8_t *)dstSurfaceDef->plane1;
          stride = dstSurfaceDef->stride1;
          for (size_t i = 0; i < (sliceHeight+1)/2;i++) { //will work only for the 420 ones
            ret = write(fd, base, (mDstWidth+1)/2);
            if (ret < 0) goto cleanup;
            base += stride;
          }

          //dump Cr/Cb
          base = (uint8_t *)dstSurfaceDef->plane2;
          stride = dstSurfaceDef->stride2;

          for (size_t i = 0; i < (sliceHeight+1)/2;i++) { //will work only for the 420 ones
            ret = write(fd, base, (mDstWidth+1)/2);
            if (ret < 0) goto cleanup;
            base += stride;
          }

      } else {
          /* dump chroma */
          base = (uint8_t *)dstSurfaceDef->plane1;
          stride = dstSurfaceDef->stride1;
          for (size_t i = 0; i < (sliceHeight+1)/2;i++) { //will work only for the 420 ones
            ret = write(fd, base, mDstWidth);
            if (ret < 0) goto cleanup;
            base += stride;
          }
      }
    } else {
      C2D_RGB_SURFACE_DEF * dstSurfaceDef = (C2D_RGB_SURFACE_DEF *)mDstSurfaceDef;
      uint8_t * base = (uint8_t *)dstSurfaceDef->buffer;
      stride = dstSurfaceDef->stride;
      sliceHeight = dstSurfaceDef->height;

      ALOGI("%s: rgb surface base is %p", __FUNCTION__, base);
      ALOGI("%s: rgb surface dumpsslice height is %lu\n",
                                   __FUNCTION__, (unsigned long)sliceHeight);
      ALOGI("%s: rgb surface dump stride is %lu\n",
                                   __FUNCTION__, (unsigned long)stride);

      int bpp = 1; //bytes per pixel
      if (mDstFormat == RGB565) {
        bpp = 2;
      } else if (mDstFormat == RGBA8888  || mDstFormat == RGBA8888_UBWC) {
        bpp = 4;
      }

      int count = 0;
      for (size_t i = 0; i < sliceHeight; i++) {
        ret = write(fd, base, mDstWidth*bpp);
        if (ret < 0) {
          ALOGI("%s: write failed, count = %d\n", __FUNCTION__, count);
          goto cleanup;
        }
        base += stride;
        count += stride;
      }
    }
 cleanup:
    if (ret < 0) {
      ALOGE("%s: file write failed w/ errno %s", __FUNCTION__, strerror(errno));
    }
    close(fd);
    return ret < 0 ? ret : 0;
}
