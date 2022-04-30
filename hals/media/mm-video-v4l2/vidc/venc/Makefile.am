#AM_CFLAGS = -Wall
#AM_CFLAGS = -Wundef
#AM_CFLAGS += -Wstrict-prototypes
#AM_CFLAGS += -Wno-trigraphs
#AM_CFLAGS += -Wno-multichar

AM_CPPFLAGS = -D__alignx\(x\)=__attribute__\(\(__aligned__\(x\)\)\)
AM_CPPFLAGS += -D__align=__alignx
AM_CPPFLAGS += -Dinline=__inline
AM_CPPFLAGS += -DIMAGE_APPS_PROC
AM_CPPFLAGS += -DCDECL
AM_CPPFLAGS += -DT_ARM
AM_CPPFLAGS += -DNO_ARM_CLZ
AM_CPPFLAGS += -D_ANDROID_
AM_CPPFLAGS += -UENABLE_DEBUG_LOW
AM_CPPFLAGS += -DENABLE_DEBUG_HIGH
AM_CPPFLAGS += -DENABLE_DEBUG_ERROR
AM_CPPFLAGS += -UINPUT_BUFFER_LOG
AM_CPPFLAGS += -UOUTPUT_BUFFER_LOG
AM_CPPFLAGS += -Werror
AM_CPPFLAGS += -D_ANDROID_ICS_
AM_CPPFLAGS += -DUSE_ION
AM_CPPFLAGS += -Wno-enum-compare
#AM_CPPFLAGS += "-include stdint.h"
AM_CPPFLAGS += "-Dstrlcpy=g_strlcpy"
AM_CPPFLAGS += "-Dstrlcat=g_strlcat"
AM_CPPFLAGS += "-std=c++11"
AM_CPPFLAGS += -Wno-undef
AM_CPPFLAGS += -Wno-multichar
AM_CPPFLAGS += -g -O3
AM_CPPFLAGS += "-DHAVE_ANDROID_OS"
AM_CPPFLAGS += "-DUSE_CAMERA_METABUFFER_UTILS"

if USE_GLIB
AM_CPPFLAGS += -D_USE_GLIB_
endif

if TARGET_MSM8610
AM_CPPFLAGS += -D_MSM8610_
endif

if TARGETS_THAT_SUPPORT_PQ
AM_CPPFLAGS += -D_PQ_
endif

if TARGETS_THAT_USE_FLAG_MSM8226
AM_CPPFLAGS += -D_MSM8226_
endif

if TARGET_USES_MEDIA_EXTENSIONS
AM_CPPFLAGS += -DUSE_NATIVE_HANDLE_SOURCE
endif

if MASTER_SIDE_CP_TARGET_LIST
AM_CPPFLAGS += -DMASTER_SIDE_CP
endif

AM_CPPFLAGS += -I$(top_srcdir)/mm-video-v4l2/vidc/common/inc/
AM_CPPFLAGS += -I$(top_srcdir)/mm-video-v4l2/vidc/venc/inc/
AM_CPPFLAGS += -I$(top_srcdir)/libc2dcolorconvert/
AM_CPPFLAGS += -I$(top_srcdir)/mm-core/inc/
AM_CPPFLAGS += -I$(top_srcdir)/mm-core/src/common/
AM_CPPFLAGS += -I$(top_srcdir)/libstagefrighthw/
AM_CPPFLAGS += -I$(top_srcdir)/libplatformconfig/
AM_CPPFLAGS += -I$(WORKSPACE)/graphics/libdrm/freedreno/kgsl/
AM_CPPFLAGS += -I$(WORKSPACE)/display/display-hal/gralloc/
AM_CPPFLAGS += -I$(WORKSPACE)/display/display-hal/libqdutils/
AM_CPPFLAGS += -I$(WORKSPACE)/hardware/libhardware/include/
AM_CPPFLAGS += -I${WORKSPACE}/frameworks/native/include/
sources = src/omx_video_base.cpp
sources += src/omx_video_encoder.cpp
sources += src/video_encoder_device_v4l2.cpp
sources += $(top_srcdir)/mm-video-v4l2/vidc/common/src/vidc_common.cpp
sources += $(top_srcdir)/mm-video-v4l2/vidc/common/src/vidc_vendor_extensions.cpp

lib_LTLIBRARIES = libOmxVenc.la
libOmxVenc_la_SOURCES = $(sources)
libOmxVenc_la_CFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS) -fPIC
libOmxVenc_la_LDFLAGS = -module -shared -avoid-version
libOmxVenc_la_LDFLAGS += -lstdc++ -lpthread -llog -lutils -lcutils -lglib-2.0 -lbase -ldl -lpthread -lion

libOmxVenc_la_LDFLAGS += -L$(top_builddir)/libc2dcolorconvert  -L$(top_builddir)/libplatformconfig
libOmxVenc_la_LDFLAGS += -lc2dcolorconvert -lplatformconfig -larbitrarybytes
