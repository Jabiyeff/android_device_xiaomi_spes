AM_CFLAGS = -Werror
AM_CFLAGS += -Wall
AM_CFLAGS += -g -O3

AM_CPPFLAGS = -g -O3
AM_CFLAGS += "-std=c++11"
AM_CPPFLAGS += -Wno-error=unused-variable
AM_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\)


AM_CPPFLAGS += "-Dstrlcpy=g_strlcpy"
AM_CPPFLAGS += "-Dstrlcat=g_strlcat"
AM_CPPFLAGS += -Wextra
AM_CPPFLAGS += -Wno-missing-field-initializers
AM_CPPFLAGS += -fexceptions


AM_CPPFLAGS += -I$(top_srcdir)/mm-core/inc/
AM_CPPFLAGS += -I$(top_srcdir)/mm-video-v4l2/vidc/common/inc/
c_sources         := PlatformConfig.cpp
c_sources         += ConfigParser.cpp

lib_LTLIBRARIES = libplatformconfig.la
libplatformconfig_la_SOURCES = $(c_sources)
libplatformconfig_la_CFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS) -fPIC
libplatformconfig_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS) -fPIC
libplatformconfig_la_LIBADD = -lexpat -llog -lcutils -lutils
