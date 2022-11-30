header_sources = display_config.h

cpp_sources = qd_utils.cpp \
              display_config.cpp

qdutils_includedir = $(pkgincludedir)/display
qdutils_include_HEADERS = $(header_sources)

lib_LTLIBRARIES = libqdutils.la
libqdutils_la_CC = @CC@
libqdutils_la_SOURCES = $(c_sources)
libqdutils_la_CFLAGS = $(COMMON_CFLAGS) -DLOG_TAG=\"qdutils\"
libqdutils_la_CPPFLAGS = $(AM_CPPFLAGS)
libqdutils_LDADD = -lhardware -lcutils -llog -lbinder
libqdutils_la_LIBADD = ../libqservice/libqservice.la
libqdutils_la_LDFLAGS = -shared -avoid-version
