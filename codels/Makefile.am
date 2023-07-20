
lib_LTLIBRARIES = libColorTracker_codels.la

libColorTracker_codels_la_SOURCES  =	ColorTracker_c_types.h
libColorTracker_codels_la_SOURCES +=	ColorTracker_codels.cc
libColorTracker_codels_la_SOURCES +=	ColorTracker_track_codels.cc

libColorTracker_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libColorTracker_codels_la_LIBADD   =	$(requires_LIBS)
libColorTracker_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libColorTracker_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libColorTracker_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	ColorTracker_c_types.h
CLEANFILES=	${BUILT_SOURCES}

ColorTracker_c_types.h: ${top_srcdir}/ColorTracker.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/ColorTracker_c_types.h.d
