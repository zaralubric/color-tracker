
dnl Autoconf file for building ColorTracker codels library.

AC_PREREQ(2.59)

AC_INIT([ColorTracker-genom3],[1.0],[shasthamsa@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acColorTracker.h])
AM_INIT_AUTOMAKE([foreign no-define])

dnl Compilers
dnl
LT_INIT([disable-static])
AC_PROG_CC
AC_PROG_CXX


dnl Require GNU make
AC_CACHE_CHECK([for GNU make], [ac_cv_path_MAKE],
  [AC_PATH_PROGS_FEATURE_CHECK([MAKE], [make gmake],
    [case `$ac_path_MAKE --version 2>/dev/null` in
       *GNU*) ac_cv_path_MAKE=$ac_path_MAKE; ac_path_MAKE_found=:;;
     esac],
    [AC_MSG_ERROR([could not find GNU make])])])
AC_SUBST([MAKE], [$ac_cv_path_MAKE])


dnl External packages
PKG_CHECK_MODULES(requires, [
  genom3 >= 2.99.4
  openrobots2-idl >= 2.1
  felix-idl
  vision-idl
])
PKG_CHECK_MODULES(codels_requires, [
  opencv4, felix-g3utils, eigen3, vision-idl
])

AC_PATH_PROG(GENOM3, [genom3], [no])
if test "$GENOM3" = "no"; then
  AC_MSG_ERROR([genom3 tool not found], 2)
fi

dnl --with-templates option
AG_OPT_TEMPLATES([$GENOM3 ],
    [$srcdir/ColorTracker.gen])

dnl Doc
AM_MISSING_PROG([ASCIIDOCTOR], [asciidoctor])

dnl Output
AC_CONFIG_FILES([
	ColorTracker-genom3.pc
	ColorTracker-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES
