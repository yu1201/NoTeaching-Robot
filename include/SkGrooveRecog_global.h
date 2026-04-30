#pragma once

#include <QtCore/qglobal.h>

#if defined(SKGROOVERECOG_LIBRARY)
#  define SKGROOVERECOG_EXPORT Q_DECL_EXPORT
#else
#  define SKGROOVERECOG_EXPORT
#endif
