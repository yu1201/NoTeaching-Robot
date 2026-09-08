#pragma once

// These values are build inputs. No INI/database or branding switch may enable
// or disable license enforcement at runtime.
#ifndef HK_LICENSE_MODE
#define HK_LICENSE_MODE 0
#endif
#if HK_LICENSE_MODE < 0 || HK_LICENSE_MODE > 2
#error HK_LICENSE_MODE must be 0 (Off), 1 (Audit), or 2 (Enforce)
#endif
#ifndef HK_LICENSE_CHANNEL
#define HK_LICENSE_CHANNEL "neutral"
#endif
#ifndef HK_LICENSE_SERVER_URL
#define HK_LICENSE_SERVER_URL "https://103.217.203.52/license/api/v1"
#endif
// Supply the dedicated license verification key through the release build's
// forced-include header. Never reuse the OTA key or ship a private signing key.
#ifndef HK_LICENSE_PUBLIC_KEY_B64
#define HK_LICENSE_PUBLIC_KEY_B64 ""
#endif
#ifndef HK_LICENSE_KEY_ID
#define HK_LICENSE_KEY_ID ""
#endif
