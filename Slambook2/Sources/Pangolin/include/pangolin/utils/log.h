/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// TODO: Something a bit more useful here, probably using format_string.h

#pragma once

#ifndef _ANDROID_
#   include <cstdio>
#   define pango_print_debug(...) printf(__VA_ARGS__)
#   define pango_print_info(...)  printf(__VA_ARGS__)
#   define pango_print_error(...) fprintf(stderr, __VA_ARGS__)
#   define pango_print_warn(...)  fprintf(stderr, __VA_ARGS__)
#else
#   include <android/log.h>
#   define pango_print_debug(...) __android_log_print(ANDROID_LOG_DEBUG, "pango", __VA_ARGS__ );
#   define pango_print_info(...)  __android_log_print(ANDROID_LOG_INFO,  "pango", __VA_ARGS__ );
#   define pango_print_error(...) __android_log_print(ANDROID_LOG_ERROR, "pango", __VA_ARGS__ );
#   define pango_print_warn(...)  __android_log_print(ANDROID_LOG_ERROR, "pango", __VA_ARGS__ );
#endif
