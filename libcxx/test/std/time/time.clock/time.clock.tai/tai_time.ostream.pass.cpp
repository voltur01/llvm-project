//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

// REQUIRES: std-at-least-c++20
// UNSUPPORTED: no-filesystem, no-localization, no-tzdb
// UNSUPPORTED: GCC-ALWAYS_INLINE-FIXME

// TODO FMT This test should not require std::to_chars(floating-point)
// XFAIL: availability-fp_to_chars-missing

// XFAIL: libcpp-has-no-experimental-tzdb
// XFAIL: availability-tzdb-missing

// REQUIRES: locale.fr_FR.UTF-8
// REQUIRES: locale.ja_JP.UTF-8

// <chrono>

// class tai_clock;

// template<class charT, class traits, class Duration>
//   basic_ostream<charT, traits>&
//     operator<<(basic_ostream<charT, traits>& os, const tai_time<Duration>& tp);

#include <chrono>
#include <cassert>
#include <ratio>
#include <sstream>

#include "make_string.h"
#include "platform_support.h" // locale name macros
#include "test_macros.h"

#define SV(S) MAKE_STRING_VIEW(CharT, S)

template <class CharT, class Duration>
static std::basic_string<CharT> stream_c_locale(std::chrono::tai_time<Duration> time_point) {
  std::basic_stringstream<CharT> sstr;
  sstr << std::fixed << time_point;
  return sstr.str();
}

template <class CharT, class Duration>
static std::basic_string<CharT> stream_fr_FR_locale(std::chrono::tai_time<Duration> time_point) {
  std::basic_stringstream<CharT> sstr;
  const std::locale locale(LOCALE_fr_FR_UTF_8);
  sstr.imbue(locale);
  sstr << std::fixed << time_point;
  return sstr.str();
}

template <class CharT, class Duration>
static std::basic_string<CharT> stream_ja_JP_locale(std::chrono::tai_time<Duration> time_point) {
  std::basic_stringstream<CharT> sstr;
  const std::locale locale(LOCALE_ja_JP_UTF_8);
  sstr.imbue(locale);
  sstr << std::fixed << time_point;
  return sstr.str();
}

template <class CharT>
static void test_c() {
  using namespace std::literals::chrono_literals;
  namespace cr = std::chrono;

  assert(stream_c_locale<CharT>(cr::tai_time<cr::nanoseconds>{946'688'523'123'456'789ns}) ==
         SV("1988-01-01 01:02:03.123456789"));
  assert(stream_c_locale<CharT>(cr::tai_time<cr::microseconds>{946'688'523'123'456us}) ==
         SV("1988-01-01 01:02:03.123456"));

  assert(stream_c_locale<CharT>(cr::tai_time<cr::milliseconds>{946'684'822'123ms}) == SV("1988-01-01 00:00:22.123"));
  assert(stream_c_locale<CharT>(cr::tai_seconds{1'234'567'890s}) == SV("1997-02-13 23:31:30"));
  assert(stream_c_locale<CharT>(cr::tai_time<cr::minutes>{20'576'131min}) == SV("1997-02-13 23:31:00"));
  assert(stream_c_locale<CharT>(cr::tai_time<cr::hours>{342'935h}) == SV("1997-02-13 23:00:00"));

  assert(stream_c_locale<CharT>(cr::tai_time<cr::duration<signed char, std::ratio<2, 1>>>{
             cr::duration<signed char, std::ratio<2, 1>>{60}}) == SV("1958-01-01 00:02:00"));
  assert(stream_c_locale<CharT>(cr::tai_time<cr::duration<short, std::ratio<1, 2>>>{
             cr::duration<short, std::ratio<1, 2>>{3600}}) == SV("1958-01-01 00:30:00.0"));
  assert(stream_c_locale<CharT>(cr::tai_time<cr::duration<int, std::ratio<1, 4>>>{
             cr::duration<int, std::ratio<1, 4>>{3600}}) == SV("1958-01-01 00:15:00.00"));
  assert(stream_c_locale<CharT>(cr::tai_time<cr::duration<long, std::ratio<1, 10>>>{
             cr::duration<long, std::ratio<1, 10>>{36611}}) == SV("1958-01-01 01:01:01.1"));
  assert(stream_c_locale<CharT>(cr::tai_time<cr::duration<long long, std::ratio<1, 100>>>{
             cr::duration<long long, std::ratio<1, 100>>{12'345'678'9010}}) == SV("1997-02-13 23:31:30.10"));
}

template <class CharT>
static void test_fr_FR() {
  using namespace std::literals::chrono_literals;
  namespace cr = std::chrono;

  assert(stream_fr_FR_locale<CharT>(cr::tai_time<cr::nanoseconds>{946'688'523'123'456'789ns}) ==
         SV("1988-01-01 01:02:03,123456789"));
  assert(stream_fr_FR_locale<CharT>(cr::tai_time<cr::microseconds>{946'688'523'123'456us}) ==
         SV("1988-01-01 01:02:03,123456"));

  assert(stream_fr_FR_locale<CharT>(cr::tai_time<cr::milliseconds>{946'684'822'123ms}) ==
         SV("1988-01-01 00:00:22,123"));
  assert(stream_fr_FR_locale<CharT>(cr::tai_seconds{1'234'567'890s}) == SV("1997-02-13 23:31:30"));
  assert(stream_fr_FR_locale<CharT>(cr::tai_time<cr::minutes>{20'576'131min}) == SV("1997-02-13 23:31:00"));
  assert(stream_fr_FR_locale<CharT>(cr::tai_time<cr::hours>{342'935h}) == SV("1997-02-13 23:00:00"));

  assert(stream_fr_FR_locale<CharT>(cr::tai_time<cr::duration<signed char, std::ratio<2, 1>>>{
             cr::duration<signed char, std::ratio<2, 1>>{60}}) == SV("1958-01-01 00:02:00"));
  assert(stream_fr_FR_locale<CharT>(cr::tai_time<cr::duration<short, std::ratio<1, 2>>>{
             cr::duration<short, std::ratio<1, 2>>{3600}}) == SV("1958-01-01 00:30:00,0"));
  assert(stream_fr_FR_locale<CharT>(cr::tai_time<cr::duration<int, std::ratio<1, 4>>>{
             cr::duration<int, std::ratio<1, 4>>{3600}}) == SV("1958-01-01 00:15:00,00"));
  assert(stream_fr_FR_locale<CharT>(cr::tai_time<cr::duration<long, std::ratio<1, 10>>>{
             cr::duration<long, std::ratio<1, 10>>{36611}}) == SV("1958-01-01 01:01:01,1"));
  assert(stream_fr_FR_locale<CharT>(cr::tai_time<cr::duration<long long, std::ratio<1, 100>>>{
             cr::duration<long long, std::ratio<1, 100>>{12'345'678'9010}}) == SV("1997-02-13 23:31:30,10"));
}

template <class CharT>
static void test_ja_JP() {
  using namespace std::literals::chrono_literals;
  namespace cr = std::chrono;

  assert(stream_ja_JP_locale<CharT>(cr::tai_time<cr::nanoseconds>{946'688'523'123'456'789ns}) ==
         SV("1988-01-01 01:02:03.123456789"));
  assert(stream_ja_JP_locale<CharT>(cr::tai_time<cr::microseconds>{946'688'523'123'456us}) ==
         SV("1988-01-01 01:02:03.123456"));

  assert(stream_ja_JP_locale<CharT>(cr::tai_time<cr::milliseconds>{946'684'822'123ms}) ==
         SV("1988-01-01 00:00:22.123"));
  assert(stream_ja_JP_locale<CharT>(cr::tai_seconds{1'234'567'890s}) == SV("1997-02-13 23:31:30"));
  assert(stream_ja_JP_locale<CharT>(cr::tai_time<cr::minutes>{20'576'131min}) == SV("1997-02-13 23:31:00"));
  assert(stream_ja_JP_locale<CharT>(cr::tai_time<cr::hours>{342'935h}) == SV("1997-02-13 23:00:00"));

  assert(stream_ja_JP_locale<CharT>(cr::tai_time<cr::duration<signed char, std::ratio<2, 1>>>{
             cr::duration<signed char, std::ratio<2, 1>>{60}}) == SV("1958-01-01 00:02:00"));
  assert(stream_ja_JP_locale<CharT>(cr::tai_time<cr::duration<short, std::ratio<1, 2>>>{
             cr::duration<short, std::ratio<1, 2>>{3600}}) == SV("1958-01-01 00:30:00.0"));
  assert(stream_ja_JP_locale<CharT>(cr::tai_time<cr::duration<int, std::ratio<1, 4>>>{
             cr::duration<int, std::ratio<1, 4>>{3600}}) == SV("1958-01-01 00:15:00.00"));
  assert(stream_ja_JP_locale<CharT>(cr::tai_time<cr::duration<long, std::ratio<1, 10>>>{
             cr::duration<long, std::ratio<1, 10>>{36611}}) == SV("1958-01-01 01:01:01.1"));
  assert(stream_ja_JP_locale<CharT>(cr::tai_time<cr::duration<long long, std::ratio<1, 100>>>{
             cr::duration<long long, std::ratio<1, 100>>{12'345'678'9010}}) == SV("1997-02-13 23:31:30.10"));
}

template <class CharT>
static void test() {
  test_c<CharT>();
  test_fr_FR<CharT>();
  test_ja_JP<CharT>();
}

int main(int, char**) {
  test<char>();

#ifndef TEST_HAS_NO_WIDE_CHARACTERS
  test<wchar_t>();
#endif

  return 0;
}
