#pragma once

// Copyright 2012, 2017 joydeepb@ri.cmu.edu
// Robotics Institute, Carnegie Mellon University
//
// Shared Utilities Library
//
//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================

#include <execinfo.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <string>
#include <vector>

namespace util {
namespace crash {

std::string ExecuteCommand(const char* cmd) {
  FILE* pipe = popen(cmd, "r");
  if (!pipe) return "ERROR";
  char buffer[128];
  std::string result = "";
  while (!feof(pipe)) {
    if (fgets(buffer, sizeof(buffer), pipe) != NULL) {
      result += buffer;
    }
  }
  pclose(pipe);
  return result;
}

void PrintStackTrace(FILE* file = stdout) {
  static const int kMaxTraceDepth = 256;
  static const int kMaxStringLength = 4096;
  void* trace[kMaxTraceDepth];
  char str[kMaxStringLength];

  time_t t = time(NULL);
  struct tm time;
  localtime_r(&t, &time);
  strftime(str, sizeof(str), "Stack Trace at %F %H:%M:%S:", &time);
  puts(str);

  const int trace_size = backtrace(trace, kMaxTraceDepth);

  // Execute addr2line once with all addresses.
  snprintf(str, kMaxStringLength, "addr2line -pfCe /proc/%d/exe", getpid());
  for (int i = 1; i < trace_size; ++i) {
    snprintf(str + strlen(str), sizeof(str) - strlen(str), " %p", trace[i]);
  }
  std::string s = ExecuteCommand(str);
  // Ensure we don't miss the last line in case it somehow doesn't have a
  // newline.
  s.push_back('\n');

  // Parse output lines.
  std::vector<std::pair<std::string, std::string> > lines;
  size_t maxlen = 0;
  size_t pos1 = 0, pos2 = s.find('\n');
  while (pos2 != std::string::npos) {
    std::string line = s.substr(pos1, pos2 - pos1);
    size_t n = line.find(" at ");
    if (n != std::string::npos) {
      std::pair<std::string, std::string> p;
      p.first = line.substr(0, n);
      p.second = line.substr(n + 4);
      maxlen = std::max(maxlen, n);
      lines.push_back(p);
      if (p.first == "main") break;
    }
    pos1 = pos2 + 1;
    pos2 = s.find('\n', pos1);
  }

  // Compute how many maximum digits are required to print the backtrace steps.
  int num_digits = 1;
  int n = lines.size();
  while (n /= 10) {
    num_digits++;
  }

  for (size_t i = 0; i < lines.size(); ++i) {
    printf("%*d: \x1b[33m%-*s\x1b[m \x1b[32m%s\x1b[m\n", num_digits,
           static_cast<int>(i), static_cast<int>(maxlen),
           lines[i].first.c_str(), lines[i].second.c_str());
  }
}

void FatalSignalHandler(int signo) {
  fprintf(stderr, "Received fatal signal %s\n", strsignal(signo));
  fflush(stderr);
  PrintStackTrace();
  exit(1);
}

}  // namespace crash
}  // namespace util