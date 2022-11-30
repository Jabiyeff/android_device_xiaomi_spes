/*
 * Copyright (C) 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "histogram_collector.h"

void sigint_handler(int) {}

void show_usage(char *progname) {
  std::cout << "Usage: ./" + std::string(progname) + " {options} \n"
            << "Sample the V (as in HSV) channel of the pixels that were displayed onscreen.\n\n"
            << "\tOptions:\n"
            << "\t-h      display this help message\n"
            << "\t-o      write output to specified filename\n"
            << "\t-t NUM  Collect results over NUM seconds, and then exit\n"
            << "\t-m NUM  Only store the last NUM frames of statistics\n";
}

int main(int argc, char **argv) {
  struct sigaction sigHandler;
  sigHandler.sa_handler = sigint_handler;
  sigemptyset(&sigHandler.sa_mask);
  sigHandler.sa_flags = 0;
  sigaction(SIGINT, &sigHandler, NULL);

  int c;
  char *output_filename = NULL;
  int timeout = -1;
  while ((c = getopt(argc, argv, "o:t:h")) != -1) {
    switch (c) {
      case 'o':
        output_filename = optarg;
        break;
      case 't':
        timeout = strtol(optarg, NULL, 10);
        break;
      default:
      case 'h':
        show_usage(argv[0]);
        return EXIT_SUCCESS;
    }
  }

  histogram::HistogramCollector histogram;
  histogram.start();

  bool cancelled_during_wait = false;
  if (timeout > 0) {
    std::cout << "Sampling for " << timeout << " seconds.\n";
    struct timespec request, remaining;
    request.tv_sec = timeout;
    request.tv_nsec = 0;
    cancelled_during_wait = (nanosleep(&request, &remaining) != 0);
  } else {
    std::cout << "Sampling until Ctrl-C is pressed\n";
    sigsuspend(&sigHandler.sa_mask);
  }

  std::cout << "Sampling results:\n";

  histogram.stop();

  if (cancelled_during_wait) {
    std::cout << "Timed histogram collection cancelled via signal\n";
    return EXIT_SUCCESS;
  }

  if (output_filename) {
    std::cout << "\nWriting statistics to: " << output_filename << '\n';
    std::ofstream output_file;
    output_file.open(output_filename);
    if (!output_file.is_open()) {
      std::cerr << "Error, could not open given file: " << output_filename << "\n";
      return EXIT_FAILURE;
    }
    output_file << histogram.Dump();
    output_file.close();
  } else {
    std::cout << histogram.Dump() << '\n';
  }

  return EXIT_SUCCESS;
}
