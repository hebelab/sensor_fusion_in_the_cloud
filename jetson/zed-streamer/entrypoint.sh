#!/bin/bash

cmake .
make
./ZED_Streaming_Sender ${ZED_STREAM_PORT}