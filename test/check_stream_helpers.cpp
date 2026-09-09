// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Checks the vector<bool> stream helpers that resource/interface_factories.cpp.em
// embeds in every generated factory. They cannot use the memcpy path the other
// primitive vectors use (vector<bool> has no contiguous bool storage), so they
// copy element by element -- which only stays inside the buffer if advance(),
// the call that bounds-checks, runs before the copy rather than after it.
//
// Driven by check_stream_helpers.sh, which extracts the helpers from the .em
// template so this exercises the shipped text and not a copy of it.

#include <ros/serialization.h>

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "stream_helpers.inc"

namespace rs = ros::serialization;

// 4-byte length prefix + one byte per element, which is what the generated
// code writes: streamVectorSize() followed by streamPrimitiveVectorBool().
static uint32_t wire_size(size_t elements)
{
  return static_cast<uint32_t>(4 + elements);
}

static void check_round_trip()
{
  const std::vector<bool> in{true, false, true, true, false};
  std::vector<uint8_t> buffer(wire_size(in.size()));

  rs::OStream out(buffer.data(), static_cast<uint32_t>(buffer.size()));
  ros1_bridge::streamVectorSize(out, in);
  ros1_bridge::streamPrimitiveVectorBool(out, in);

  std::vector<bool> back;
  rs::IStream is(buffer.data(), static_cast<uint32_t>(buffer.size()));
  ros1_bridge::streamVectorSize(is, back);
  ros1_bridge::streamPrimitiveVectorBool(is, back);

  assert(back == in);
}

// A ROS 1 message whose length prefix promises more elements than the buffer
// holds. The helper must reject it, and must not have read the missing bytes
// on the way to finding out (run under -fsanitize=address).
static void check_truncated_read_throws()
{
  std::vector<uint8_t> buffer(4);
  const uint32_t promised = 1024 * 1024;
  memcpy(buffer.data(), &promised, sizeof(promised));

  std::vector<bool> back;
  rs::IStream is(buffer.data(), static_cast<uint32_t>(buffer.size()));
  ros1_bridge::streamVectorSize(is, back);
  assert(back.size() == promised);

  bool threw = false;
  try {
    ros1_bridge::streamPrimitiveVectorBool(is, back);
  } catch (const rs::StreamOverrunException &) {
    threw = true;
  }
  assert(threw);
}

// The mirror case on the write side: a buffer too small for the vector must
// throw instead of being written past its end.
static void check_short_write_throws()
{
  const std::vector<bool> in{true, true, true, true, true};
  std::vector<uint8_t> buffer(wire_size(in.size()) - 2);

  rs::OStream out(buffer.data(), static_cast<uint32_t>(buffer.size()));
  ros1_bridge::streamVectorSize(out, in);

  bool threw = false;
  try {
    ros1_bridge::streamPrimitiveVectorBool(out, in);
  } catch (const rs::StreamOverrunException &) {
    threw = true;
  }
  assert(threw);
}

int main()
{
  check_round_trip();
  check_truncated_read_throws();
  check_short_write_throws();
  printf("stream helpers: round trip, truncated read and short write all OK\n");
  return 0;
}
