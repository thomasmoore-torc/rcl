// Copyright 2026 Torc Robotics, Inc.
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
//
// Tests for rcl/init_options_clock_source.h -- the rcl-layer half of
// bridging a registered clock type down into rmw_init_options_t's
// clock_source_data/clock_source_get_now fields (rmw/init_options.h).

#include <gtest/gtest.h>

#include "rcl/clock_type_registry.h"
#include "rcl/error_handling.h"
#include "rcl/init_options.h"
#include "rcl/init_options_clock_source.h"
#include "rmw/init_options.h"

namespace
{

rcl_time_point_value_t g_next_value = 555;
int g_init_calls = 0;

rcl_ret_t fake_init(void ** data, rcl_allocator_t * allocator)
{
  (void) allocator;
  ++g_init_calls;
  static int sentinel = 0;
  *data = &sentinel;
  return RCL_RET_OK;
}

rcl_ret_t fake_fini(void * data)
{
  (void) data;
  return RCL_RET_OK;
}

rcl_ret_t fake_get_now(void * data, rcl_time_point_value_t * now)
{
  (void) data;
  *now = g_next_value;
  return RCL_RET_OK;
}

class TestInitOptionsClockSource : public ::testing::Test
{
public:
  void SetUp() override
  {
    g_next_value = 555;
    g_init_calls = 0;
    impl_.init = fake_init;
    impl_.fini = fake_fini;
    impl_.get_now = fake_get_now;
    ASSERT_EQ(RCL_RET_OK, rcl_clock_type_register("test_init_opts_clock", &impl_, &type_));

    init_options_ = rcl_get_zero_initialized_init_options();
    ASSERT_EQ(RCL_RET_OK, rcl_init_options_init(&init_options_, rcl_get_default_allocator()));
  }

  void TearDown() override
  {
    EXPECT_EQ(RCL_RET_OK, rcl_init_options_fini(&init_options_));
    EXPECT_EQ(RCL_RET_OK, rcl_clock_type_unregister(type_));
  }

protected:
  rcl_clock_type_impl_t impl_;
  rcl_clock_type_t type_;
  rcl_init_options_t init_options_;
};

}  // namespace

TEST_F(TestInitOptionsClockSource, populates_rmw_fields_with_working_callback) {
  rcl_ret_t ret = rcl_init_options_set_clock_source(
    &init_options_, type_, rcl_get_default_allocator());
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_EQ(1, g_init_calls);

  rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options_);
  ASSERT_NE(nullptr, rmw_options);
  ASSERT_NE(nullptr, rmw_options->clock_source_get_now);
  ASSERT_NE(nullptr, rmw_options->clock_source_data);

  // The whole point: rmw-layer code with no rcl dependency can drive this
  // purely through the two plain fields, with no further lookup of its own.
  g_next_value = 987654321;
  EXPECT_EQ(987654321, rmw_options->clock_source_get_now(rmw_options->clock_source_data));
}

TEST_F(TestInitOptionsClockSource, unregistered_type_is_rejected) {
  rcl_clock_type_t bogus_type = static_cast<rcl_clock_type_t>(type_ + 100000);
  rcl_ret_t ret = rcl_init_options_set_clock_source(
    &init_options_, bogus_type, rcl_get_default_allocator());
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, ret);
  rcl_reset_error();

  rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options_);
  ASSERT_NE(nullptr, rmw_options);
  EXPECT_EQ(nullptr, rmw_options->clock_source_get_now);
}

TEST_F(TestInitOptionsClockSource, null_init_options_is_rejected) {
  rcl_ret_t ret = rcl_init_options_set_clock_source(nullptr, type_, rcl_get_default_allocator());
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, ret);
  rcl_reset_error();
}
