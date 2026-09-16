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
// Tests for rcl/clock_type_registry.h -- the prototype implementation of
// the pluggable-clock-source design proposed in Torc's draft REP
// (rep-draft-pluggable-clock-sources.rst, torc-compute-foundation/adr).

#include <gtest/gtest.h>

#include "rcl/clock_type_registry.h"
#include "rcl/error_handling.h"
#include "rcl/time.h"

namespace
{

int g_init_calls = 0;
int g_fini_calls = 0;
int g_get_now_calls = 0;
rcl_time_point_value_t g_next_value = 1000;

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
  ++g_fini_calls;
  return RCL_RET_OK;
}

rcl_ret_t fake_get_now(void * data, rcl_time_point_value_t * now)
{
  (void) data;
  ++g_get_now_calls;
  *now = g_next_value;
  return RCL_RET_OK;
}

rcl_ret_t failing_init(void ** data, rcl_allocator_t * allocator)
{
  (void) data;
  (void) allocator;
  return RCL_RET_ERROR;
}

class TestClockTypeRegistry : public ::testing::Test
{
public:
  void SetUp() override
  {
    g_init_calls = 0;
    g_fini_calls = 0;
    g_get_now_calls = 0;
    g_next_value = 1000;
    impl_.init = fake_init;
    impl_.fini = fake_fini;
    impl_.get_now = fake_get_now;
  }

protected:
  rcl_clock_type_impl_t impl_;
};

}  // namespace

TEST_F(TestClockTypeRegistry, register_returns_id_above_reserved_range) {
  rcl_clock_type_t type;
  rcl_ret_t ret = rcl_clock_type_register("test_clock_basic", &impl_, &type);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_GE(type, RCL_CLOCK_TYPE_CUSTOM_ID_OFFSET);

  EXPECT_EQ(RCL_RET_OK, rcl_clock_type_unregister(type));
}

TEST_F(TestClockTypeRegistry, duplicate_name_is_rejected) {
  rcl_clock_type_t type1;
  ASSERT_EQ(RCL_RET_OK, rcl_clock_type_register("test_clock_dup", &impl_, &type1));

  rcl_clock_type_t type2;
  rcl_ret_t ret = rcl_clock_type_register("test_clock_dup", &impl_, &type2);
  EXPECT_EQ(RCL_RET_ERROR, ret);
  rcl_reset_error();

  EXPECT_EQ(RCL_RET_OK, rcl_clock_type_unregister(type1));
}

TEST_F(TestClockTypeRegistry, lookup_by_name_and_by_id_agree) {
  rcl_clock_type_t type;
  ASSERT_EQ(RCL_RET_OK, rcl_clock_type_register("test_clock_lookup", &impl_, &type));

  rcl_clock_type_t looked_up_type;
  rcl_clock_type_impl_t looked_up_impl;
  ASSERT_EQ(
    RCL_RET_OK,
    rcl_clock_type_lookup_by_name("test_clock_lookup", &looked_up_type, &looked_up_impl));
  EXPECT_EQ(type, looked_up_type);
  EXPECT_EQ(impl_.get_now, looked_up_impl.get_now);

  rcl_clock_type_impl_t by_id_impl;
  ASSERT_EQ(RCL_RET_OK, rcl_clock_type_lookup_by_id(type, &by_id_impl));
  EXPECT_EQ(impl_.get_now, by_id_impl.get_now);

  EXPECT_EQ(RCL_RET_OK, rcl_clock_type_unregister(type));
}

TEST_F(TestClockTypeRegistry, unregistered_name_and_id_are_not_found) {
  rcl_clock_type_t out_type;
  rcl_clock_type_impl_t out_impl;
  EXPECT_EQ(
    RCL_RET_INVALID_ARGUMENT,
    rcl_clock_type_lookup_by_name("test_clock_never_registered", &out_type, &out_impl));
  rcl_reset_error();

  EXPECT_EQ(
    RCL_RET_INVALID_ARGUMENT,
    rcl_clock_type_lookup_by_id(
      static_cast<rcl_clock_type_t>(RCL_CLOCK_TYPE_CUSTOM_ID_OFFSET + 99999), &out_impl));
  rcl_reset_error();
}

TEST_F(TestClockTypeRegistry, unregister_then_lookup_fails) {
  rcl_clock_type_t type;
  ASSERT_EQ(RCL_RET_OK, rcl_clock_type_register("test_clock_unreg", &impl_, &type));
  ASSERT_EQ(RCL_RET_OK, rcl_clock_type_unregister(type));

  rcl_clock_type_impl_t out_impl;
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rcl_clock_type_lookup_by_id(type, &out_impl));
  rcl_reset_error();
}

TEST_F(TestClockTypeRegistry, unregister_of_builtin_type_is_rejected) {
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rcl_clock_type_unregister(RCL_ROS_TIME));
  rcl_reset_error();
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rcl_clock_type_unregister(RCL_SYSTEM_TIME));
  rcl_reset_error();
}

// End-to-end: rcl_clock_init() (the actual, real call site every consumer
// uses) delegates to rcl_clock_init_custom() for a registered type, and the
// resulting rcl_clock_t behaves like any other clock -- same get_now/fini
// call paths as the three built-in types.
TEST_F(TestClockTypeRegistry, rcl_clock_init_uses_registered_type_end_to_end) {
  rcl_clock_type_t type;
  ASSERT_EQ(RCL_RET_OK, rcl_clock_type_register("test_clock_e2e", &impl_, &type));

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ASSERT_EQ(RCL_RET_OK, rcl_clock_init(type, &clock, &allocator)) << rcl_get_error_string().str;
  EXPECT_EQ(1, g_init_calls);
  EXPECT_EQ(type, clock.type);

  g_next_value = 42424242;
  rcl_time_point_value_t now = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_clock_get_now(&clock, &now));
  EXPECT_EQ(42424242, now);
  EXPECT_EQ(1, g_get_now_calls);

  EXPECT_EQ(RCL_RET_OK, rcl_clock_fini(&clock));
  EXPECT_EQ(1, g_fini_calls);

  EXPECT_EQ(RCL_RET_OK, rcl_clock_type_unregister(type));
}

TEST_F(TestClockTypeRegistry, rcl_clock_init_propagates_impl_init_failure) {
  rcl_clock_type_impl_t failing_impl = impl_;
  failing_impl.init = failing_init;

  rcl_clock_type_t type;
  ASSERT_EQ(RCL_RET_OK, rcl_clock_type_register("test_clock_init_fail", &failing_impl, &type));

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  EXPECT_EQ(RCL_RET_ERROR, rcl_clock_init(type, &clock, &allocator));
  rcl_reset_error();

  EXPECT_EQ(RCL_RET_OK, rcl_clock_type_unregister(type));
}

TEST_F(TestClockTypeRegistry, register_rejects_null_arguments) {
  rcl_clock_type_t type;
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rcl_clock_type_register(nullptr, &impl_, &type));
  rcl_reset_error();
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rcl_clock_type_register("name", nullptr, &type));
  rcl_reset_error();
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rcl_clock_type_register("name", &impl_, nullptr));
  rcl_reset_error();
}

TEST_F(TestClockTypeRegistry, register_rejects_empty_name) {
  rcl_clock_type_t type;
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rcl_clock_type_register("", &impl_, &type));
  rcl_reset_error();
}
