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

#include "rcl/init_options_clock_source.h"

#include <stdint.h>

#include "rcl/clock_type_registry.h"
#include "rcl/error_handling.h"
#include "rmw/init_options.h"

// Bundles a specific registered clock instance's own get_now/data pair so
// one, single, non-templated trampoline below can serve every call to this
// function regardless of which clock_type was resolved -- unlike
// rclcpp::experimental::register_clock_source()'s trampoline problem (see
// that file's own comment), this doesn't need a compile-time-generated
// pool of distinct function pointers, because *we* control what
// clock_source_data points to (we're the ones calling impl.init() and
// setting the field, right here), so the dispatch info can simply travel
// inside the heap-allocated data itself.
typedef struct rcl_clock_source_wrapper_s
{
  void * inner_data;
  rcl_ret_t (* inner_get_now)(void * data, rcl_time_point_value_t * now);
} rcl_clock_source_wrapper_t;

static int64_t
rcl_clock_source_get_now_trampoline(void * data)
{
  rcl_clock_source_wrapper_t * wrapper = (rcl_clock_source_wrapper_t *) data;
  rcl_time_point_value_t now = 0;
  // Best-effort, matching rmw_fastrtps_shared_cpp's own trampoline: this
  // signature (matching rmw_init_options_t.clock_source_get_now) has no
  // error-return path, and per-call logging on failure here would risk
  // flooding logs on what should be a rare, near-startup path becoming a
  // steady-state one instead.
  wrapper->inner_get_now(wrapper->inner_data, &now);
  return now;
}

rcl_ret_t
rcl_init_options_set_clock_source(
  rcl_init_options_t * init_options,
  rcl_clock_type_t clock_type,
  rcl_allocator_t allocator)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(init_options, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ALLOCATOR_WITH_MSG(&allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

  rcl_clock_type_impl_t impl;
  // rcl_clock_type_lookup_by_id() already sets its own error message and
  // returns RCL_RET_INVALID_ARGUMENT on failure -- propagate that directly
  // rather than setting a second message on top of it (RCL_SET_ERROR_MSG
  // while one is already set triggers rcutils's "overwriting unset error"
  // warning, since it assumes the caller meant to reset first).
  rcl_ret_t lookup_ret = rcl_clock_type_lookup_by_id(clock_type, &impl);
  if (RCL_RET_OK != lookup_ret) {
    return lookup_ret;
  }

  rcl_clock_source_wrapper_t * wrapper =
    (rcl_clock_source_wrapper_t *)allocator.allocate(sizeof(rcl_clock_source_wrapper_t),
    allocator.state);
  if (NULL == wrapper) {
    RCL_SET_ERROR_MSG("allocating clock source wrapper failed");
    return RCL_RET_BAD_ALLOC;
  }

  rcl_ret_t init_ret = impl.init(&wrapper->inner_data, &allocator);
  if (RCL_RET_OK != init_ret) {
    allocator.deallocate(wrapper, allocator.state);
    return init_ret;
  }
  wrapper->inner_get_now = impl.get_now;

  rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(init_options);
  if (NULL == rmw_options) {
    // impl.fini() is intentionally not called here on this error path --
    // see the "Known prototype limitation" note in
    // rcl/init_options_clock_source.h; this function doesn't attempt to
    // tear down what it constructs on any path, success or failure.
    allocator.deallocate(wrapper, allocator.state);
    RCL_SET_ERROR_MSG("failed to retrieve the underlying rmw_init_options_t");
    return RCL_RET_ERROR;
  }

  rmw_options->clock_source_data = wrapper;
  rmw_options->clock_source_get_now = rcl_clock_source_get_now_trampoline;
  return RCL_RET_OK;
}
