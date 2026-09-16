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

/// @file
///
/// Torc-specific addition, not part of upstream rcl: the rcl-layer half of
/// bridging a registered clock type (rcl/clock_type_registry.h) down into
/// rmw_init_options_t's clock_source_data/clock_source_get_now fields
/// (rmw/init_options.h). Kept in a separate header/source file from
/// init_options.h/.c specifically so the diff against those two upstream
/// files stays empty -- everything this needs from them is already public
/// API (rcl_init_options_get_rmw_init_options()).
///
/// This exists at the rcl layer, not the rmw layer, because resolving a
/// registered clock type is an rcl-layer operation (rcl/clock_type_registry.h)
/// and rmw implementations sit below rcl in this stack's dependency graph
/// -- they must not call back up into it. See rmw/init_options.h's own
/// comment on these two fields for the fuller version of this rationale.

#ifndef RCL__INIT_OPTIONS_CLOCK_SOURCE_H_
#define RCL__INIT_OPTIONS_CLOCK_SOURCE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "rcl/allocator.h"
#include "rcl/init_options.h"
#include "rcl/time.h"
#include "rcl/types.h"
#include "rcl/visibility_control.h"

/// Resolve `clock_type` and populate `init_options`'s underlying
/// rmw_init_options_t's clock_source_data/clock_source_get_now fields from
/// it, for any rmw implementation that supports
/// RMW_FEATURE_CUSTOM_CLOCK_SOURCE (rmw/features.h) to pick up.
/**
 * Constructs its own instance of `clock_type` (its own call to the
 * registered implementation's init(), e.g. its own open file descriptor if
 * the implementation needs one) -- independent of, and not shared with,
 * any rcl_clock_t (e.g. an rclcpp::Clock) already using the same
 * registered type in this process. See the pluggable-clock-source REP
 * draft's Rationale section (rep-drafts/rep-draft-pluggable-clock-sources.rst
 * in torc-compute-foundation/adr) for why: avoiding a live object crossing
 * the rmw vendor boundary, at the cost of one extra instance.
 *
 * Known prototype limitation, not yet addressed: nothing currently calls
 * the constructed instance's fini() or frees the small wrapper this
 * allocates -- rmw_init_options_t's own fini() is vendor-implemented and
 * has no way to know about this rcl-layer construct. Since an
 * rcl_init_options_t is typically created once and lives for the duration
 * of a process's rcl_context_t, this is a bounded, once-per-process leak
 * rather than a per-message one, but it would need real ownership wiring
 * before this is more than a prototype.
 *
 * \param[in] init_options must already be initialized (rcl_init_options_init()
 *   already called on it)
 * \param[in] clock_type must have come from rcl_clock_type_register() and
 *   still be currently registered
 * \param[in] allocator the allocator to use for this call's own allocations
 *   (the constructed clock instance and its wrapper); independent of
 *   whatever allocator `init_options` itself was created with
 * \return #RCL_RET_OK if the clock source was successfully resolved and
 *   set, or
 * \return #RCL_RET_INVALID_ARGUMENT if any argument is invalid, or
 *   `clock_type` was never registered, or
 * \return #RCL_RET_ERROR if the registered implementation's own init()
 *   callback failed, or the underlying rmw_init_options_t could not be
 *   retrieved.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_init_options_set_clock_source(
  rcl_init_options_t * init_options,
  rcl_clock_type_t clock_type,
  rcl_allocator_t allocator);

#ifdef __cplusplus
}
#endif

#endif  // RCL__INIT_OPTIONS_CLOCK_SOURCE_H_
