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
/// Prototype implementation of the pluggable-clock-source design proposed in
/// Torc's draft REP (rep-draft-pluggable-clock-sources.rst). Lets an
/// application register a new, uniquely-identified rcl_clock_type_t backed
/// by its own get_now/init/fini implementation, built on the per-instance
/// get_now dispatch rcl_clock_t already has today (see rcl/time.h) but that
/// was, before this file, only ever populated with one of the three built-in
/// types.
///
/// Known prototype limitation: the registry lock in
/// src/rcl/clock_type_registry.c uses <stdatomic.h> directly rather than the
/// rcutils_atomic_* macros used elsewhere in this file for portability, so
/// this has only been exercised on the POSIX/Linux targets this prototype is
/// scoped to (Torc's embedded Linux AD-Kit), not on Windows/_WIN32. Porting
/// the lock to the rcutils win32 atomics shim is a follow-up if/when this is
/// pursued as a real upstream REP submission.

#ifndef RCL__CLOCK_TYPE_REGISTRY_H_
#define RCL__CLOCK_TYPE_REGISTRY_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "rcl/allocator.h"
#include "rcl/macros.h"
#include "rcl/time.h"
#include "rcl/types.h"
#include "rcl/visibility_control.h"

/// First clock type id handed out by rcl_clock_type_register().
/**
 * Values below this are reserved for rcl_clock_type_t's fixed, built-in
 * values (RCL_CLOCK_UNINITIALIZED, RCL_ROS_TIME, RCL_SYSTEM_TIME,
 * RCL_STEADY_TIME) and will never be returned by rcl_clock_type_register().
 */
#define RCL_CLOCK_TYPE_CUSTOM_ID_OFFSET 1000

/// Maximum length, including the terminating null, of a name passed to
/// rcl_clock_type_register().
#define RCL_CLOCK_TYPE_NAME_MAX_SIZE 128

/// The callbacks backing one registered, custom rcl_clock_type_t.
/**
 * `init`/`fini` mirror `get_now` in taking only an opaque `void *`, not the
 * enclosing rcl_clock_t -- so an implementation that needs its allocator
 * again at fini time (e.g. to free memory it allocated in init) needs to
 * capture a copy of it inside `*data` itself, the same way get_now's own
 * per-instance state already has to. rcl_clock_fini() still separately owns
 * generic, type-independent teardown (freeing jump_callbacks) before calling
 * `fini`, exactly as it does today for the three built-in clock types.
 */
typedef struct rcl_clock_type_impl_s
{
  /// Called once by rcl_clock_init_custom() to populate clock->data.
  /**
   * \param[out] data will be passed to every later get_now/fini call for
   *   this clock instance; this callback owns choosing what it points to
   * \param[in] allocator the allocator passed to rcl_clock_init_custom();
   *   valid only for the duration of this call -- capture a copy inside
   *   `*data` if `fini` will need it again
   */
  rcl_ret_t (* init)(void ** data, rcl_allocator_t * allocator);
  /// Called once by rcl_clock_fini() to release whatever `init` allocated.
  rcl_ret_t (* fini)(void * data);
  /// Same signature and contract as rcl_clock_t::get_now.
  rcl_ret_t (* get_now)(void * data, rcl_time_point_value_t * now);
} rcl_clock_type_impl_t;

/// Register a new, uniquely-identified clock type backed by `impl`.
/**
 * `impl` is copied into the registry; the caller does not need to keep the
 * pointer it passed in alive afterwards.
 *
 * The returned `*out_type` is valid for the remaining lifetime of the
 * process, or until explicitly unregistered with
 * rcl_clock_type_unregister() -- it is not related to, comparable with, or
 * meaningful in any other process, and must never be persisted or sent
 * across a process boundary (the same is already true of
 * builtin_interfaces/Time on the wire, which carries no clock-type tag at
 * all, before or after this).
 *
 * This function is thread-safe with itself, with
 * rcl_clock_type_unregister(), and with rcl_clock_type_lookup_by_name(), but
 * is not real-time-safe: it may block briefly on the registry lock and may
 * allocate.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | Yes
 * Uses Atomics       | Yes
 * Lock-Free          | No
 *
 * \param[in] name a process-wide-unique name identifying this clock type;
 *   truncated to RCL_CLOCK_TYPE_NAME_MAX_SIZE - 1 characters
 * \param[in] impl the implementation backing this clock type
 * \param[out] out_type the newly allocated, unique clock type id
 * \return #RCL_RET_OK if the registration succeeded, or
 * \return #RCL_RET_INVALID_ARGUMENT if any argument is invalid, or
 * \return #RCL_RET_ERROR if `name` is already registered, the registry has
 *   no free slots, or an unspecified error occurs.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_clock_type_register(
  const char * name,
  const rcl_clock_type_impl_t * impl,
  rcl_clock_type_t * out_type);

/// Unregister a previously-registered clock type.
/**
 * Does not fini any rcl_clock_t instances of this type that are still live
 * -- the caller must rcl_clock_fini() every clock of this type before
 * unregistering it, the same ordering requirement already implied by
 * rcl_ros_clock_fini()/etc. needing to run before whatever set up the
 * underlying ROS_TIME/SYSTEM_TIME/STEADY_TIME machinery tears down.
 *
 * Thread-safety: see rcl_clock_type_register().
 *
 * \param[in] type the type id, previously returned by
 *   rcl_clock_type_register()
 * \return #RCL_RET_OK if the type was unregistered, or
 * \return #RCL_RET_INVALID_ARGUMENT if `type` was never registered, or is
 *   one of the four fixed rcl_clock_type_t values.
 */
RCL_PUBLIC
rcl_ret_t
rcl_clock_type_unregister(rcl_clock_type_t type);

/// Look up a previously-registered clock type by name.
/**
 * This is the accessor a consumer *other than* the one that originally
 * called rcl_clock_type_register() uses to retrieve the same registered
 * implementation independently -- for example, an rmw implementation
 * resolving the same clock type an application already registered via
 * rclcpp, without needing a live rcl_clock_t* handed to it across that
 * layering boundary.
 *
 * Thread-safety: see rcl_clock_type_register().
 *
 * \param[in] name the name passed to rcl_clock_type_register()
 * \param[out] out_type the registered type id
 * \param[out] out_impl a copy of the registered implementation; safe to use
 *   after this call returns even if the type is later unregistered, since
 *   it is a copy, not a reference into the registry
 * \return #RCL_RET_OK if found, or
 * \return #RCL_RET_INVALID_ARGUMENT if `name` is not currently registered.
 */
RCL_PUBLIC
rcl_ret_t
rcl_clock_type_lookup_by_name(
  const char * name,
  rcl_clock_type_t * out_type,
  rcl_clock_type_impl_t * out_impl);

/// Look up a previously-registered clock type by its id.
/**
 * The by-name and by-id lookups are deliberately kept separate rather than
 * folded into one function taking either: callers with a name in hand
 * (typically an application that itself called rcl_clock_type_register())
 * and callers with only a type id (typically rcl-internal code, such as
 * rcl_clock_init_custom() and rcl_init_options_set_clock_source(), that
 * received the id from somewhere else) are different enough call sites
 * that a single dual-mode function would need its own disambiguation
 * anyway.
 *
 * Thread-safety: see rcl_clock_type_register().
 *
 * \param[in] type the type id, previously returned by
 *   rcl_clock_type_register()
 * \param[out] out_impl a copy of the registered implementation
 * \return #RCL_RET_OK if found, or
 * \return #RCL_RET_INVALID_ARGUMENT if `type` is not currently registered.
 */
RCL_PUBLIC
rcl_ret_t
rcl_clock_type_lookup_by_id(
  rcl_clock_type_t type,
  rcl_clock_type_impl_t * out_impl);

/// Initialize a clock of a previously-registered custom type.
/**
 * Mirrors rcl_ros_clock_init()/rcl_system_clock_init()/
 * rcl_steady_clock_init() for the three built-in types: calls `impl->init`
 * for the registered type and populates clock->get_now/clock->fini/
 * clock->data from it, exactly as those built-in functions populate the
 * same fields for their own type. rcl_clock_init() already calls this
 * automatically for any clock_type outside the four fixed values, so most
 * callers should just call rcl_clock_init() directly; this is exposed
 * separately for symmetry with the built-in per-type init functions, and
 * for callers that already know they want a custom type specifically.
 *
 * \param[in] custom_type must have come from rcl_clock_type_register() and
 *   still be currently registered
 * \param[in] clock the handle to the clock which is being initialized
 * \param[in] allocator the allocator to use for allocations
 * \return #RCL_RET_OK if the time source was successfully initialized, or
 * \return #RCL_RET_INVALID_ARGUMENT if any argument is invalid, or
 *   `custom_type` was never registered, or
 * \return whatever `impl->init` itself returns on failure.
 */
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_clock_init_custom(
  rcl_clock_type_t custom_type,
  rcl_clock_t * clock,
  rcl_allocator_t * allocator);

#ifdef __cplusplus
}
#endif

#endif  // RCL__CLOCK_TYPE_REGISTRY_H_
