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

#include "rcl/clock_type_registry.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>

#include "rcl/error_handling.h"

// Prototype scope: a fixed-capacity array is deliberately simpler than a
// real hash map for a registry that, in practice, holds a handful of
// entries registered once near process startup (e.g. one control-time
// source per process), not a hot, high-cardinality lookup path.
#define RCL_CLOCK_TYPE_REGISTRY_MAX_ENTRIES 32

typedef struct rcl_clock_type_registry_entry_s
{
  bool in_use;
  rcl_clock_type_t type;
  char name[RCL_CLOCK_TYPE_NAME_MAX_SIZE];
  rcl_clock_type_impl_t impl;
} rcl_clock_type_registry_entry_t;

static rcl_clock_type_registry_entry_t g_registry[RCL_CLOCK_TYPE_REGISTRY_MAX_ENTRIES];
static atomic_flag g_registry_lock = ATOMIC_FLAG_INIT;
static rcl_clock_type_t g_next_type_id = RCL_CLOCK_TYPE_CUSTOM_ID_OFFSET;

// See the "Known prototype limitation" note in clock_type_registry.h: this
// spinlock is POSIX/Linux-portable C11 stdatomic, not the rcutils_atomic_*
// macros used elsewhere in this codebase for Windows portability too.
static void
rcl_clock_type_registry_lock(void)
{
  while (atomic_flag_test_and_set(&g_registry_lock)) {
    // Registration/lookup are rare, non-hot-path operations (see the
    // MAX_ENTRIES comment above); a busy-wait spin is an acceptable
    // trade-off against pulling in a new mutex abstraction not otherwise
    // used anywhere else in this codebase.
  }
}

static void
rcl_clock_type_registry_unlock(void)
{
  atomic_flag_clear(&g_registry_lock);
}

// Caller must hold the registry lock.
static rcl_clock_type_registry_entry_t *
rcl_clock_type_registry_find_by_name(const char * name)
{
  for (size_t i = 0; i < RCL_CLOCK_TYPE_REGISTRY_MAX_ENTRIES; ++i) {
    if (g_registry[i].in_use && strncmp(g_registry[i].name, name, sizeof(g_registry[i].name)) == 0)
    {
      return &g_registry[i];
    }
  }
  return NULL;
}

// Caller must hold the registry lock.
static rcl_clock_type_registry_entry_t *
rcl_clock_type_registry_find_by_id(rcl_clock_type_t type)
{
  for (size_t i = 0; i < RCL_CLOCK_TYPE_REGISTRY_MAX_ENTRIES; ++i) {
    if (g_registry[i].in_use && g_registry[i].type == type) {
      return &g_registry[i];
    }
  }
  return NULL;
}

rcl_ret_t
rcl_clock_type_register(
  const char * name,
  const rcl_clock_type_impl_t * impl,
  rcl_clock_type_t * out_type)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(name, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(impl, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(impl->init, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(impl->fini, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(impl->get_now, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(out_type, RCL_RET_INVALID_ARGUMENT);
  if ('\0' == name[0]) {
    RCL_SET_ERROR_MSG("name must not be empty");
    return RCL_RET_INVALID_ARGUMENT;
  }

  rcl_ret_t ret = RCL_RET_OK;
  rcl_clock_type_registry_lock();

  if (NULL != rcl_clock_type_registry_find_by_name(name)) {
    RCL_SET_ERROR_MSG("a clock type with this name is already registered");
    ret = RCL_RET_ERROR;
    goto unlock_and_return;
  }

  rcl_clock_type_registry_entry_t * slot = NULL;
  for (size_t i = 0; i < RCL_CLOCK_TYPE_REGISTRY_MAX_ENTRIES; ++i) {
    if (!g_registry[i].in_use) {
      slot = &g_registry[i];
      break;
    }
  }
  if (NULL == slot) {
    RCL_SET_ERROR_MSG("clock type registry is full");
    ret = RCL_RET_ERROR;
    goto unlock_and_return;
  }

  slot->type = g_next_type_id++;
  strncpy(slot->name, name, sizeof(slot->name) - 1);
  slot->name[sizeof(slot->name) - 1] = '\0';
  slot->impl = *impl;
  slot->in_use = true;

  *out_type = slot->type;

unlock_and_return:
  rcl_clock_type_registry_unlock();
  return ret;
}

rcl_ret_t
rcl_clock_type_unregister(rcl_clock_type_t type)
{
  if (type < RCL_CLOCK_TYPE_CUSTOM_ID_OFFSET) {
    RCL_SET_ERROR_MSG("type is one of the fixed rcl_clock_type_t values, not a registered one");
    return RCL_RET_INVALID_ARGUMENT;
  }

  rcl_ret_t ret = RCL_RET_OK;
  rcl_clock_type_registry_lock();

  rcl_clock_type_registry_entry_t * entry = rcl_clock_type_registry_find_by_id(type);
  if (NULL == entry) {
    RCL_SET_ERROR_MSG("type was not registered");
    ret = RCL_RET_INVALID_ARGUMENT;
    goto unlock_and_return;
  }
  entry->in_use = false;

unlock_and_return:
  rcl_clock_type_registry_unlock();
  return ret;
}

rcl_ret_t
rcl_clock_type_lookup_by_name(
  const char * name,
  rcl_clock_type_t * out_type,
  rcl_clock_type_impl_t * out_impl)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(name, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(out_type, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(out_impl, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;
  rcl_clock_type_registry_lock();

  rcl_clock_type_registry_entry_t * entry = rcl_clock_type_registry_find_by_name(name);
  if (NULL == entry) {
    RCL_SET_ERROR_MSG("name is not currently registered");
    ret = RCL_RET_INVALID_ARGUMENT;
    goto unlock_and_return;
  }
  *out_type = entry->type;
  *out_impl = entry->impl;

unlock_and_return:
  rcl_clock_type_registry_unlock();
  return ret;
}

rcl_ret_t
rcl_clock_type_lookup_by_id(
  rcl_clock_type_t type,
  rcl_clock_type_impl_t * out_impl)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(out_impl, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;
  rcl_clock_type_registry_lock();

  rcl_clock_type_registry_entry_t * entry = rcl_clock_type_registry_find_by_id(type);
  if (NULL == entry) {
    RCL_SET_ERROR_MSG("type is not currently registered");
    ret = RCL_RET_INVALID_ARGUMENT;
    goto unlock_and_return;
  }
  *out_impl = entry->impl;

unlock_and_return:
  rcl_clock_type_registry_unlock();
  return ret;
}

rcl_ret_t
rcl_clock_init_custom(
  rcl_clock_type_t custom_type,
  rcl_clock_t * clock,
  rcl_allocator_t * allocator)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(clock, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

  rcl_clock_type_impl_t impl;
  rcl_ret_t lookup_ret = rcl_clock_type_lookup_by_id(custom_type, &impl);
  if (RCL_RET_OK != lookup_ret) {
    RCL_SET_ERROR_MSG("custom_type was not registered with rcl_clock_type_register");
    return RCL_RET_INVALID_ARGUMENT;
  }

  // Mirrors rcl_init_generic_clock() (time.c, internal/static there) --
  // duplicated here rather than exposed from time.c, to keep this file's
  // only dependency on time.c's internals to the rcl_clock_t struct layout
  // itself, which is already public API.
  clock->type = RCL_CLOCK_UNINITIALIZED;
  clock->jump_callbacks = NULL;
  clock->num_jump_callbacks = 0u;
  clock->get_now = NULL;
  clock->fini = NULL;
  clock->data = NULL;
  clock->allocator = *allocator;

  rcl_ret_t ret = impl.init(&clock->data, allocator);
  if (RCL_RET_OK != ret) {
    return ret;
  }
  clock->get_now = impl.get_now;
  clock->fini = impl.fini;
  clock->type = custom_type;
  return RCL_RET_OK;
}
