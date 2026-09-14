// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/src/Core/util/LruCache.h>

using Eigen::internal::LruCache;

// Move-only value with a side-effecting destructor, used to assert that
// eviction and clear() actually destroy values.
struct DestructorTracer {
  int id;
  std::vector<int>* sink;

  DestructorTracer(int id_, std::vector<int>* sink_) : id(id_), sink(sink_) {}
  DestructorTracer(const DestructorTracer&) = delete;
  DestructorTracer& operator=(const DestructorTracer&) = delete;
  DestructorTracer(DestructorTracer&& o) noexcept : id(o.id), sink(o.sink) { o.sink = nullptr; }
  DestructorTracer& operator=(DestructorTracer&&) = delete;
  ~DestructorTracer() {
    if (sink) sink->push_back(id);
  }
};

static void test_basic_get_and_miss() {
  LruCache<int, std::string> cache(3);
  VERIFY(cache.empty());
  VERIFY_IS_EQUAL(cache.size(), std::size_t(0));
  VERIFY_IS_EQUAL(cache.capacity(), std::size_t(3));
  VERIFY(cache.find(42) == nullptr);

  cache.insert(1, "one");
  cache.insert(2, "two");
  VERIFY_IS_EQUAL(cache.size(), std::size_t(2));
  VERIFY(!cache.empty());

  std::string* p = cache.find(1);
  VERIFY(p != nullptr);
  VERIFY_IS_EQUAL(*p, std::string("one"));
  VERIFY(cache.find(2) != nullptr);
  VERIFY(cache.find(3) == nullptr);
}

static void test_lru_eviction() {
  LruCache<int, int> cache(3);
  cache.insert(1, 100);
  cache.insert(2, 200);
  cache.insert(3, 300);

  // Fourth insert evicts 1 (least-recently-used so far).
  cache.insert(4, 400);
  VERIFY_IS_EQUAL(cache.size(), std::size_t(3));
  VERIFY(cache.find(1) == nullptr);
  VERIFY(cache.find(2) != nullptr);
  VERIFY(cache.find(3) != nullptr);
  VERIFY(cache.find(4) != nullptr);
}

static void test_hit_promotes_to_mru() {
  LruCache<int, int> cache(3);
  cache.insert(1, 100);
  cache.insert(2, 200);
  cache.insert(3, 300);

  // Touch 1 — it is now MRU, so 2 becomes LRU.
  VERIFY(cache.find(1) != nullptr);
  cache.insert(4, 400);
  VERIFY(cache.find(1) != nullptr);
  VERIFY(cache.find(2) == nullptr);
  VERIFY(cache.find(3) != nullptr);
  VERIFY(cache.find(4) != nullptr);
}

static void test_replace_existing_key() {
  LruCache<int, std::string> cache(3);
  cache.insert(1, "one");
  cache.insert(2, "two");

  // Re-inserting the same key replaces the value and promotes to MRU.
  std::string* p = cache.insert(1, "ONE");
  VERIFY(p != nullptr);
  VERIFY_IS_EQUAL(*p, std::string("ONE"));
  VERIFY_IS_EQUAL(cache.size(), std::size_t(2));

  // Re-inserting 1 made 2 the LRU; filling to capacity then exceeding evicts 2.
  cache.insert(3, "three");
  cache.insert(4, "four");
  VERIFY(cache.find(1) != nullptr);
  VERIFY(cache.find(2) == nullptr);
  VERIFY(cache.find(3) != nullptr);
  VERIFY(cache.find(4) != nullptr);
}

static void test_replace_destroys_old_value() {
  std::vector<int> destructions;
  LruCache<int, DestructorTracer> cache(2);
  cache.insert(1, DestructorTracer{1, &destructions});
  cache.insert(2, DestructorTracer{2, &destructions});
  destructions.clear();

  // Replacement destroys the old entry and does not require Value to be
  // move-assignable.
  DestructorTracer* p = cache.insert(1, DestructorTracer{10, &destructions});
  VERIFY(p != nullptr);
  VERIFY_IS_EQUAL(p->id, 10);
  VERIFY_IS_EQUAL(destructions.size(), std::size_t(1));
  VERIFY_IS_EQUAL(destructions.front(), 1);

  // Replacing 1 promoted it to MRU, so the next insert evicts 2.
  destructions.clear();
  cache.insert(3, DestructorTracer{3, &destructions});
  VERIFY_IS_EQUAL(destructions.size(), std::size_t(1));
  VERIFY_IS_EQUAL(destructions.front(), 2);
  VERIFY(cache.find(1) != nullptr);
  VERIFY(cache.find(3) != nullptr);
}

static void test_evicted_value_is_destroyed() {
  std::vector<int> destructions;
  {
    LruCache<int, DestructorTracer> cache(2);
    cache.insert(1, DestructorTracer{1, &destructions});
    cache.insert(2, DestructorTracer{2, &destructions});
    destructions.clear();  // discard any destructor calls from the temporaries above

    // Insert 3 — entry 1 (LRU) must be destroyed before 3 is added.
    cache.insert(3, DestructorTracer{3, &destructions});
    VERIFY_IS_EQUAL(destructions.size(), std::size_t(1));
    VERIFY_IS_EQUAL(destructions.front(), 1);

    destructions.clear();
  }
  // Destruction of the cache destroys the remaining entries (2 and 3, in some
  // order). We do not pin down the order, only the multiset.
  VERIFY_IS_EQUAL(destructions.size(), std::size_t(2));
  std::sort(destructions.begin(), destructions.end());
  VERIFY_IS_EQUAL(destructions[0], 2);
  VERIFY_IS_EQUAL(destructions[1], 3);
}

static void test_clear() {
  std::vector<int> destructions;
  LruCache<int, DestructorTracer> cache(4);
  cache.insert(1, DestructorTracer{1, &destructions});
  cache.insert(2, DestructorTracer{2, &destructions});
  destructions.clear();

  cache.clear();
  VERIFY(cache.empty());
  VERIFY_IS_EQUAL(destructions.size(), std::size_t(2));
  VERIFY(cache.find(1) == nullptr);

  // Reusable after clear.
  cache.insert(7, DestructorTracer{7, &destructions});
  VERIFY(cache.find(7) != nullptr);
}

static void test_move_semantics() {
  LruCache<int, int> a(2);
  a.insert(1, 100);
  a.insert(2, 200);

  LruCache<int, int> b(std::move(a));
  VERIFY_IS_EQUAL(b.size(), std::size_t(2));
  VERIFY(b.find(1) != nullptr);
  VERIFY(b.find(2) != nullptr);

  LruCache<int, int> c(1);
  c = std::move(b);
  VERIFY_IS_EQUAL(c.size(), std::size_t(2));  // capacity moved too
  VERIFY_IS_EQUAL(c.capacity(), std::size_t(2));
  VERIFY(c.find(1) != nullptr);
}

static void test_capacity_one() {
  LruCache<int, int> cache(1);
  cache.insert(1, 100);
  VERIFY(cache.find(1) != nullptr);
  cache.insert(2, 200);
  VERIFY(cache.find(1) == nullptr);
  VERIFY(cache.find(2) != nullptr);
}

static void test_complex_key() {
  // Verify the cache works with a user-supplied Hash / KeyEqual.
  struct Key {
    int a;
    int b;
    bool operator==(const Key& o) const { return a == o.a && b == o.b; }
  };
  struct KeyHash {
    std::size_t operator()(const Key& k) const noexcept { return std::hash<int>{}(k.a) * 31u + std::hash<int>{}(k.b); }
  };
  LruCache<Key, int, KeyHash> cache(2);
  cache.insert(Key{1, 2}, 12);
  cache.insert(Key{3, 4}, 34);
  int* p = cache.find(Key{1, 2});
  VERIFY(p != nullptr);
  VERIFY_IS_EQUAL(*p, 12);
  VERIFY(cache.find(Key{9, 9}) == nullptr);
}

EIGEN_DECLARE_TEST(lru_cache) {
  CALL_SUBTEST(test_basic_get_and_miss());
  CALL_SUBTEST(test_lru_eviction());
  CALL_SUBTEST(test_hit_promotes_to_mru());
  CALL_SUBTEST(test_replace_existing_key());
  CALL_SUBTEST(test_replace_destroys_old_value());
  CALL_SUBTEST(test_evicted_value_is_destroyed());
  CALL_SUBTEST(test_clear());
  CALL_SUBTEST(test_move_semantics());
  CALL_SUBTEST(test_capacity_one());
  CALL_SUBTEST(test_complex_key());
}
