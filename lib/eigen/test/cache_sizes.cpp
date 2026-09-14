// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

#ifdef EIGEN_CPU_CACHE_SYSFS
#include <sched.h>
#include <sys/stat.h>
#include <cstdlib>
#include <string>
#include <vector>
#endif

void cache_sizes_plausible() {
  // Whatever the platform reports has to be a plausible data-cache size: a stray unit suffix
  // would otherwise be read as a handful of bytes and silently shrink every blocking size.
  std::ptrdiff_t l1 = -1, l2 = -1, l3 = -1, l3_per_cpu = -1;
  internal::queryCacheSizes(l1, l2, l3, l3_per_cpu);
  for (std::ptrdiff_t size : {l1, l2, l3, l3_per_cpu})
    if (size > 0) VERIFY(size >= 1024);
  // A share is one CPU's slice of one L3 instance, so it can never exceed the reported L3.
  VERIFY(l3_per_cpu >= 0);
  if (l3_per_cpu > 0) VERIFY(l3 > 0 && l3_per_cpu <= l3);
}

#ifdef EIGEN_CPU_CACHE_SYSFS

using internal::CpuCacheTopology;

bool same_topology(const CpuCacheTopology& a, const CpuCacheTopology& b) {
  return a.l1 == b.l1 && a.l2 == b.l2 && a.l3 == b.l3 && a.l3_per_cpu == b.l3_per_cpu;
}

CpuCacheTopology topology_of(std::ptrdiff_t l1, std::ptrdiff_t l2, std::ptrdiff_t l3, std::ptrdiff_t l3_per_cpu) {
  CpuCacheTopology topology;
  topology.l1 = l1;
  topology.l2 = l2;
  topology.l3 = l3;
  topology.l3_per_cpu = l3_per_cpu;
  return topology;
}

void cache_sizes_parsers() {
  VERIFY(internal::parseCpuCacheSize("64K\n") == 64 * 1024);
  VERIFY(internal::parseCpuCacheSize("2048K\n") == 2048 * 1024);
  VERIFY(internal::parseCpuCacheSize("32M\n") == 32 * 1024 * 1024);
  // A size Eigen cannot make sense of has to read as "unknown", never as a few bytes.
  VERIFY(internal::parseCpuCacheSize("bogus") == 0);
  VERIFY(internal::parseCpuCacheSize("0K") == 0);
  // A bare count is bytes, not kibibytes.
  VERIFY(internal::parseCpuCacheSize("512\n") == 512);

  VERIFY(internal::parseCpuListCount("0\n") == 1);
  VERIFY(internal::parseCpuListCount("0-3\n") == 4);
  VERIFY(internal::parseCpuListCount("0-3,8-11\n") == 8);
  VERIFY(internal::parseCpuListCount("0,2,4\n") == 3);
  VERIFY(internal::parseCpuListCount("") == 0);
  VERIFY(internal::parseCpuListCount("3-0") == 0);
  VERIFY(internal::parseCpuListCount("0-3\r\n") == 4);
  // Trailing text means the format is not the one assumed, so the count is not trustworthy:
  // a small count here would inflate l3_per_cpu rather than leave the share unknown.
  VERIFY(internal::parseCpuListCount("0-3junk") == 0);
  VERIFY(internal::parseCpuListCount("0-3 8-11") == 0);
  VERIFY(internal::parseCpuListCount("0-3,") == 0);
  VERIFY(internal::parseCpuListCount("0-3,junk") == 0);
  VERIFY(internal::parseCpuListCount("junk") == 0);
  // strtol would accept a sign, but no CPU has a negative id, and none has one beyond what a kernel can number.
  VERIFY(internal::parseCpuListCount("-1") == 0);
  VERIFY(internal::parseCpuListCount("0-70000") == 0);

  // The ranges are handed over one at a time, and a list that turns out malformed is reported as such even
  // though its leading ranges were already visited.
  int ranges = 0, cpus = 0;
  const auto tally = [&](int first, int last) {
    ++ranges;
    cpus += last - first + 1;
  };
  VERIFY(internal::parseCpuList("0-3,8,10-11\n", tally));
  VERIFY(ranges == 3 && cpus == 7);
  VERIFY(!internal::parseCpuList("0-3,8,10-11junk\n", tally));
}

// A sysfs tree laid out exactly as Linux publishes it, describing whatever machine a test needs.
struct SysfsFixture {
  std::string root;
  std::vector<std::string> created;

  SysfsFixture() {
    const char* base = std::getenv("TMPDIR");
    const std::string pattern = std::string(base != nullptr ? base : "/tmp") + "/eigen-cache-sizes-XXXXXX";
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    if (mkdtemp(buffer.data()) != nullptr) {
      root = buffer.data();
      created.push_back(root);
    }
  }
  ~SysfsFixture() {
    for (auto it = created.rbegin(); it != created.rend(); ++it) std::remove(it->c_str());
  }

  void directory(const std::string& path) {
    if (mkdir(path.c_str(), 0700) == 0) created.push_back(path);
  }
  void file(const std::string& relative, const char* text) {
    const std::string path = root + "/" + relative;
    std::FILE* stream = std::fopen(path.c_str(), "w");
    VERIFY(stream != nullptr);
    std::fputs(text, stream);
    std::fclose(stream);
    created.push_back(path);
  }
  void cpu(int id) {
    directory(root + "/cpu" + std::to_string(id));
    directory(root + "/cpu" + std::to_string(id) + "/cache");
  }
  void cache(int id, int index, const char* level, const char* type, const char* size, const char* shared) {
    const std::string dir = "cpu" + std::to_string(id) + "/cache/index" + std::to_string(index);
    directory(root + "/" + dir);
    file(dir + "/level", level);
    file(dir + "/type", type);
    file(dir + "/size", size);
    file(dir + "/shared_cpu_list", shared);
  }
};

void cache_sizes_fixture_topology() {
  SysfsFixture fixture;
  if (fixture.root.empty()) return;
  const char* const root = fixture.root.c_str();

  // Two big cores (0 and 3) with a 16MB L3 between them, and two little cores (1 and 2) with a 2MB
  // cluster L3. CPU 3 is online but publishes no cache directory at all, and CPU 2 has no L2, so its
  // L3 sits at the index the others use for L2.
  fixture.cpu(0);
  fixture.cache(0, 0, "1\n", "Data\n", "64K\n", "0\n");
  fixture.cache(0, 1, "1\n", "Instruction\n", "128K\n", "0\n");
  fixture.cache(0, 2, "2\n", "Unified\n", "1024K\n", "0\n");
  fixture.cache(0, 3, "3\n", "Unified\n", "16384K\n", "0,3\n");
  fixture.cpu(1);
  fixture.cache(1, 0, "1\n", "Data\n", "32K\n", "1\n");
  fixture.cache(1, 1, "1\n", "Instruction\n", "32K\n", "1\n");
  fixture.cache(1, 2, "2\n", "Unified\n", "256K\n", "1\n");
  fixture.cache(1, 3, "3\n", "Unified\n", "2048K\n", "1-2\n");
  fixture.cpu(2);
  fixture.cache(2, 0, "1\n", "Data\n", "32K\n", "2\n");
  fixture.cache(2, 1, "1\n", "Instruction\n", "32K\n", "2\n");
  fixture.cache(2, 2, "3\n", "Unified\n", "2048K\n", "1-2\n");
  fixture.directory(fixture.root + "/cpu3");
  fixture.file("online", "0-3\n");

  const CpuCacheTopology big = topology_of(64 * 1024, 1024 * 1024, 16384 * 1024, 8192 * 1024);
  const CpuCacheTopology little = topology_of(32 * 1024, 256 * 1024, 2048 * 1024, 1024 * 1024);
  const CpuCacheTopology unknown = topology_of(0, 0, 0, 0);

  // Each CPU on its own. The larger instruction cache must not pass for the L1 data cache.
  VERIFY(same_topology(internal::readCpuCacheTopologySysfs(root, 0), big));
  VERIFY(same_topology(internal::readCpuCacheTopologySysfs(root, 1), little));
  VERIFY(
      same_topology(internal::readCpuCacheTopologySysfs(root, 2), topology_of(32 * 1024, 0, 2048 * 1024, 1024 * 1024)));
  VERIFY(same_topology(internal::readCpuCacheTopologySysfs(root, 3), unknown));

  // A thread that may land on any CPU has to block for the smallest caches among them.
  const auto every_cpu = [](int) { return true; };
  VERIFY(same_topology(internal::queryCpuCacheTopologySysfs(root, every_cpu), little));
  // One confined to the big cores sees only their geometry, and a mask that excludes CPU 0 must not carry
  // CPU 0's geometry along.
  VERIFY(same_topology(internal::queryCpuCacheTopologySysfs(root, [](int cpu) { return cpu == 0; }), big));
  VERIFY(same_topology(internal::queryCpuCacheTopologySysfs(root, [](int cpu) { return cpu == 0 || cpu == 3; }), big));
  VERIFY(same_topology(internal::queryCpuCacheTopologySysfs(root, [](int cpu) { return cpu != 0; }), little));
  VERIFY(
      same_topology(internal::queryCpuCacheTopologySysfs(root, [](int cpu) { return cpu == 1 || cpu == 3; }), little));
  // A CPU that reports nothing, or no CPU at all, leaves the geometry unknown rather than defaulted.
  VERIFY(same_topology(internal::queryCpuCacheTopologySysfs(root, [](int cpu) { return cpu == 3; }), unknown));
  VERIFY(same_topology(internal::queryCpuCacheTopologySysfs(root, [](int) { return false; }), unknown));

  // An online list that cannot be trusted, or that is missing, means no CPU is known.
  fixture.file("online", "0-3junk\n");
  VERIFY(same_topology(internal::queryCpuCacheTopologySysfs(root, every_cpu), unknown));
  fixture.file("online", "0-3");
  VERIFY(same_topology(internal::queryCpuCacheTopologySysfs(root, every_cpu), unknown));
  std::remove((fixture.root + "/online").c_str());
  VERIFY(same_topology(internal::queryCpuCacheTopologySysfs(root, every_cpu), unknown));
}

void cache_sizes_affinity() {
  // On a kernel that publishes the topology, Eigen has to pick it up instead of falling back to
  // its compiled-in defaults. glibc's sysconf answers only on x86, so before the sysfs fallback
  // this failed on every other Linux architecture.
  std::ptrdiff_t l1, l2, l3, l3_per_cpu;
  internal::queryCacheSizes(l1, l2, l3, l3_per_cpu);
  const CpuCacheTopology mine = internal::queryCpuCacheTopologySysfs();
  if (mine.l1 > 0) VERIFY(l1 > 0);
  VERIFY(l3_per_cpu == mine.l3_per_cpu);

  // Confine the thread to every CPU it may use but CPU 0: the detected geometry has to follow the mask.
  cpu_set_t original;
  if (sched_getaffinity(0, sizeof(original), &original) != 0) return;
  if (CPU_COUNT(&original) < 2 || !CPU_ISSET(0, &original)) return;
  cpu_set_t restricted = original;
  CPU_CLR(0, &restricted);
  VERIFY(sched_setaffinity(0, sizeof(restricted), &restricted) == 0);
  const CpuCacheTopology seen = internal::queryCpuCacheTopologySysfs();
  const CpuCacheTopology expected = internal::queryCpuCacheTopologySysfs(
      "/sys/devices/system/cpu", [&restricted](int cpu) { return cpu < CPU_SETSIZE && CPU_ISSET(cpu, &restricted); });
  VERIFY(sched_setaffinity(0, sizeof(original), &original) == 0);
  VERIFY(same_topology(seen, expected));
}

#endif  // EIGEN_CPU_CACHE_SYSFS

EIGEN_DECLARE_TEST(cache_sizes) {
  CALL_SUBTEST(cache_sizes_plausible());
#ifdef EIGEN_CPU_CACHE_SYSFS
  CALL_SUBTEST(cache_sizes_parsers());
  CALL_SUBTEST(cache_sizes_fixture_topology());
  CALL_SUBTEST(cache_sizes_affinity());
#endif
}
