/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "vmem.h"

#include <cassert>
#include <fmt/core.h>

#include "champsim.h"
#include "dram_controller.h"
#include "util/bits.h"

using namespace champsim::data::data_literals;

VirtualMemory::VirtualMemory(champsim::data::bytes page_table_page_size, std::size_t page_table_levels, champsim::chrono::clock::duration minor_penalty,
                             MEMORY_CONTROLLER& dram_, std::optional<uint64_t> randomization_seed_)
    : randomization_seed(randomization_seed_), dram(dram_), minor_fault_penalty(minor_penalty), pt_levels(page_table_levels),
      pte_page_size(page_table_page_size),
      next_pte_page(
          champsim::dynamic_extent{champsim::data::bits{LOG2_PAGE_SIZE}, champsim::data::bits{champsim::lg2(champsim::data::bytes{pte_page_size}.count())}}, 0)
{
  assert(pte_page_size > 1_kiB);
  assert(champsim::is_power_of_2(pte_page_size.count()));

  champsim::page_number last_vpage{
      champsim::lowest_address_for_size(champsim::data::bytes{PAGE_SIZE + champsim::ipow(pte_page_size.count(), static_cast<unsigned>(pt_levels))})};
  champsim::data::bits required_bits{LOG2_PAGE_SIZE + champsim::lg2(last_vpage.to<uint64_t>())};
  if (required_bits > champsim::address::bits) {
    fmt::print("[VMEM] WARNING: virtual memory configuration would require {} bits of addressing.\n", required_bits); // LCOV_EXCL_LINE
  }
  if (required_bits > champsim::data::bits{champsim::lg2(dram.size().count())}) {
    fmt::print("[VMEM] WARNING: physical memory size is smaller than virtual memory size.\n"); // LCOV_EXCL_LINE
  }
  populate_pages();
  // shuffle_pages();
}

VirtualMemory::VirtualMemory(champsim::data::bytes page_table_page_size, std::size_t page_table_levels, champsim::chrono::clock::duration minor_penalty,
                             MEMORY_CONTROLLER& dram_)
    : VirtualMemory(page_table_page_size, page_table_levels, minor_penalty, dram_, {})
{
}

void VirtualMemory::populate_pages()
{
  champsim::page_number base_address{0};
  assert(dram.size() > 1_MiB);

  if (DIRECT_PAGE_ALLOCATION_ON) {
    ppage_free_list.resize((dram.size() / PAGE_SIZE).count());
  } else {
    ppage_free_list.resize(((dram.size() - 1_MiB) / PAGE_SIZE).count());
    base_address = champsim::page_number{champsim::lowest_address_for_size(std::max<champsim::data::mebibytes>(champsim::data::bytes{PAGE_SIZE}, 1_MiB))};
  }
  assert(ppage_free_list.size() != 0);

  for (auto it = ppage_free_list.begin(); it != ppage_free_list.end(); it++) {
    *it = base_address;
    base_address++;
  }
}

void VirtualMemory::shuffle_pages()
{
  if (randomization_seed.has_value())
    std::shuffle(ppage_free_list.begin(), ppage_free_list.end(), std::mt19937_64{randomization_seed.value()});
}

champsim::dynamic_extent VirtualMemory::extent(std::size_t level) const
{
  const champsim::data::bits lower{LOG2_PAGE_SIZE + champsim::lg2(pte_page_size.count()) * (level - 1)};
  const auto size = static_cast<std::size_t>(champsim::lg2(pte_page_size.count()));
  return champsim::dynamic_extent{lower, size};
}

champsim::data::bits VirtualMemory::shamt(std::size_t level) const { return extent(level).lower; }

uint64_t VirtualMemory::get_offset(champsim::address vaddr, std::size_t level) const { return champsim::address_slice{extent(level), vaddr}.to<uint64_t>(); }

uint64_t VirtualMemory::get_offset(champsim::page_number vaddr, std::size_t level) const { return get_offset(champsim::address{vaddr}, level); }

champsim::page_number VirtualMemory::ppage_front() const
{
  assert(available_ppages() > 0);
  return ppage_free_list.front();
}

void VirtualMemory::ppage_pop()
{
  ppage_free_list.pop_front();
  if (available_ppages() == 0) {
    fmt::print("[VMEM] WARNING: Out of physical memory, freeing ppages\n");
    populate_pages();
    // shuffle_pages();
  }
}

std::size_t VirtualMemory::available_ppages() const { return (ppage_free_list.size()); }

std::pair<champsim::page_number, champsim::chrono::clock::duration> VirtualMemory::va_to_pa(uint32_t cpu_num, champsim::page_number vpage)
{
  const auto key = std::pair{cpu_num, champsim::page_number{vpage}};
  if (DIRECT_PAGE_ALLOCATION_ON) {
    auto ppage_iter = vpage_to_ppage_map.find(key);
    // vpage is already mapped to a ppage
    if (ppage_iter != vpage_to_ppage_map.end()) {
      return std::pair{ppage_iter->second, champsim::chrono::clock::duration::zero()};
    }

    const auto total_ppages = static_cast<uint64_t>((dram.size() / PAGE_SIZE).count());
    assert(total_ppages != 0);
    const auto desired_index = vpage.to<uint64_t>() % total_ppages;
    const champsim::page_number desired_ppage{desired_index};

    auto owner_vpage_iter = ppage_to_vpage_map.find(desired_ppage);
    if (owner_vpage_iter != ppage_to_vpage_map.end()) {
      vpage_to_ppage_map.erase({owner_vpage_iter->second.first, owner_vpage_iter->second.second});
    }

    vpage_to_ppage_map.emplace(key, desired_ppage);
    ppage_to_vpage_map[desired_ppage] = {cpu_num, champsim::page_number{vpage}};

    if constexpr (champsim::debug_print) {
      fmt::print("[VMEM] {} paddr: {} vpage: {} fault: {}\n", __func__, desired_ppage, champsim::page_number{vpage}, true);
    }

    return std::pair{desired_ppage, minor_fault_penalty};
  } else {
    auto [ppage_iter, fault] = vpage_to_ppage_map.try_emplace(key, ppage_front());
    // this vpage doesn't yet have a ppage mapping
    if (fault) {
      ppage_pop();
    }

    auto penalty = fault ? minor_fault_penalty : champsim::chrono::clock::duration::zero();

    if constexpr (champsim::debug_print) {
      fmt::print("[VMEM] {} paddr: {} vpage: {} fault: {}\n", __func__, ppage_iter->second, champsim::page_number{vpage}, fault);
    }

    return std::pair{ppage_iter->second, penalty};
  }
}

std::pair<champsim::page_number, bool> VirtualMemory::va_to_pa_without_allocation(uint32_t cpu_num, champsim::page_number vpage)
{
  bool is_allocated = true;
  const auto key = std::pair{cpu_num, champsim::page_number{vpage}};
  auto ppage_iter = vpage_to_ppage_map.find(key);
  if (ppage_iter == vpage_to_ppage_map.end()) {
    is_allocated = false;
    return std::pair{champsim::page_number{}, is_allocated};
  }
  return std::pair{ppage_iter->second, is_allocated};
}

// returns the physical address of the PTE for the given virtual address at the given level
std::pair<champsim::address, champsim::chrono::clock::duration> VirtualMemory::get_pte_pa(uint32_t cpu_num, champsim::page_number vaddr, std::size_t level)
{
  // allocate a new PTE page as `active_pte_page` if necessary
  // the size of PTE pages can be smaller than the size of a physical page according to the config settings
  // that is 4KB default, but if that is smaller than usual like 2KB or 1KB, one physical page can hold multiple PTE pages
  // so we need to check the offset within the physical page here
  // when the offset reaches 0, we need to allocate a new physical page for the next PTE page
  if (champsim::page_offset{next_pte_page} == champsim::page_offset{0}) {
    active_pte_page = ppage_front();
    ppage_pop();
  }

  champsim::dynamic_extent pte_table_entry_extent{champsim::address::bits, shamt(level)}; //
  auto [ppage, fault] =
      page_table.try_emplace({cpu_num, level, champsim::address_slice{pte_table_entry_extent, vaddr}}, champsim::splice(active_pte_page, next_pte_page));

  // this PTE doesn't yet have a mapping
  if (fault) {
    next_pte_page++;
  }

  auto offset = get_offset(vaddr, level);
  champsim::address paddr{
      champsim::splice(ppage->second, champsim::address_slice{champsim::dynamic_extent{champsim::data::bits{champsim::lg2(pte_entry::byte_multiple)},
                                                                                       static_cast<std::size_t>(champsim::lg2(pte_page_size.count()))},
                                                              offset})};
  if constexpr (champsim::debug_print) {
    fmt::print("[VMEM] {} paddr: {} vaddr: {} pt_page_offset: {} translation_level: {} fault: {}\n", __func__, paddr, vaddr, offset, level, fault);
  }

  auto penalty = minor_fault_penalty;
  if (!fault) {
    penalty = champsim::chrono::clock::duration::zero();
  }

  return {paddr, penalty};
}
