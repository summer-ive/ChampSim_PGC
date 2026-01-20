#include "spp_pgc_ideal.h"

#include <cassert>
#include <iostream>

#include "prefetcher_helper.h"

void spp_pgc_ideal::prefetcher_initialize()
{
  std::cout << "Prefetcher: spp_pgc_ideal" << std::endl;
  std::cout << "PGC enabled: " << (IS_PGC_ENABLED ? "true" : "false") << std::endl;
  std::cout << "[SPP] signature-table unit size: 2^" << SIG_UNIT_BIT << " [Byte]\n";
  std::cout << "Initialize SIGNATURE TABLE" << std::endl;
  std::cout << "ST_SET: " << ST_SET << std::endl;
  std::cout << "ST_WAY: " << ST_WAY << std::endl;
  std::cout << "ST_TAG_BIT: " << ST_TAG_BIT << std::endl;

  std::cout << std::endl << "Initialize PATTERN TABLE" << std::endl;
  std::cout << "PT_SET: " << PT_SET << std::endl;
  std::cout << "PT_WAY: " << PT_WAY << std::endl;
  std::cout << "SIG_DELTA_BIT: " << SIG_DELTA_BIT << std::endl;
  std::cout << "C_SIG_BIT: " << C_SIG_BIT << std::endl;
  std::cout << "C_DELTA_BIT: " << C_DELTA_BIT << std::endl;

  std::cout << std::endl << "Initialize PREFETCH FILTER" << std::endl;
  std::cout << "FILTER_SET: " << FILTER_SET << std::endl;

  // pass pointers
  ST._parent = this;
  PT._parent = this;
  FILTER._parent = this;
  GHR._parent = this;
}

void spp_pgc_ideal::reset_roi_status()
{
  for (auto& pair : count_map) {
    pair.second = 0;
  }
  pgc_distance_map_l2c.clear();
  pgc_distance_map_llc.clear();
  narrowly_defined_pgc_distance_map_l2c.clear();
  narrowly_defined_pgc_distance_map_llc.clear();
  return;
}

bool spp_pgc_ideal::is_adjacent_in_virtual(uint32_t trigger_cpu, champsim::page_number trigger_vpage, champsim::page_number pf_ppage)
{
  bool is_allocated;
  champsim::page_number trigger_ppage;
  std::tie(trigger_ppage, is_allocated) = va_to_pa_ideal(trigger_cpu, trigger_vpage);
  if (!is_allocated) // trigger_vpage is not mapped to any physical page
    return false;

  if (pf_ppage == trigger_ppage) // same page
    return true;
  const long long step = (pf_ppage > trigger_ppage) ? 1 : -1;
  long long delta = 0;
  while (pf_ppage - delta != trigger_ppage) {
    delta += step;
  }

  champsim::page_number cur_vpage = trigger_vpage;
  champsim::page_number cur_ppage = trigger_ppage;

  while (delta != 0) {
    champsim::page_number adj_vpage = cur_vpage + step;
    champsim::page_number adj_ppage;
    std::tie(adj_ppage, is_allocated) = va_to_pa_ideal(trigger_cpu, adj_vpage);
    if (!is_allocated) // adj_vpage is not mapped to any physical page
      return false;

    if (adj_ppage != (cur_ppage + step))
      return false;

    cur_vpage = adj_vpage;
    cur_ppage = adj_ppage;
    delta -= step;
  }

  if (cur_ppage == pf_ppage) {
    return true;
  }

  std::cout << "The target physical page address doesn't match the incremented physical page address. There may be a bug." << std::endl;
  return false;
}

void spp_pgc_ideal::prefetcher_cycle_operate() {}

uint32_t spp_pgc_ideal::prefetcher_cache_operate(uint32_t trigger_cpu, champsim::address trigger_paddr, champsim::address trigger_vaddr, champsim::address ip,
                                                 bool cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in)
{
  if (!intern_->warmup && !roi_stats_initialized) {
    reset_roi_status();
    roi_stats_initialized = true;
  }
  const champsim::page_number trigger_ppage{trigger_paddr};
  const champsim::page_number trigger_vpage{trigger_vaddr};
  uint32_t last_sig = 0, curr_sig = 0, depth = 0;
  std::vector<uint32_t> confidence_q(intern_->MSHR_SIZE);

  typename spp_pgc_ideal::offset_type::difference_type delta = 0;
  std::vector<typename spp_pgc_ideal::offset_type::difference_type> delta_q(intern_->MSHR_SIZE);

  for (uint32_t i = 0; i < intern_->MSHR_SIZE; i++) {
    confidence_q[i] = 0;
    delta_q[i] = 0;
  }
  GHR.global_accuracy = GHR.pf_issued ? ((100 * GHR.pf_useful) / GHR.pf_issued) : 0;

  if constexpr (SPP_DEBUG_PRINT) {
    std::cout << std::endl << "[ChampSim] " << __func__ << " trigger_paddr: " << trigger_paddr;
    std::cout << " trigger_ppage: " << trigger_ppage << std::endl;
  }

  // Stage 1: Read and update a sig stored in ST
  // last_sig and delta are used to update (sig, delta) correlation in PT
  // curr_sig is used to read prefetch candidates in PT
  ST.read_and_update_sig(trigger_paddr, last_sig, curr_sig, delta);

  // Also check the prefetch filter in parallel to update global accuracy counters
  FILTER.check(trigger_paddr, spp_pgc_ideal::L2C_DEMAND);

  // Stage 2: Update delta patterns stored in PT
  if (last_sig)
    PT.update_pattern(last_sig, delta);

  // Stage 3: Start prefetching
  auto base_paddr = trigger_paddr;
  uint32_t lookahead_conf = 100, pf_q_head = 0, pf_q_tail = 0;
  uint8_t do_lookahead = 0;

  do {
    uint32_t lookahead_way = PT_WAY;
    PT.read_pattern(curr_sig, delta_q, confidence_q, lookahead_way, lookahead_conf, pf_q_tail, depth);

    do_lookahead = 0;
    for (uint32_t i = pf_q_head; i < pf_q_tail; i++) {
      champsim::address pf_paddr{champsim::block_number{base_paddr} + delta_q[i]};
      champsim::page_number pf_ppage{pf_paddr};
      champsim::page_number base_ppage{base_paddr};
      bool is_pgc_candidate = (pf_ppage != trigger_ppage);               // Prefetch target page is different from the trigger page
      bool is_narrowly_defined_pgc_candidate = (pf_ppage != base_ppage); // Prefetch request is crossing the physical page boundary
      int page_distance = pf_ppage.to<int>() - trigger_ppage.to<int>();

      // ここでcand集計
      count_map["prefetch_candidate_total"]++;

      if (confidence_q[i] >= PF_THRESHOLD) {
        const bool is_prefetch_in_this_level = (confidence_q[i] >= FILL_THRESHOLD);
        if (is_prefetch_in_this_level) {
          count_map["prefetch_candidate_l2c"]++;
        } else {
          count_map["prefetch_candidate_llc"]++;
        }

        // prefetch filter check
        if (FILTER.check(pf_paddr, (is_prefetch_in_this_level ? spp_pgc_ideal::SPP_L2C_PREFETCH : spp_pgc_ideal::SPP_LLC_PREFETCH))) {

          // case when PGC is disabled
          if (!IS_PGC_ENABLED && is_pgc_candidate) {
            if constexpr (GHR_ON) {
              // Store this prefetch request in GHR to bootstrap SPP learning when
              // we see a ST miss (i.e., accessing a new page)
              GHR.update_entry(curr_sig, confidence_q[i], spp_pgc_ideal::offset_type{pf_paddr}, delta_q[i]);
            }
            continue;
          }

          // pgc page continuity check
          if (!is_adjacent_in_virtual(trigger_cpu, trigger_vpage, pf_ppage)) {
            if (is_prefetch_in_this_level) {
              count_map["trashed_va_discontinuous_pgc_l2c"]++;
            } else {
              count_map["trashed_va_discontinuous_pgc_llc"]++;
            }
            continue;
          }

          bool is_prefetch_succeed = prefetch_line(pf_paddr, is_prefetch_in_this_level, 0); // Use addr (not base_addr) to obey the same physical page boundary

          auto cache_line = champsim::block_number{pf_paddr};
          // ChampSimではハッシュ関数を用いてプリフェッチフィルタのスロット割当を行う。
          // ハッシュ値のうち、下位からREMAINDER_BIT分はタグに、そこから上位のQUOTIENT_BIT分をインデクシングに使用。
          uint64_t hash = get_hash(cache_line.to<uint64_t>());
          uint32_t q = (hash >> REMAINDER_BIT) & ((1 << QUOTIENT_BIT) - 1);

          if (is_prefetch_in_this_level) {
            count_map["prefetch_request_l2c"]++;
            if (is_pgc_candidate) {
              count_map["pgc_request_l2c"]++;
              if (is_narrowly_defined_pgc_candidate) {
                count_map["narrowly_defined_pgc_request_l2c"]++;
              }
            }
            if (is_prefetch_succeed) {
              count_map["prefetch_issued_l2c"]++;
              if (is_pgc_candidate) {
                count_map["pgc_issued_l2c"]++;
                pgc_distance_map_l2c[page_distance]++;
                FILTER.is_pgc[q] = 1;
                if (is_narrowly_defined_pgc_candidate) {
                  count_map["narrowly_defined_pgc_issued_l2c"]++;
                  narrowly_defined_pgc_distance_map_l2c[page_distance]++;
                  FILTER.is_narrowly_defined_pgc[q] = 1;
                }
              } else {
                FILTER.is_pgc[q] = 0;
                FILTER.is_narrowly_defined_pgc[q] = 0;
              }
            }
          } else {
            count_map["prefetch_request_llc"]++;
            if (is_pgc_candidate) {
              count_map["pgc_request_llc"]++;
              if (is_narrowly_defined_pgc_candidate) {
                count_map["narrowly_defined_pgc_request_llc"]++;
              }
            }
            if (is_prefetch_succeed) {
              count_map["prefetch_issued_llc"]++;
              if (is_pgc_candidate) {
                count_map["pgc_issued_llc"]++;
                pgc_distance_map_llc[page_distance]++;
                if (is_narrowly_defined_pgc_candidate) {
                  count_map["narrowly_defined_pgc_issued_llc"]++;
                  narrowly_defined_pgc_distance_map_llc[page_distance]++;
                }
              }
            }
          }

          if (is_pgc_candidate) {
            if constexpr (GHR_ON) {
              // Store this prefetch request in GHR to bootstrap SPP learning when
              // we see a ST miss (i.e., accessing a new page)
              GHR.update_entry(curr_sig, confidence_q[i], spp_pgc_ideal::offset_type{pf_paddr}, delta_q[i]);
            }
          }
          if (is_prefetch_in_this_level) {
            GHR.pf_issued++;
            if (GHR.pf_issued > GLOBAL_COUNTER_MAX) {
              GHR.pf_issued >>= 1;
              GHR.pf_useful >>= 1;
            }
            if constexpr (SPP_DEBUG_PRINT) {
              std::cout << "[ChampSim] SPP L2 prefetch issued GHR.pf_issued: " << GHR.pf_issued << " GHR.pf_useful: " << GHR.pf_useful << std::endl;
            }
          }

          if constexpr (SPP_DEBUG_PRINT) {
            std::cout << "[ChampSim] " << __func__ << " base_paddr: " << base_paddr << " pf_paddr: " << pf_paddr;
            std::cout << " prefetch_delta: " << delta_q[i] << " confidence: " << confidence_q[i];
            std::cout << " depth: " << depth << std::endl;
          }
        }
        do_lookahead = 1;
      } else {
        count_map["trashed_prefetch_low_confidence"]++;
        if (IS_PGC_ENABLED && (pf_ppage != trigger_ppage)) {
          count_map["trashed_pgc_low_confidence"]++;
        }
      }
    }
    pf_q_head = pf_q_tail;

    // Update base_addr and curr_sig
    if (lookahead_way < PT_WAY) {
      uint32_t set = get_hash(curr_sig) % PT_SET;
      base_paddr += (PT.delta[set][lookahead_way] << LOG2_BLOCK_SIZE);

      // PT.delta uses a 7-bit sign magnitude representation to generate
      // sig_delta
      // int sig_delta = (PT.delta[set][lookahead_way] < 0) ? ((((-1) *
      // PT.delta[set][lookahead_way]) & 0x3F) + 0x40) :
      // PT.delta[set][lookahead_way];
      auto sig_delta = (PT.delta[set][lookahead_way] < 0) ? (((-1) * PT.delta[set][lookahead_way]) + (1 << (SIG_DELTA_BIT - 1))) : PT.delta[set][lookahead_way];
      curr_sig = ((curr_sig << SIG_SHIFT) ^ sig_delta) & SIG_MASK;
    }

    if constexpr (SPP_DEBUG_PRINT) {
      std::cout << "Looping curr_sig: " << std::hex << curr_sig << " base_paddr: " << base_paddr << std::dec;
      std::cout << " pf_q_head: " << pf_q_head << " pf_q_tail: " << pf_q_tail << " depth: " << depth << std::endl;
    }
  } while (LOOKAHEAD_ON && do_lookahead);

  return metadata_in;
}

uint32_t spp_pgc_ideal::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr,
                                              uint32_t metadata_in)
{
  if constexpr (FILTER_ON) {
    if constexpr (SPP_DEBUG_PRINT) {
      std::cout << std::endl;
    }
    FILTER.check(evicted_addr, spp_pgc_ideal::L2C_EVICT);
  }

  return metadata_in;
}

void spp_pgc_ideal::prefetcher_final_stats()
{
  std::cout << "[SPP] total prefetch candidate: " << count_map["prefetch_candidate_l2c"] + count_map["prefetch_candidate_llc"] << "\n";
  std::cout << "[SPP] l2c prefetch candidate: " << count_map["prefetch_candidate_l2c"] << "\n";
  std::cout << "[SPP] llc prefetch candidate: " << count_map["prefetch_candidate_llc"] << "\n";

  std::cout << "[SPP] trashed prefetch candidates with lower confidence than llc fill threshold: " << count_map["trashed_prefetch_low_confidence"] << "\n";
  std::cout << "[SPP] trashed pgc candidates with lower confidence than llc fill threshold: " << count_map["trashed_pgc_low_confidence"] << "\n";

  std::cout << "[SPP] trashed l2c pgc candidates with virtual address discontinuity: " << count_map["trashed_va_discontinuous_pgc_l2c"] << "\n";
  std::cout << "[SPP] trashed llc pgc candidates with virtual address discontinuity: " << count_map["trashed_va_discontinuous_pgc_llc"] << "\n";

  std::cout << "[SPP] total prefetch request: " << count_map["prefetch_request_l2c"] + count_map["prefetch_request_llc"] << "\n";
  std::cout << "[SPP] l2c prefetch request: " << count_map["prefetch_request_l2c"] << "\n";
  std::cout << "[SPP] llc prefetch request: " << count_map["prefetch_request_llc"] << "\n";
  std::cout << "[SPP] total pgc request: " << count_map["pgc_request_l2c"] + count_map["pgc_request_llc"] << "\n";
  std::cout << "[SPP] l2c pgc request: " << count_map["pgc_request_l2c"] << "\n";
  std::cout << "[SPP] llc pgc request: " << count_map["pgc_request_llc"] << "\n";
  std::cout << "[SPP] total narrowly defined pgc request: " << count_map["narrowly_defined_pgc_request_l2c"] + count_map["narrowly_defined_pgc_request_llc"]
            << "\n";
  std::cout << "[SPP] l2c narrowly defined pgc request: " << count_map["narrowly_defined_pgc_request_l2c"] << "\n";
  std::cout << "[SPP] llc narrowly defined pgc request: " << count_map["narrowly_defined_pgc_request_llc"] << "\n";

  std::cout << "[SPP] total prefetch issued: " << count_map["prefetch_issued_l2c"] + count_map["prefetch_issued_llc"] << "\n";
  std::cout << "[SPP] l2c prefetch issued: " << count_map["prefetch_issued_l2c"] << "\n";
  std::cout << "[SPP] llc prefetch issued: " << count_map["prefetch_issued_llc"] << "\n";
  std::cout << "[SPP] total pgc issued: " << count_map["pgc_issued_l2c"] + count_map["pgc_issued_llc"] << "\n";
  std::cout << "[SPP] l2c pgc issued: " << count_map["pgc_issued_l2c"] << "\n";
  std::cout << "[SPP] llc pgc issued: " << count_map["pgc_issued_llc"] << "\n";
  std::cout << "[SPP] total narrowly defined pgc issued: " << count_map["narrowly_defined_pgc_issued_l2c"] + count_map["narrowly_defined_pgc_issued_llc"]
            << "\n";
  std::cout << "[SPP] l2c narrowly defined pgc issued: " << count_map["narrowly_defined_pgc_issued_l2c"] << "\n";
  std::cout << "[SPP] llc narrowly defined pgc issued: " << count_map["narrowly_defined_pgc_issued_llc"] << "\n";

  std::cout << "[SPP] l2c useful prefetch: " << count_map["useful_prefetch_l2c"] << "\n";
  std::cout << "[SPP] l2c useful pgc: " << count_map["useful_pgc_l2c"] << "\n";
  std::cout << "[SPP] l2c useful narrowly defined pgc: " << count_map["useful_narrowly_defined_pgc_l2c"] << "\n";

  std::unordered_map<int, uint64_t> pgc_distance_map_total;
  for (auto& [dist, cnt] : pgc_distance_map_l2c)
    pgc_distance_map_total[dist] += cnt;
  for (auto& [dist, cnt] : pgc_distance_map_llc)
    pgc_distance_map_total[dist] += cnt;
  std::unordered_map<int, uint64_t> narrowly_defined_pgc_distance_map_total;
  for (auto& [dist, cnt] : narrowly_defined_pgc_distance_map_l2c)
    narrowly_defined_pgc_distance_map_total[dist] += cnt;
  for (auto& [dist, cnt] : narrowly_defined_pgc_distance_map_llc)
    narrowly_defined_pgc_distance_map_total[dist] += cnt;

  std::vector<std::pair<int, uint64_t>> pgc_distance_vector_total(pgc_distance_map_total.begin(), pgc_distance_map_total.end());
  std::vector<std::pair<int, uint64_t>> pgc_distance_vector_l2c(pgc_distance_map_l2c.begin(), pgc_distance_map_l2c.end());
  std::vector<std::pair<int, uint64_t>> pgc_distance_vector_llc(pgc_distance_map_llc.begin(), pgc_distance_map_llc.end());
  std::vector<std::pair<int, uint64_t>> narrowly_defined_pgc_distance_vector_total(narrowly_defined_pgc_distance_map_total.begin(),
                                                                                   narrowly_defined_pgc_distance_map_total.end());
  std::vector<std::pair<int, uint64_t>> narrowly_defined_pgc_distance_vector_l2c(narrowly_defined_pgc_distance_map_l2c.begin(),
                                                                                 narrowly_defined_pgc_distance_map_l2c.end());
  std::vector<std::pair<int, uint64_t>> narrowly_defined_pgc_distance_vector_llc(narrowly_defined_pgc_distance_map_llc.begin(),
                                                                                 narrowly_defined_pgc_distance_map_llc.end());

  std::sort(pgc_distance_vector_total.begin(), pgc_distance_vector_total.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  std::sort(pgc_distance_vector_l2c.begin(), pgc_distance_vector_l2c.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  std::sort(pgc_distance_vector_llc.begin(), pgc_distance_vector_llc.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  std::sort(narrowly_defined_pgc_distance_vector_total.begin(), narrowly_defined_pgc_distance_vector_total.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  std::sort(narrowly_defined_pgc_distance_vector_l2c.begin(), narrowly_defined_pgc_distance_vector_l2c.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  std::sort(narrowly_defined_pgc_distance_vector_llc.begin(), narrowly_defined_pgc_distance_vector_llc.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  std::cout << "[SPP] total pgc distance:\n";
  for (const auto& [dist, cnt] : pgc_distance_vector_total)
    std::cout << "  distance " << dist << ": " << cnt << "\n";
  std::cout << "[SPP] l2c pgc distance:\n";
  for (const auto& [dist, cnt] : pgc_distance_vector_l2c)
    std::cout << "  distance " << dist << ": " << cnt << "\n";
  std::cout << "[SPP] llc pgc distance:\n";
  for (const auto& [dist, cnt] : pgc_distance_vector_llc)
    std::cout << "  distance " << dist << ": " << cnt << "\n";

  std::cout << "[SPP] total narrowly defined pgc distance:\n";
  for (const auto& [dist, cnt] : narrowly_defined_pgc_distance_vector_total)
    std::cout << "  distance " << dist << ": " << cnt << "\n";
  std::cout << "[SPP] l2c narrowly defined pgc distance:\n";
  for (const auto& [dist, cnt] : narrowly_defined_pgc_distance_vector_l2c)
    std::cout << "  distance " << dist << ": " << cnt << "\n";
  std::cout << "[SPP] llc narrowly defined pgc distance:\n";
  for (const auto& [dist, cnt] : narrowly_defined_pgc_distance_vector_llc)
    std::cout << "  distance " << dist << ": " << cnt << "\n";
}

// TODO: Find a good 64-bit hash function
uint64_t spp_pgc_ideal::get_hash(uint64_t key)
{
  // Robert Jenkins' 32 bit mix function
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);

  // Knuth's multiplicative method
  key = (key >> 3) * 2654435761;

  return key;
}

void spp_pgc_ideal::SIGNATURE_TABLE::read_and_update_sig(champsim::address addr, uint32_t& last_sig, uint32_t& curr_sig,
                                                         typename offset_type::difference_type& delta)
{
  auto set = get_hash(spp_pgc_ideal::unit_number{addr}.to<uint64_t>()) % ST_SET;
  auto match = ST_WAY;
  tag_type partial_page{addr};
  offset_type unit_offset{addr};
  uint8_t ST_hit = 0;
  long sig_delta = 0;

  if constexpr (SPP_DEBUG_PRINT) {
    std::cout << "[ST] " << __func__ << " page: " << champsim::page_number{addr} << " partial_page: " << std::hex << partial_page << std::dec << std::endl;
  }

  // Case 2: Invalid
  if (match == ST_WAY) {
    for (match = 0; match < ST_WAY; match++) {
      if (valid[set][match] && (tag[set][match] == partial_page)) {
        last_sig = sig[set][match];
        delta = champsim::offset(last_offset[set][match], unit_offset);

        if (delta) {
          // Build a new sig based on 7-bit sign magnitude representation of delta
          // sig_delta = (delta < 0) ? ((((-1) * delta) & 0x3F) + 0x40) : delta;

          // magnitude: clamp to 6-bit (0..63)
          long mag = std::labs((long)delta);
          const long mag_max = (1L << (SIG_DELTA_BIT - 1)) - 1; // 63 when SIG_DELTA_BIT=7
          if (mag > mag_max)
            mag = mag_max;

          // sign-magnitude encoding
          sig_delta = (delta < 0) ? (mag | (1L << (SIG_DELTA_BIT - 1))) : mag;

          sig[set][match] = ((last_sig << SIG_SHIFT) ^ sig_delta) & SIG_MASK;
          curr_sig = sig[set][match];
          last_offset[set][match] = unit_offset;

          if constexpr (SPP_DEBUG_PRINT) {
            std::cout << "[ST] " << __func__ << " hit set: " << set << " way: " << match;
            std::cout << " valid: " << valid[set][match] << " tag: " << std::hex << tag[set][match];
            std::cout << " last_sig: " << last_sig << " curr_sig: " << curr_sig;
            std::cout << " delta: " << std::dec << delta << " last_offset: " << unit_offset << std::endl;
          }
        } else
          last_sig = 0; // Hitting the same cache line, delta is zero

        ST_hit = 1;
        break;
      }
    }
  }

  // Case 2: Invalid
  if (match == ST_WAY) {
    for (match = 0; match < ST_WAY; match++) {
      if (valid[set][match] == 0) {
        valid[set][match] = 1;
        tag[set][match] = partial_page;
        sig[set][match] = 0;
        curr_sig = sig[set][match];
        last_offset[set][match] = unit_offset;

        if constexpr (SPP_DEBUG_PRINT) {
          std::cout << "[ST] " << __func__ << " invalid set: " << set << " way: " << match;
          std::cout << " valid: " << valid[set][match] << " tag: " << std::hex << partial_page;
          std::cout << " sig: " << sig[set][match] << " last_offset: " << std::dec << unit_offset << std::endl;
        }

        break;
      }
    }
  }

  if constexpr (SPP_SANITY_CHECK) {
    // Assertion
    if (match == ST_WAY) {
      for (match = 0; match < ST_WAY; match++) {
        if (lru[set][match] == ST_WAY - 1) { // Find replacement victim
          tag[set][match] = partial_page;
          sig[set][match] = 0;
          curr_sig = sig[set][match];
          last_offset[set][match] = unit_offset;

          if constexpr (SPP_DEBUG_PRINT) {
            std::cout << "[ST] " << __func__ << " miss set: " << set << " way: " << match;
            std::cout << " valid: " << valid[set][match] << " victim tag: " << std::hex << tag[set][match] << " new tag: " << partial_page;
            std::cout << " sig: " << sig[set][match] << " last_offset: " << std::dec << unit_offset << std::endl;
          }

          break;
        }
      }

      // Assertion
      if (match == ST_WAY) {
        std::cout << "[ST] Cannot find a replacement victim!" << std::endl;
        assert(0);
      }
    }
  }

  if constexpr (GHR_ON) {
    if (ST_hit == 0) {
      uint32_t GHR_found = _parent->GHR.check_entry(unit_offset);
      if (GHR_found < MAX_GHR_ENTRY) {
        sig_delta = (_parent->GHR.delta[GHR_found] < 0) ? (((-1) * _parent->GHR.delta[GHR_found]) + (1 << (SIG_DELTA_BIT - 1))) : _parent->GHR.delta[GHR_found];
        sig[set][match] = ((_parent->GHR.sig[GHR_found] << SIG_SHIFT) ^ sig_delta) & SIG_MASK;
        curr_sig = sig[set][match];
      }
    }
  }

  // Update LRU
  for (uint32_t way = 0; way < ST_WAY; way++) {
    if (lru[set][way] < lru[set][match]) {
      lru[set][way]++;

      if constexpr (SPP_SANITY_CHECK) {
        // Assertion
        if (lru[set][way] >= ST_WAY) {
          std::cout << "[ST] LRU value is wrong! set: " << set << " way: " << way << " lru: " << lru[set][way] << std::endl;
          assert(0);
        }
      }
    }
  }

  lru[set][match] = 0; // Promote to the MRU position
}

void spp_pgc_ideal::PATTERN_TABLE::update_pattern(uint32_t last_sig, typename offset_type::difference_type curr_delta)
{
  // Update (sig, delta) correlation
  uint32_t set = get_hash(last_sig) % PT_SET, match = 0;

  // Case 1: Hit
  for (match = 0; match < PT_WAY; match++) {
    if (delta[set][match] == curr_delta) {
      c_delta[set][match]++;
      c_sig[set]++;
      if (c_sig[set] > C_SIG_MAX) {
        for (uint32_t way = 0; way < PT_WAY; way++)
          c_delta[set][way] >>= 1;
        c_sig[set] >>= 1;
      }

      if constexpr (SPP_DEBUG_PRINT) {
        std::cout << "[PT] " << __func__ << " hit sig: " << std::hex << last_sig << std::dec << " set: " << set << " way: " << match;
        std::cout << " delta: " << delta[set][match] << " c_delta: " << c_delta[set][match] << " c_sig: " << c_sig[set] << std::endl;
      }

      break;
    }
  }

  // Case 2: Miss
  if (match == PT_WAY) {
    uint32_t victim_way = PT_WAY, min_counter = C_SIG_MAX;

    for (match = 0; match < PT_WAY; match++) {
      if (c_delta[set][match] < min_counter) { // Select an entry with the minimum c_delta
        victim_way = match;
        min_counter = c_delta[set][match];
      }
    }

    delta[set][victim_way] = curr_delta;
    c_delta[set][victim_way] = 0;
    c_sig[set]++;
    if (c_sig[set] > C_SIG_MAX) {
      for (uint32_t way = 0; way < PT_WAY; way++)
        c_delta[set][way] >>= 1;
      c_sig[set] >>= 1;
    }

    if constexpr (SPP_DEBUG_PRINT) {
      std::cout << "[PT] " << __func__ << " miss sig: " << std::hex << last_sig << std::dec << " set: " << set << " way: " << victim_way;
      std::cout << " delta: " << delta[set][victim_way] << " c_delta: " << c_delta[set][victim_way] << " c_sig: " << c_sig[set] << std::endl;
    }

    if constexpr (SPP_SANITY_CHECK) {
      // Assertion
      if (victim_way == PT_WAY) {
        std::cout << "[PT] Cannot find a replacement victim!" << std::endl;
        assert(0);
      }
    }
  }
}

void spp_pgc_ideal::PATTERN_TABLE::read_pattern(uint32_t curr_sig, std::vector<typename offset_type::difference_type>& delta_q,
                                                std::vector<uint32_t>& confidence_q, uint32_t& lookahead_way, uint32_t& lookahead_conf, uint32_t& pf_q_tail,
                                                uint32_t& depth)
{
  // Update (sig, delta) correlation
  uint32_t set = get_hash(curr_sig) % PT_SET, local_conf = 0, pf_conf = 0, max_conf = 0;

  if (c_sig[set]) {
    for (uint32_t way = 0; way < PT_WAY; way++) {
      if (pf_q_tail >= delta_q.size()) {
        break;
      }

      local_conf = (100 * c_delta[set][way]) / c_sig[set];
      pf_conf = depth ? (_parent->GHR.global_accuracy * c_delta[set][way] / c_sig[set] * lookahead_conf / 100) : local_conf;

      if (pf_conf >= PF_THRESHOLD) {
        confidence_q[pf_q_tail] = pf_conf;
        delta_q[pf_q_tail] = delta[set][way];

        // Lookahead path follows the most confident entry
        if (pf_conf > max_conf) {
          lookahead_way = way;
          max_conf = pf_conf;
        }
        pf_q_tail++;

        if constexpr (SPP_DEBUG_PRINT) {
          std::cout << "[PT] " << __func__ << " HIGH CONF: " << pf_conf << " sig: " << std::hex << curr_sig << std::dec << " set: " << set << " way: " << way;
          std::cout << " delta: " << delta[set][way] << " c_delta: " << c_delta[set][way] << " c_sig: " << c_sig[set];
          std::cout << " conf: " << local_conf << " depth: " << depth << std::endl;
        }
      } else {
        if constexpr (SPP_DEBUG_PRINT) {
          std::cout << "[PT] " << __func__ << "  LOW CONF: " << pf_conf << " sig: " << std::hex << curr_sig << std::dec << " set: " << set << " way: " << way;
          std::cout << " delta: " << delta[set][way] << " c_delta: " << c_delta[set][way] << " c_sig: " << c_sig[set];
          std::cout << " conf: " << local_conf << " depth: " << depth << std::endl;
        }
      }
    }

    lookahead_conf = max_conf;
    if (lookahead_conf >= PF_THRESHOLD)
      depth++;

    if constexpr (SPP_DEBUG_PRINT) {
      std::cout << "global_accuracy: " << _parent->GHR.global_accuracy << " lookahead_conf: " << lookahead_conf << std::endl;
    }
  } else {
    confidence_q[pf_q_tail] = 0;
  }
}

bool spp_pgc_ideal::PREFETCH_FILTER::check(champsim::address check_addr, FILTER_REQUEST filter_request)
{
  champsim::block_number cache_line{check_addr};
  auto hash = get_hash(cache_line.to<uint64_t>());
  auto quotient = (hash >> REMAINDER_BIT) & ((1 << QUOTIENT_BIT) - 1);
  auto remainder = hash % (1 << REMAINDER_BIT);

  if constexpr (SPP_DEBUG_PRINT) {
    std::cout << "[FILTER] check_addr: " << check_addr << " hash: " << hash << " quotient: " << quotient << " remainder: " << remainder << std::endl;
  }

  switch (filter_request) {
  case spp_pgc_ideal::SPP_L2C_PREFETCH:
    if ((valid[quotient] || useful[quotient]) && remainder_tag[quotient] == remainder) {
      if constexpr (SPP_DEBUG_PRINT) {
        std::cout << "[FILTER] " << __func__ << " line is already in the filter check_addr: " << check_addr << " cache_line: " << cache_line;
        std::cout << " quotient: " << quotient << " valid: " << valid[quotient] << " useful: " << useful[quotient] << std::endl;
      }

      return false; // False return indicates "Do not prefetch"
    } else {
      valid[quotient] = 1;  // Mark as prefetched
      useful[quotient] = 0; // Reset useful bit
      remainder_tag[quotient] = remainder;

      if constexpr (SPP_DEBUG_PRINT) {
        std::cout << "[FILTER] " << __func__ << " set valid for check_addr: " << check_addr << " cache_line: " << cache_line;
        std::cout << " quotient: " << quotient << " remainder_tag: " << remainder_tag[quotient] << " valid: " << valid[quotient]
                  << " useful: " << useful[quotient] << std::endl;
      }
    }
    break;

  case spp_pgc_ideal::SPP_LLC_PREFETCH:
    if ((valid[quotient] || useful[quotient]) && remainder_tag[quotient] == remainder) {
      if constexpr (SPP_DEBUG_PRINT) {
        std::cout << "[FILTER] " << __func__ << " line is already in the filter check_addr: " << check_addr << " cache_line: " << cache_line;
        std::cout << " quotient: " << quotient << " valid: " << valid[quotient] << " useful: " << useful[quotient] << std::endl;
      }

      return false; // False return indicates "Do not prefetch"
    } else {
      // NOTE: SPP_LLC_PREFETCH has relatively low confidence (FILL_THRESHOLD <= SPP_LLC_PREFETCH < PF_THRESHOLD)
      // Therefore, it is safe to prefetch this cache line in the large LLC and save precious L2C capacity
      // If this prefetch request becomes more confident and SPP eventually issues SPP_L2C_PREFETCH,
      // we can get this cache line immediately from the LLC (not from DRAM)
      // To allow this fast prefetch from LLC, SPP does not set the valid bit for SPP_LLC_PREFETCH

      // valid[quotient] = 1;
      // useful[quotient] = 0;

      if constexpr (SPP_DEBUG_PRINT) {
        std::cout << "[FILTER] " << __func__ << " don't set valid for check_addr: " << check_addr << " cache_line: " << cache_line;
        std::cout << " quotient: " << quotient << " valid: " << valid[quotient] << " useful: " << useful[quotient] << std::endl;
      }
    }
    break;

  case spp_pgc_ideal::L2C_DEMAND:
    if ((remainder_tag[quotient] == remainder) && (useful[quotient] == 0)) {
      useful[quotient] = 1;
      if (valid[quotient]) {
        _parent->GHR.pf_useful++; // This cache line was prefetched by SPP and actually used in the program

        // If this slot is derived from PGC, then count up PGC-useful.
        _parent->count_map["useful_prefetch_l2c"]++;
        if (is_pgc[quotient]) {
          _parent->count_map["useful_pgc_l2c"]++;
          if (is_narrowly_defined_pgc[quotient]) {
            _parent->count_map["useful_narrowly_defined_pgc_l2c"]++;
          }
        }
      }

      if constexpr (SPP_DEBUG_PRINT) {
        std::cout << "[FILTER] " << __func__ << " set useful for check_addr: " << check_addr << " cache_line: " << cache_line;
        std::cout << " quotient: " << quotient << " valid: " << valid[quotient] << " useful: " << useful[quotient];
        std::cout << " GHR.pf_issued: " << _parent->GHR.pf_issued << " GHR.pf_useful: " << _parent->GHR.pf_useful << std::endl;
      }
    }
    break;

  case spp_pgc_ideal::L2C_EVICT:
    // Decrease global pf_useful counter when there is a useless prefetch (prefetched but not used)
    if (valid[quotient] && !useful[quotient] && _parent->GHR.pf_useful)
      _parent->GHR.pf_useful--;

    // Reset filter entry
    valid[quotient] = 0;
    useful[quotient] = 0;
    remainder_tag[quotient] = 0;
    is_pgc[quotient] = 0;
    is_narrowly_defined_pgc[quotient] = 0;
    break;

  default:
    // Assertion
    std::cout << "[FILTER] Invalid filter request type: " << filter_request << std::endl;
    assert(0);
  }

  return true;
}

void spp_pgc_ideal::GLOBAL_REGISTER::update_entry(uint32_t pf_sig, uint32_t pf_confidence, offset_type pf_offset,
                                                  typename offset_type::difference_type pf_delta)
{
  // NOTE: GHR implementation is slightly different from the original paper
  // Instead of matching (last_offset + delta), GHR simply stores and matches the pf_offset
  uint32_t min_conf = 100, victim_way = MAX_GHR_ENTRY;

  if constexpr (SPP_DEBUG_PRINT) {
    std::cout << "[GHR] Crossing the page boundary pf_sig: " << std::hex << pf_sig << std::dec;
    std::cout << " confidence: " << pf_confidence << " pf_offset: " << pf_offset << " pf_delta: " << pf_delta << std::endl;
  }

  for (uint32_t i = 0; i < MAX_GHR_ENTRY; i++) {
    // if (sig[i] == pf_sig) { // TODO: Which one is better and consistent?
    //  If GHR already holds the same pf_sig, update the GHR entry with the latest info
    if (valid[i] && (offset[i] == pf_offset)) {
      // If GHR already holds the same pf_offset, update the GHR entry with the latest info
      sig[i] = pf_sig;
      confidence[i] = pf_confidence;
      // offset[i] = pf_offset;
      delta[i] = pf_delta;

      if constexpr (SPP_DEBUG_PRINT) {
        std::cout << "[GHR] Found a matching index: " << i << std::endl;
      }

      return;
    }

    // GHR replacement policy is based on the stored confidence value
    // An entry with the lowest confidence is selected as a victim
    if (confidence[i] < min_conf) {
      min_conf = confidence[i];
      victim_way = i;
    }
  }

  // Assertion
  if (victim_way >= MAX_GHR_ENTRY) {
    std::cout << "[GHR] Cannot find a replacement victim!" << std::endl;
    assert(0);
  }

  if constexpr (SPP_DEBUG_PRINT) {
    std::cout << "[GHR] Replace index: " << victim_way << " pf_sig: " << std::hex << sig[victim_way] << std::dec;
    std::cout << " confidence: " << confidence[victim_way] << " pf_offset: " << offset[victim_way] << " pf_delta: " << delta[victim_way] << std::endl;
  }

  valid[victim_way] = 1;
  sig[victim_way] = pf_sig;
  confidence[victim_way] = pf_confidence;
  offset[victim_way] = pf_offset;
  delta[victim_way] = pf_delta;
}

uint32_t spp_pgc_ideal::GLOBAL_REGISTER::check_entry(offset_type unit_offset)
{
  uint32_t max_conf = 0, max_conf_way = MAX_GHR_ENTRY;

  for (uint32_t i = 0; i < MAX_GHR_ENTRY; i++) {
    if ((offset[i] == unit_offset) && (max_conf < confidence[i])) {
      max_conf = confidence[i];
      max_conf_way = i;
    }
  }

  return max_conf_way;
}
