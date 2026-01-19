#ifndef SPP_PGC_PTE_H
#define SPP_PGC_PTE_H

#include <cstdint>
#include <vector>

#include "cache.h"
#include "chrono.h"
#include "modules.h"
#include "msl/lru_table.h"
#include "vmem.h"

struct spp_pgc_pte : public champsim::modules::prefetcher {

  // SPP functional knobs
  constexpr static bool LOOKAHEAD_ON = true;
  constexpr static bool FILTER_ON = true;
  constexpr static bool GHR_ON = false;
  constexpr static bool SPP_SANITY_CHECK = true;
  constexpr static bool SPP_DEBUG_PRINT = false;

  // Signature table parameters
  constexpr static std::size_t ST_SET = 1;
  constexpr static std::size_t ST_WAY = 256;
  constexpr static unsigned ST_TAG_BIT = 16;
  constexpr static unsigned SIG_SHIFT = 3;
  constexpr static unsigned SIG_BIT = 12;
  constexpr static uint32_t SIG_MASK = ((1 << SIG_BIT) - 1);
  constexpr static unsigned SIG_DELTA_BIT = 7;
  // This parameter is used to change the size of signature table granularity.
  // The size of ST grain affects the accuracy of prefetch and ipc in result.
  constexpr static unsigned SIG_UNIT_BIT = 12;

  // Pattern table parameters
  constexpr static std::size_t PT_SET = 512;
  constexpr static std::size_t PT_WAY = 4;
  constexpr static unsigned C_SIG_BIT = 4;
  constexpr static unsigned C_DELTA_BIT = 4;
  constexpr static uint32_t C_SIG_MAX = ((1 << C_SIG_BIT) - 1);
  constexpr static uint32_t C_DELTA_MAX = ((1 << C_DELTA_BIT) - 1);

  // Prefetch filter parameters
  constexpr static unsigned QUOTIENT_BIT = 10;
  constexpr static unsigned REMAINDER_BIT = 6;
  constexpr static unsigned HASH_BIT = (QUOTIENT_BIT + REMAINDER_BIT + 1);
  constexpr static std::size_t FILTER_SET = (1 << QUOTIENT_BIT);
  constexpr static uint32_t FILL_THRESHOLD = 90;
  constexpr static uint32_t PF_THRESHOLD = 25;

  // Global register parameters
  constexpr static unsigned GLOBAL_COUNTER_BIT = 10;
  constexpr static uint32_t GLOBAL_COUNTER_MAX = ((1 << GLOBAL_COUNTER_BIT) - 1);
  constexpr static std::size_t MAX_GHR_ENTRY = 8;

  // PGC enabling flag
  constexpr static bool is_pgc_enabled = true;
  constexpr static bool can_get_pte_when_tlb_hit = false;

  // map to keep the translation data by cached ptes
  constexpr static std::size_t PTE_BUFFER_SET = 1; // PTE buffer is fully associative
  constexpr static std::size_t PTE_BUFFER_WAY = 16;
  struct pte_buffer_entry {
    champsim::pte_block_page_number pte_block_vpage_tag{0};
    std::array<champsim::page_number, BLOCK_SIZE / PTE_SIZE> pte_block_ppage_array;
    uint8_t valid_mask = 0;
    bool is_valid = false;

    auto index() const { return 0ULL; }
    auto tag() const { return pte_block_vpage_tag; }
  };
  struct pte_buffer_type : champsim::msl::lru_table<pte_buffer_entry> {
    pte_buffer_type() : champsim::msl::lru_table<pte_buffer_entry>(PTE_BUFFER_SET, PTE_BUFFER_WAY) {}
  };
  std::array<pte_buffer_type, NUM_CPUS> pte_buffer;
  std::pair<champsim::page_number, bool> va_to_pa_buffer(uint32_t cpu_num, champsim::page_number vpage);

  // Statistics variants for PGC simulation
  bool roi_stats_initialized = false;
  void reset_roi_status();
  std::unordered_map<std::string, uint64_t> count_map = {
      // prefetch_candidate_total = prefetch_candidate_l2c + prefetch_candidate_llc + trashed_prefetch_low_confidence
      {"prefetch_candidate_total", 0},        // total prefetch candidates read from Pattern Table
      {"prefetch_candidate_l2c", 0},          // prefetch candidates for L2C with higher confidence than l2c fill threshold
      {"prefetch_candidate_llc", 0},          // prefetch candidates for LLC with lower confidence than l2c fill threshold
      {"trashed_prefetch_low_confidence", 0}, // not requested prefetch candidates with lower confidence than llc fill threshold
      {"trashed_pgc_low_confidence", 0},      // not requested pgc candidates with lower confidence than llc fill threshold
      // trashed pgcs below are narrowly defined pgc
      {"trashed_va_discontinuous_pgc_l2c", 0}, // trashed l2c pgc request due to discontinuity on the virtual memory address
      {"trashed_va_discontinuous_pgc_llc", 0}, // trashed llc pgc request due to discontinuity on the virtual memory address
      // prefetch request
      {"prefetch_request_l2c", 0},
      {"prefetch_request_llc", 0},
      {"pgc_request_l2c", 0},
      {"pgc_request_llc", 0},
      {"narrowly_defined_pgc_request_l2c", 0},
      {"narrowly_defined_pgc_request_llc", 0},
      // prefetch issued
      {"prefetch_issued_l2c", 0},
      {"prefetch_issued_llc", 0},
      {"pgc_issued_l2c", 0},
      {"pgc_issued_llc", 0},
      {"narrowly_defined_pgc_issued_l2c", 0},
      {"narrowly_defined_pgc_issued_llc", 0},
      // prefetch useful
      // TODO: llc useful metrics are not implemented
      {"useful_prefetch_l2c", 0},
      {"useful_prefetch_llc", 0},
      {"useful_pgc_l2c", 0},
      {"useful_pgc_llc", 0},
      {"useful_narrowly_defined_pgc_l2c", 0},
      {"useful_narrowly_defined_pgc_llc", 0},
  };
  std::unordered_map<int, uint64_t> pgc_distance_map_l2c;
  std::unordered_map<int, uint64_t> pgc_distance_map_llc;
  std::unordered_map<int, uint64_t> narrowly_defined_pgc_distance_map_l2c;
  std::unordered_map<int, uint64_t> narrowly_defined_pgc_distance_map_llc;
  bool is_adjacent_in_virtual(uint32_t trigger_cpu, champsim::page_number trigger_vpage, champsim::page_number pf_ppage);

  using prefetcher::prefetcher;
  uint32_t prefetcher_cache_operate(uint32_t trigger_cpu, champsim::address trigger_paddr, champsim::address trigger_vaddr, champsim::address ip,
                                    bool cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);

  void prefetcher_initialize();
  void prefetcher_cycle_operate();
  void prefetcher_final_stats();

  enum FILTER_REQUEST { SPP_L2C_PREFETCH, SPP_LLC_PREFETCH, L2C_DEMAND, L2C_EVICT }; // Request type for prefetch filter
  static uint64_t get_hash(uint64_t key);

  struct unit_number_extent : champsim::dynamic_extent {
    unit_number_extent() : dynamic_extent(champsim::data::bits{std::numeric_limits<uint64_t>::digits}, champsim::data::bits{SIG_UNIT_BIT}) {}
  };
  using unit_number = champsim::address_slice<unit_number_extent>;

  struct block_in_unit_extent : champsim::dynamic_extent {
    block_in_unit_extent() : dynamic_extent(champsim::data::bits{SIG_UNIT_BIT}, champsim::data::bits{LOG2_BLOCK_SIZE}) {}
  };
  using offset_type = champsim::address_slice<block_in_unit_extent>;

  class SIGNATURE_TABLE
  {
    struct tag_extent : champsim::dynamic_extent {
      tag_extent() : dynamic_extent(champsim::data::bits{ST_TAG_BIT + SIG_UNIT_BIT}, champsim::data::bits{SIG_UNIT_BIT}) {}
    };

  public:
    spp_pgc_pte* _parent;
    using tag_type = champsim::address_slice<tag_extent>;

    bool valid[ST_SET][ST_WAY];
    tag_type tag[ST_SET][ST_WAY];
    offset_type last_offset[ST_SET][ST_WAY];
    uint32_t sig[ST_SET][ST_WAY], lru[ST_SET][ST_WAY];

    SIGNATURE_TABLE()
    {
      for (uint32_t set = 0; set < ST_SET; set++)
        for (uint32_t way = 0; way < ST_WAY; way++) {
          valid[set][way] = 0;
          tag[set][way] = tag_type{};
          last_offset[set][way] = offset_type{};
          sig[set][way] = 0;
          lru[set][way] = way;
        }
    };

    void read_and_update_sig(champsim::address addr, uint32_t& last_sig, uint32_t& curr_sig, typename offset_type::difference_type& delta);
  };

  class PATTERN_TABLE
  {
  public:
    spp_pgc_pte* _parent;
    typename offset_type::difference_type delta[PT_SET][PT_WAY];
    uint32_t c_delta[PT_SET][PT_WAY], c_sig[PT_SET];

    PATTERN_TABLE()
    {
      for (uint32_t set = 0; set < PT_SET; set++) {
        for (uint32_t way = 0; way < PT_WAY; way++) {
          delta[set][way] = 0;
          c_delta[set][way] = 0;
        }
        c_sig[set] = 0;
      }
    }

    void update_pattern(uint32_t last_sig, typename offset_type::difference_type curr_delta);
    void read_pattern(uint32_t curr_sig, std::vector<typename offset_type::difference_type>& prefetch_delta, std::vector<uint32_t>& confidence_q,
                      uint32_t& lookahead_way, uint32_t& lookahead_conf, uint32_t& pf_q_tail, uint32_t& depth);
  };

  class PREFETCH_FILTER
  {
  public:
    spp_pgc_pte* _parent;
    uint64_t remainder_tag[FILTER_SET];
    bool valid[FILTER_SET],                  // Consider this as "prefetched"
        useful[FILTER_SET],                  // Consider this as "used"
        is_pgc[FILTER_SET],                  // Consider this as "page-crossing prefetched"
        is_narrowly_defined_pgc[FILTER_SET]; // Consider this as "narrowly defined page-crossing prefetched"

    PREFETCH_FILTER()
    {
      for (uint32_t set = 0; set < FILTER_SET; set++) {
        remainder_tag[set] = 0;
        valid[set] = 0;
        useful[set] = 0;
        is_pgc[set] = 0;
        is_narrowly_defined_pgc[set] = 0;
      }
    }

    bool check(champsim::address pf_addr, FILTER_REQUEST filter_request);
  };

  class GLOBAL_REGISTER
  {
  public:
    spp_pgc_pte* _parent;
    // Global counters to calculate global prefetching accuracy
    uint32_t pf_useful, pf_issued;
    uint32_t global_accuracy; // Alpha value in Section III. Equation 3

    // Global History Register (GHR) entries
    uint8_t valid[MAX_GHR_ENTRY];
    uint32_t sig[MAX_GHR_ENTRY], confidence[MAX_GHR_ENTRY];
    offset_type offset[MAX_GHR_ENTRY];
    typename offset_type::difference_type delta[MAX_GHR_ENTRY];

    GLOBAL_REGISTER()
    {
      pf_useful = 0;
      pf_issued = 0;
      global_accuracy = 0;

      for (uint32_t i = 0; i < MAX_GHR_ENTRY; i++) {
        valid[i] = 0;
        sig[i] = 0;
        confidence[i] = 0;
        offset[i] = offset_type{};
        delta[i] = 0;
      }
    }

    void update_entry(uint32_t pf_sig, uint32_t pf_confidence, offset_type pf_offset, typename offset_type::difference_type pf_delta);
    uint32_t check_entry(offset_type page_offset);
  };

  SIGNATURE_TABLE ST;
  PATTERN_TABLE PT;
  PREFETCH_FILTER FILTER;
  GLOBAL_REGISTER GHR;
};

#endif
