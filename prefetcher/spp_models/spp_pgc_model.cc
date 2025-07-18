/* SPP [http://ieeexplore.ieee.org/document/7783763/] */

#include <bits/stdc++.h>

using namespace std;

class Table
{
public:
  Table(int width, int height) : width(width), height(height), cells(height, vector<string>(width)) {}

  void set_row(int row, const vector<string>& data, int start_col = 0)
  {
    assert(data.size() + start_col == this->width);
    for (unsigned col = start_col; col < this->width; col += 1)
      this->set_cell(row, col, data[col]);
  }

  void set_col(int col, const vector<string>& data, int start_row = 0)
  {
    assert(data.size() + start_row == this->height);
    for (unsigned row = start_row; row < this->height; row += 1)
      this->set_cell(row, col, data[row]);
  }

  void set_cell(int row, int col, string data)
  {
    assert(0 <= row && row < (int)this->height);
    assert(0 <= col && col < (int)this->width);
    this->cells[row][col] = data;
  }

  void set_cell(int row, int col, double data)
  {
    this->oss.str("");
    this->oss << setw(11) << fixed << setprecision(8) << data;
    this->set_cell(row, col, this->oss.str());
  }

  void set_cell(int row, int col, int64_t data)
  {
    this->oss.str("");
    this->oss << setw(11) << std::left << data;
    this->set_cell(row, col, this->oss.str());
  }

  void set_cell(int row, int col, int data) { this->set_cell(row, col, (int64_t)data); }

  void set_cell(int row, int col, uint64_t data) { this->set_cell(row, col, (int64_t)data); }

  string to_string()
  {
    vector<int> widths;
    for (unsigned i = 0; i < this->width; i += 1) {
      int max_width = 0;
      for (unsigned j = 0; j < this->height; j += 1)
        max_width = max(max_width, (int)this->cells[j][i].size());
      widths.push_back(max_width + 2);
    }
    string out;
    out += Table::top_line(widths);
    out += this->data_row(0, widths);
    for (unsigned i = 1; i < this->height; i += 1) {
      out += Table::mid_line(widths);
      out += this->data_row(i, widths);
    }
    out += Table::bot_line(widths);
    return out;
  }

  string data_row(int row, const vector<int>& widths)
  {
    string out;
    for (unsigned i = 0; i < this->width; i += 1) {
      string data = this->cells[row][i];
      data.resize(widths[i] - 2, ' ');
      out += " | " + data;
    }
    out += " |\n";
    return out;
  }

  static string top_line(const vector<int>& widths) { return Table::line(widths, "┌", "┬", "┐"); }

  static string mid_line(const vector<int>& widths) { return Table::line(widths, "├", "┼", "┤"); }

  static string bot_line(const vector<int>& widths) { return Table::line(widths, "└", "┴", "┘"); }

  static string line(const vector<int>& widths, string left, string mid, string right)
  {
    string out = " " + left;
    for (unsigned i = 0; i < widths.size(); i += 1) {
      int w = widths[i];
      for (int j = 0; j < w; j += 1)
        out += "─";
      if (i != widths.size() - 1)
        out += mid;
      else
        out += right;
    }
    return out + "\n";
  }

private:
  unsigned width;
  unsigned height;
  vector<vector<string>> cells;
  ostringstream oss;
};

template <class T>
class InfiniteCache
{
public:
  class Entry
  {
  public:
    uint64_t key;
    uint64_t index;
    uint64_t tag;
    bool valid;
    T data;
  };

  Entry* erase(uint64_t key)
  {
    Entry* entry = this->find(key);
    if (!entry)
      return nullptr;
    entry->valid = false;
    this->last_erased_entry = *entry;
    int num_erased = this->entries.erase(key);
    assert(num_erased == 1);
    return &this->last_erased_entry;
  }

  /**
   * @return The old state of the entry that was written to.
   */
  Entry insert(uint64_t key, const T& data)
  {
    Entry* entry = this->find(key);
    if (entry != nullptr) {
      Entry old_entry = *entry;
      entry->data = data;
      return old_entry;
    }
    entries[key] = {key, 0, key, true, data};
    return {};
  }

  Entry* find(uint64_t key)
  {
    auto it = this->entries.find(key);
    if (it == this->entries.end())
      return nullptr;
    Entry& entry = (*it).second;
    assert(entry.tag == key && entry.valid);
    return &entry;
  }

  /**
   * For debugging purposes.
   */
  string log(vector<string> headers, function<void(Entry&, Table&, int)> write_data)
  {
    Table table(headers.size(), entries.size() + 1);
    table.set_row(0, headers);
    unsigned i = 0;
    for (auto& x : this->entries)
      write_data(x.second, table, ++i);
    return table.to_string();
  }

  void set_debug_mode(bool enable) { this->debug = enable; }

protected:
  Entry last_erased_entry;
  unordered_map<uint64_t, Entry> entries;
  bool debug = false;
};

template <class T>
class SetAssociativeCache
{
public:
  class Entry
  {
  public:
    uint64_t key;
    uint64_t index;
    uint64_t tag;
    bool valid;
    T data;
  };

  SetAssociativeCache(int size, int num_ways)
      : size(size), num_ways(num_ways), num_sets(size / num_ways), entries(num_sets, vector<Entry>(num_ways)), cams(num_sets)
  {
    assert(size % num_ways == 0);
    for (int i = 0; i < num_sets; i += 1)
      for (int j = 0; j < num_ways; j += 1)
        entries[i][j].valid = false;
  }

  Entry* erase(uint64_t key)
  {
    Entry* entry = this->find(key);
    uint64_t index = key % this->num_sets;
    uint64_t tag = key / this->num_sets;
    auto& cam = cams[index];
    int num_erased = cam.erase(tag);
    if (entry)
      entry->valid = false;
    assert(entry ? num_erased == 1 : num_erased == 0);
    return entry;
  }

  /**
   * @return The old state of the entry that was written to.
   */
  Entry insert(uint64_t key, const T& data)
  {
    Entry* entry = this->find(key);
    if (entry != nullptr) {
      Entry old_entry = *entry;
      entry->data = data;
      return old_entry;
    }
    uint64_t index = key % this->num_sets;
    uint64_t tag = key / this->num_sets;
    vector<Entry>& set = this->entries[index];
    int victim_way = -1;
    for (int i = 0; i < this->num_ways; i += 1)
      if (!set[i].valid) {
        victim_way = i;
        break;
      }
    if (victim_way == -1) {
      victim_way = this->select_victim(index);
    }
    Entry& victim = set[victim_way];
    Entry old_entry = victim;
    victim = {key, index, tag, true, data};
    auto& cam = cams[index];
    if (old_entry.valid) {
      int num_erased = cam.erase(old_entry.tag);
      assert(num_erased == 1);
    }
    cam[tag] = victim_way;
    return old_entry;
  }

  Entry* find(uint64_t key)
  {
    uint64_t index = key % this->num_sets;
    uint64_t tag = key / this->num_sets;
    auto& cam = cams[index];
    if (cam.find(tag) == cam.end())
      return nullptr;
    int way = cam[tag];
    Entry& entry = this->entries[index][way];
    assert(entry.tag == tag && entry.valid);
    return &entry;
  }

  /**
   * For debugging purposes.
   */
  string log(vector<string> headers, function<void(Entry&, Table&, int)> write_data)
  {
    vector<Entry> valid_entries = this->get_valid_entries();
    Table table(headers.size(), valid_entries.size() + 1);
    table.set_row(0, headers);
    for (unsigned i = 0; i < valid_entries.size(); i += 1)
      write_data(valid_entries[i], table, i + 1);
    return table.to_string();
  }

  void set_debug_mode(bool enable) { this->debug = enable; }

protected:
  /**
   * @return The way of the selected victim.
   */
  virtual int select_victim(uint64_t index)
  {
    /* random eviction policy if not overriden */
    return rand() % this->num_ways;
  }

  vector<Entry> get_valid_entries()
  {
    vector<Entry> valid_entries;
    for (int i = 0; i < num_sets; i += 1)
      for (int j = 0; j < num_ways; j += 1)
        if (entries[i][j].valid)
          valid_entries.push_back(entries[i][j]);
    return valid_entries;
  }

  int size;
  int num_ways;
  int num_sets;
  vector<vector<Entry>> entries;
  vector<unordered_map<uint64_t, int>> cams;
  bool debug = false;
};

template <class T>
class LRUSetAssociativeCache : public SetAssociativeCache<T>
{
  typedef SetAssociativeCache<T> Super;

public:
  LRUSetAssociativeCache(int size, int num_ways) : Super(size, num_ways), lru(this->num_sets, vector<uint64_t>(num_ways)) {}

  void set_mru(uint64_t key) { *this->get_lru(key) = this->t++; }

  void set_lru(uint64_t key) { *this->get_lru(key) = 0; }

protected:
  /* @override */
  int select_victim(uint64_t index)
  {
    vector<uint64_t>& lru_set = this->lru[index];
    return min_element(lru_set.begin(), lru_set.end()) - lru_set.begin();
  }

  uint64_t* get_lru(uint64_t key)
  {
    uint64_t index = key % this->num_sets;
    uint64_t tag = key / this->num_sets;
    int way = this->cams[index][tag];
    return &this->lru[index][way];
  }

  vector<vector<uint64_t>> lru;
  uint64_t t = 0;
};

template <class T>
class NMRUSetAssociativeCache : public SetAssociativeCache<T>
{
  typedef SetAssociativeCache<T> Super;

public:
  NMRUSetAssociativeCache(int size, int num_ways) : Super(size, num_ways), mru(this->num_sets) {}

  void set_mru(uint64_t key)
  {
    uint64_t index = key % this->num_sets;
    uint64_t tag = key / this->num_sets;
    int way = this->cams[index][tag];
    this->mru[index] = way;
  }

protected:
  /* @override */
  int select_victim(uint64_t index)
  {
    int way = rand() % (this->num_ways - 1);
    if (way >= mru[index])
      way += 1;
    return way;
  }

  vector<int> mru;
};

template <class T>
class LRUFullyAssociativeCache : public LRUSetAssociativeCache<T>
{
  typedef LRUSetAssociativeCache<T> Super;

public:
  LRUFullyAssociativeCache(int size) : Super(size, size) {}
};

template <class T>
class NMRUFullyAssociativeCache : public NMRUSetAssociativeCache<T>
{
  typedef NMRUSetAssociativeCache<T> Super;

public:
  NMRUFullyAssociativeCache(int size) : Super(size, size) {}
};

template <class T>
class DirectMappedCache : public SetAssociativeCache<T>
{
  typedef SetAssociativeCache<T> Super;

public:
  DirectMappedCache(int size) : Super(size, 1) {}
};

/** End Of Cache Framework **/

class GlobalHistoryRegister
{
public:
  class Entry
  {
  public:
    uint64_t signature;
    float confidence;
    int delta;
    int last_offset;
  };

  GlobalHistoryRegister(int size, int blocks_in_page) : size(size), blocks_in_page(blocks_in_page)
  {
    if (this->debug) {
      cerr << "[GHR] constructed with size=" << size << ", blocks_in_page=" << blocks_in_page << endl;
    }
    assert(size != 0);
    assert(blocks_in_page != 0);
  }

  Entry* find(int initial_offset)
  {
    if (this->debug) {
      cerr << "[GHR] search for offset=" << initial_offset << endl;
    }
    for (unsigned i = 0; i < q.size(); i += 1) {
      int predicted_offset = (q[i].last_offset + q[i].delta + blocks_in_page) % blocks_in_page;
      if (initial_offset == predicted_offset) {
        return &q[i];
      }
    }
    return nullptr;
  }

  void insert(uint64_t signature, float confidence, int delta, int last_offset)
  {
    if (this->debug) {
      cerr << "[GHR] insert signature=" << bitset<12>(signature).to_string() << ", confidence=" << confidence << ", delta=" << delta
           << ", last_offset=" << last_offset << endl;
    }
    q.push_front({signature, confidence, delta, last_offset});
    if (q.size() > this->size)
      q.pop_back();
  }

  void set_debug_mode(bool enable) { this->debug = enable; }

private:
  unsigned size;
  int blocks_in_page;
  deque<Entry> q;
  bool debug = false;
};

class SignatureTableData
{
public:
  int last_offset;
  uint64_t signature;
};

class SignatureTable : public LRUFullyAssociativeCache<SignatureTableData>
{
  typedef LRUFullyAssociativeCache<SignatureTableData> Super;

public:
  template <class T>
  static T set_width(T x, int n)
  {
    return ((1ull << n) - 1ull) & x;
  }

  static uint64_t update_signature(uint64_t signature, int delta, int blocks_in_page)
  {
    assert(__builtin_popcount(blocks_in_page) == 1);
    int delta_width = __builtin_ctz(blocks_in_page) + 1;
    delta = set_width(delta, delta_width);
    signature = bitset<12>((signature << 3) ^ delta).to_ullong();
    return signature;
  }

  SignatureTable(int size, int blocks_in_page) : Super(size), blocks_in_page(blocks_in_page) {}

  /**
   * @return Returns an <old_signature, delta> pair.
   */
  pair<uint64_t, int> update(uint64_t page_number, int page_offset, GlobalHistoryRegister* global_history_register)
  {
    pair<uint64_t, int> sig_delta(0, 0);
    Entry* entry = this->find(page_number);
    if (entry) {
      this->set_mru(page_number);
      SignatureTableData& data = entry->data;
      /* We use a 7-bit sign+magnitude representation for both positive and negative deltas */
      int delta = page_offset - data.last_offset;
      sig_delta.first = data.signature;
      sig_delta.second = delta;
      /* don't update signature table if delta is zero */
      if (delta == 0)
        return sig_delta;
      data.signature = update_signature(data.signature, delta, this->blocks_in_page);
      data.last_offset = page_offset;
    } else {
      /* If we access a new page that is not currently tracked (i.e., a miss in the ST), SPP searches for a GHR
       * entry whose last offset and delta match the current offset value in the new page */
      uint64_t signature = 0;
      GlobalHistoryRegister::Entry* ghr_entry = global_history_register->find(page_offset);
      if (ghr_entry) {
        sig_delta.first = ghr_entry->signature;
        sig_delta.second = ghr_entry->delta;
        signature = update_signature(ghr_entry->signature, ghr_entry->delta, this->blocks_in_page);
        if (this->debug) {
          cerr << "[ST] found matching offset in GHR" << endl;
        }
      } else if (this->debug) {
        cerr << "[ST] no delta history found" << endl;
      }
      this->insert(page_number, {page_offset, signature});
      this->set_mru(page_number);
    }
    return sig_delta;
  }

  string log()
  {
    vector<string> headers({"Page Number", "Last Offset", "Signature"});
    return Super::log(headers, this->write_data);
  }

private:
  static void write_data(Entry& entry, Table& table, int row)
  {
    table.set_cell(row, 0, entry.key);
    table.set_cell(row, 1, entry.data.last_offset);
    table.set_cell(row, 2, bitset<12>(entry.data.signature).to_string());
  }

  int blocks_in_page;
};

class PatternTableData
{
public:
  int delta[4];
  uint64_t c_delta[4];
  uint64_t c_sig;
};

class PatternTable : public DirectMappedCache<PatternTableData>
{
  typedef DirectMappedCache<PatternTableData> Super;

public:
  PatternTable(int size, float prefetch_threshold) : Super(size), prefetch_threshold(prefetch_threshold)
  {
    assert(size != 0);
    assert(0.0 <= prefetch_threshold && prefetch_threshold <= 1.0);
  }

  void update(uint64_t signature, int delta)
  {
    if (this->debug) {
      cerr << "[PT] updating signature=" << bitset<12>(signature).to_string() << ", delta=" << delta << endl;
    }
    assert(delta != 0);
    Entry* entry = this->find(signature);
    if (entry) {
      PatternTableData& data = entry->data;
      int update_index = 0;
      for (int i = 0; i < 4; i += 1) {
        if (data.delta[i] == delta) {
          update_index = i;
          break;
        }
        if (data.c_delta[i] < data.c_delta[update_index])
          update_index = i;
      }
      desaturate_counters(data);
      data.delta[update_index] = delta;
      data.c_delta[update_index] += 1;
      data.c_sig += 1;
    } else {
      this->insert(signature, {{delta, 0, 0, 0}, {1, 0, 0, 0}, 1});
    }
  }

  /*
   * @return Returns vector of <delta, path_prob> pairs.
   */
  vector<pair<int, float>> get_prefetch_candidates(uint64_t signature, float path_confidence)
  {
    if (this->debug) {
      cerr << "[PT] finding prefetch candidates for signature=" << bitset<12>(signature).to_string() << " and path_confidence=" << path_confidence << endl;
    }
    assert(bitset<12>(signature).to_ullong() == signature);
    assert(0.0 <= path_confidence && path_confidence <= 1.0);
    vector<pair<int, float>> result;
    Entry* entry = this->find(signature);
    if (!entry)
      return result;
    PatternTableData& data = entry->data;
    for (int i = 0; i < 4; i += 1) {
      /* skip if invalid */
      if (data.delta[i] == 0)
        continue;
      float prob = path_confidence * data.c_delta[i] / data.c_sig;
      assert(0 <= prob && prob <= 1);
      if (prob >= this->prefetch_threshold)
        result.push_back(make_pair(data.delta[i], prob));
    }
    return result;
  }

  string log()
  {
    vector<string> headers({"Signature", "Delta[0]", "Count[0]", "Delta[1]", "Count[1]", "Delta[2]", "Count[2]", "Delta[3]", "Count[3]", "Sigma Count"});
    return Super::log(headers, this->write_data);
  }

private:
  static void write_data(Entry& entry, Table& table, int row)
  {
    table.set_cell(row, 0, bitset<12>(entry.key).to_string());
    for (int i = 0; i < 4; i += 1) {
      table.set_cell(row, 2 * i + 1, entry.data.delta[i]);
      table.set_cell(row, 2 * i + 2, entry.data.c_delta[i]);
    }
    table.set_cell(row, 9, entry.data.c_sig);
  }

  static void desaturate_counters(PatternTableData& data)
  {
    /* If either Cdelta or Csig saturates, all counters associated with that signature are right shifted by 1. */
    if (data.c_sig == 15) {
      data.c_sig >>= 1;
      for (int i = 0; i < 4; i += 1)
        data.c_delta[i] >>= 1;
    }
  }

  float prefetch_threshold;
};

class PrefetchFilterData
{
public:
  bool useful;
};

class PrefetchFilter : public DirectMappedCache<PrefetchFilterData>
{
  typedef DirectMappedCache<PrefetchFilterData> Super;

public:
  using Super::Super;
};

class spp_pgc
{
public:
  spp_pgc(int blocks_in_page = 64, int signature_table_size = 256, int pattern_table_size = 512, int prefetch_filter_size = 1024,
          int global_history_register_size = 8, float prefetch_threshold = 0.25)
      : blocks_in_page(blocks_in_page), signature_table(signature_table_size, blocks_in_page), pattern_table(pattern_table_size, prefetch_threshold),
        prefetch_filter(prefetch_filter_size), global_history_register(global_history_register_size, blocks_in_page)
  {
  }

  /**
   * @return A vector of block numbers that should be prefetched.
   */
  vector<uint64_t> access(uint64_t block_number)
  {
    uint64_t page_number = block_number / this->blocks_in_page;
    uint64_t initial_page_number = page_number;
    int page_offset = block_number % this->blocks_in_page;
    if (this->debug) {
      cerr << "[spp_pgc] block_number=" << block_number << endl;
      cerr << "[spp_pgc] page_number=" << page_number << endl;
      cerr << "[spp_pgc] page_offset=" << page_offset << endl;
    }
    auto sig_delta = this->signature_table.update(page_number, page_offset, &global_history_register);
    if (this->debug) {
      cerr << this->signature_table.log();
    }
    uint64_t signature = sig_delta.first;
    int delta = sig_delta.second;
    if (delta == 0)
      return vector<uint64_t>();
    this->pattern_table.update(signature, delta);
    if (this->debug) {
      cerr << this->pattern_table.log();
    }
    signature = SignatureTable::update_signature(signature, delta, this->blocks_in_page);
    float path_confidence = 1.0;
    vector<uint64_t> preds;
    unordered_set<uint64_t> preds_set;
    int current_offset = page_offset;
    int depth = 0;
    if (this->debug) {
      cerr << "[spp_pgc] start lookahead" << endl;
      cerr << "[spp_pgc] alpha=" << this->get_alpha() << endl;
    }
    while (true) {
      if (this->debug) {
        cerr << "[spp_pgc] depth=" << depth << endl;
        cerr << "[spp_pgc] current_offset=" << current_offset << endl;
        cerr << "[spp_pgc] path_confidence=" << path_confidence << endl;
        cerr << "[spp_pgc] signature=" << bitset<12>(signature).to_string() << endl;
      }
      /* vector of <delta, prob> pairs */
      auto prefetch_candidates = this->pattern_table.get_prefetch_candidates(signature, path_confidence);
      if (this->debug) {
        cerr << "[spp_pgc] prefetch_candidates=<";
        for (auto& x : prefetch_candidates)
          cerr << "(" << x.first << ", " << x.second << ")";
        cerr << ">" << endl;
      }
      if (prefetch_candidates.empty()) {
        if (this->debug) {
          cerr << "[spp_pgc] no more prefetch candidates, breaking loop" << endl;
        }
        break;
      }
      int max_index = 0;
      for (int i = 0; i < (int)prefetch_candidates.size(); i += 1) {
        int delta = prefetch_candidates[i].first;
        float prob = prefetch_candidates[i].second;
        if (prob > prefetch_candidates[max_index].second)
          max_index = i;

        // 有効なメモリ範囲の存在チェック
        int64_t next_block_number_signed = int64_t(page_number) * this->blocks_in_page + int64_t(current_offset) + int64_t(delta);
        if (next_block_number_signed < 0) {
          if (this->debug) {
            cerr << "[SPP_PGC] skip prefetch candidate whose block number < 0" << endl;
          }
          continue;
        }
        // TODO: メモリアドレス上限チェック(必要であれば)

        int next_offset = current_offset + delta;
        // if (this->is_inside_page(next_offset)) {
        //   preds.push_back(page_number * this->blocks_in_page + next_offset);
        // } else if (this->is_inside_page(current_offset)) {
        //   this->global_history_register.insert(signature, prob, delta, current_offset);
        // }

        int next_block_number = page_number * this->blocks_in_page + next_offset;
        if (preds_set.count(next_block_number) > 0 || this->prefetch_filter.find(next_block_number)) {
          continue;
        } // プリフェッチをフィルタリング

        preds.push_back(next_block_number);
        preds_set.insert(next_block_number);
        if (!this->is_inside_page(next_offset)) {
          cross_page_prefetch_count++;
          int page_distance = next_offset / blocks_in_page;
          if (next_offset < 0 && next_offset % blocks_in_page != 0) {
            page_distance -= 1;
          }
          pgc_distance_map[page_distance]++;
        } // lookaheadによってページ境界を超えた後もプリフェッチを継続させる
        total_prefetch_count++;
      }
      int delta = prefetch_candidates[max_index].first;
      float prob = prefetch_candidates[max_index].second;
      signature = SignatureTable::update_signature(signature, delta, this->blocks_in_page);
      current_offset += delta;
      path_confidence = this->get_alpha() * prob;
      depth += 1;
    }
    this->update_prefetch_filter(preds);
    return preds;
  }

  float get_alpha()
  {
    float alpha = 1.0 * this->c_useful / this->c_total;
    assert(alpha < 1.0);
    return alpha;
  }

  void prefetch_hit(uint64_t block_number)
  {
    PrefetchFilter::Entry* entry = this->prefetch_filter.find(block_number);
    if (entry && !entry->data.useful) {
      entry->data.useful = true;
      this->c_useful += 1;
    }
  }

  void set_debug_mode(bool enable)
  {
    this->debug = enable;
    this->global_history_register.set_debug_mode(enable);
    this->signature_table.set_debug_mode(enable);
    this->pattern_table.set_debug_mode(enable);
  }

  void prefetcher_final_stats() { cout << "出力関数テスト" << endl; }

private:
  void update_prefetch_filter(const vector<uint64_t>& prefetch_blocks)
  {
    for (auto& block_number : prefetch_blocks) {
      if (this->prefetch_filter.find(block_number))
        continue;
      PrefetchFilter::Entry entry = this->prefetch_filter.insert(block_number, {false});
      if (!entry.valid)
        this->c_total += 1;
      else if (entry.key != block_number && entry.data.useful)
        this->c_useful -= 1;
      assert(1 <= this->c_total && this->c_total <= 1025);
      assert(0 <= this->c_useful && this->c_useful <= 1024);
    }
  }

  bool is_inside_page(int page_offset) { return (0 <= page_offset && page_offset < this->blocks_in_page); }

  int blocks_in_page;
  /* The SPP module consists of three main structures (Signature Table, Pattern Table, and Prefetch Filter) and a
   * small Global History Register (GHR), which is used for cross-page boundary bootstrapped learning. */
  SignatureTable signature_table;
  PatternTable pattern_table;
  PrefetchFilter prefetch_filter;
  GlobalHistoryRegister global_history_register;
  /* global accuracy counters (10 bits) */
  uint64_t c_useful = 0;
  uint64_t c_total = 1;
  bool debug = false;

  // プリフェッチの総数をカウント
  uint64_t total_prefetch_count = 0;
  // PGCをカウント
  uint64_t cross_page_prefetch_count = 0;
  // PGCの距離を集計
  unordered_map<int, uint64_t> pgc_distance_map;
};

class CacheData
{
public:
  bool prefetch;
};

class Cache : public LRUSetAssociativeCache<CacheData>
{
  typedef LRUSetAssociativeCache<CacheData> Super;

public:
  Cache(int size, int block_size, int num_ways, int page_size, string name) : Super(size / block_size, num_ways), spp_module(page_size / block_size), name(name)
  {
    assert(size % block_size == 0);
    assert(__builtin_popcount(this->num_sets) == 1);
    assert(__builtin_popcount(this->num_ways) == 1);
  }

  void access(uint64_t address, string type)
  {
    this->stats["Accesses"] += 1;
    uint64_t block_number = address / block_size;
    Entry* entry = this->find(block_number);
    if (entry) {
      this->set_mru(block_number);
      if (entry->data.prefetch) {
        this->stats["Prefetch Hits"] += 1;
        spp_module.prefetch_hit(block_number);
        entry->data.prefetch = false;
      }
    } else {
      this->stats["Misses"] += 1;
      this->fetch(block_number, false);
    }
    if (type != "Read Request")
      return;
    vector<uint64_t> to_prefetch = spp_module.access(block_number);
    if (this->debug) {
      cerr << "[Cache] to_prefetch=<";
      for (auto& block_number : to_prefetch)
        cerr << "(" << block_number << ")";
      cerr << ">" << endl;
    }
    for (auto& block_number : to_prefetch)
      this->fetch(block_number, true);
  }

  vector<string> get_log_headers()
  {
    vector<string> log_headers = {"Name", "Alpha"};
    for (auto& x : this->stats)
      log_headers.push_back(x.first);
    return log_headers;
  }

  /**
   * Write log info in specified row of given table.
   */
  void log(Table& table, int row)
  {
    int col = 0;
    table.set_cell(row, col++, this->name);
    table.set_cell(row, col++, this->spp_module.get_alpha());
    for (auto& x : this->stats)
      table.set_cell(row, col++, x.second);
  }

  uint64_t get_accesses() { return this->stats["Accesses"]; }

  void set_debug_mode(bool enable)
  {
    this->spp_module.set_debug_mode(enable);
    this->debug = enable;
  }

private:
  void fetch(uint64_t block_number, bool prefetch)
  {
    if (this->find(block_number))
      return;
    if (prefetch)
      this->stats["Prefetches"] += 1;
    Entry old_entry = this->insert(block_number, {prefetch});
    this->set_mru(block_number);
    if (old_entry.valid && old_entry.data.prefetch)
      this->stats["Non-useful Prefetches"] += 1;
  }

  int block_size = 64;
  spp_pgc spp_module;
  string name;
  bool debug = false;
  unordered_map<string, uint64_t> stats = {{"Accesses", 0}, {"Misses", 0}, {"Prefetches", 0}, {"Prefetch Hits", 0}, {"Non-useful Prefetches", 0}};
};

// int main() {
//     /* debugging parameters */
//     const bool DEBUG = false;
//     const int DEBUG_START = 0;
//     const int DEBUG_END = 1E8;
//     /* == */
//     const int KB = 1024;
//     const int MB = 1024 * KB;
//     const int LOG_RATE = 1E6;
//     std::ios_base::sync_with_stdio(false);
//     vector<Cache> caches;
//     /* 4MB */
//     caches.emplace_back(Cache(4 * MB, 64, 16, 1 * KB, "4MB-1KB"));
//     caches.emplace_back(Cache(4 * MB, 64, 16, 2 * KB, "4MB-2KB"));
//     caches.emplace_back(Cache(4 * MB, 64, 16, 4 * KB, "4MB-4KB"));
//     caches.emplace_back(Cache(4 * MB, 64, 16, 8 * KB, "4MB-8KB"));
//     caches.emplace_back(Cache(4 * MB, 64, 16, 16 * KB, "4MB-16KB"));
//     /* 8MB */
//     caches.emplace_back(Cache(8 * MB, 64, 16, 1 * KB, "8MB-1KB"));
//     caches.emplace_back(Cache(8 * MB, 64, 16, 2 * KB, "8MB-2KB"));
//     caches.emplace_back(Cache(8 * MB, 64, 16, 4 * KB, "8MB-4KB"));
//     caches.emplace_back(Cache(8 * MB, 64, 16, 8 * KB, "8MB-8KB"));
//     caches.emplace_back(Cache(8 * MB, 64, 16, 16 * KB, "8MB-16KB"));
//     /* == */
//     if (DEBUG) {
//         /* only debug first cache for simplicity */
//         while (caches.size() > 1)
//             caches.pop_back();
//     }
//     Table table(caches[0].get_log_headers().size(), caches.size() + 1);
//     table.set_row(0, caches[0].get_log_headers());
//     uint64_t pc, address;
//     string type1, type2;
//     int core;
//     string event;
//     while (cin >> event) {
//         if (event != "LLC-Access")
//             continue;
//         cin >> pc >> address >> type1 >> type2 >> core;
//         for (unsigned i = 0; i < caches.size(); i += 1)
//             caches[i].access(address, type1 + " " + type2);
//         if (DEBUG && caches[0].get_accesses() >= DEBUG_START) {
//             caches[0].set_debug_mode(true);
//             caches[0].log(table, 1);
//             cerr << table.to_string() << endl;
//             if (caches[0].get_accesses() == DEBUG_END)
//                 return 0;
//             cerr << "─────────────────\n" << endl;
//             continue;
//         }
//         if (caches[0].get_accesses() % LOG_RATE == 0) {
//             for (unsigned i = 0; i < caches.size(); i += 1)
//                 caches[i].log(table, i + 1);
//             cout << table.to_string() << endl;
//         }
//     }
//     for (unsigned i = 0; i < caches.size(); i += 1)
//         caches[i].log(table, i + 1);
//     cout << table.to_string() << endl;
//     return 0;
// }
