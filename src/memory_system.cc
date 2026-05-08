#include "memory_system.h"
#include <common.h>
#include <cstddef>
#include <cstdint>
#include <ios>
#include <sys/types.h>
#include <utility>

namespace dramsim3 {
MemorySystem::MemorySystem(const std::string &config_file,
                           const std::string &output_dir,
                           std::function<void(uint64_t)> read_callback,
                           std::function<void(uint64_t)> write_callback)
    : config_(new Config(config_file, output_dir)) {
    // TODO: add compile-time option
    // TODO: ideal memory type?
    if (config_->IsHMC()) {
        dram_system_ = new HMCMemorySystem(*config_, output_dir, read_callback,
                                           write_callback);
    } else {
        dram_system_ = new JedecDRAMSystem(*config_, output_dir, read_callback,
                                           write_callback);
    }
}

MemorySystem::~MemorySystem() {
    delete (dram_system_);
    delete (config_);
}

void MemorySystem::GlobalToLocalAddr(uint64_t* channel, uint64_t* rank,
                                uint64_t* bankgroup, uint64_t* bank, 
                                uint64_t* local_addr, uint64_t hex_addr) {
    auto addr = config_->AddressMapping(hex_addr);
    (*channel) = static_cast<uint64_t>(addr.channel);
    (*rank) = static_cast<uint64_t>(addr.rank);
    (*bankgroup) = static_cast<uint64_t>(addr.bankgroup);
    (*bank) = static_cast<uint64_t>(addr.bank);
    uint64_t pos = count_ones(config_->co_mask);
    (*local_addr) = static_cast<uint64_t>((addr.row << pos) + addr.column);
    uint64_t r = addr.row;
    uint64_t c = addr.column;
    uint64_t laddr2 = (r << config_->ro_pos) + (c << config_->co_pos);
}

void MemorySystem::ClockTick() { dram_system_->ClockTick(); }

double MemorySystem::GetTCK() const { return config_->tCK; }

int MemorySystem::GetBusBits() const { return config_->bus_width; }

int MemorySystem::GetBurstLength() const { return config_->BL; }

int MemorySystem::GetQueueSize() const { return config_->trans_queue_size; }

void MemorySystem::RegisterCallbacks(
    std::function<void(uint64_t)> read_callback,
    std::function<void(uint64_t)> write_callback) {
    dram_system_->RegisterCallbacks(read_callback, write_callback);
}

bool MemorySystem::WillAcceptTransaction(uint64_t hex_addr,
                                         bool is_write) const {
    return dram_system_->WillAcceptTransaction(hex_addr, is_write);
}

bool MemorySystem::AddTransaction(uint64_t hex_addr, bool is_write, bool is_pim) {
    return dram_system_->AddTransaction(hex_addr, is_write, is_pim);
}

int MemorySystem::GetConfigParameter(std::string identifier) {
    if (identifier == "tCCD_L") {
        return config_->tCCD_L;
    }
    if (identifier == "tCCD_S") {
        return config_->tCCD_S;
    }
    if (identifier == "shift_bits") {
        return config_->shift_bits;
    }
    if (identifier == "gdl_width") {
        return config_->gdl_width;
    }
    if (identifier == "ro_mask") {
        return config_->ro_mask;
    }
    if (identifier == "co_mask") {
        return config_->co_mask;
    }
    if (identifier == "ba_mask") {
        return config_->ba_mask;
    }
    if (identifier == "bg_mask") {
        return config_->bg_mask;
    }
    if (identifier == "ra_mask") {
        return config_->ra_mask;
    }
    if (identifier == "ch_mask") {
        return config_->ch_mask;
    }
    if (identifier == "ro_pos") {
        return config_->ro_pos;
    }
    if (identifier == "co_pos") {
        return config_->co_pos;
    }
    if (identifier == "ba_pos") {
        return config_->ba_pos;
    }
    if (identifier == "bg_pos") {
        return config_->bg_pos;
    }
    if (identifier == "ra_pos") {
        return config_->ra_pos;
    }
    if (identifier == "ch_pos") {
        return config_->ch_pos;
    }
    if (identifier == "BIL") {
      return config_->BIL;
    }
    if (identifier == "tFAW") {
      return config_->tFAW;
    }
    if (identifier == "AL") {
      return config_->AL;
    }
    if (identifier == "CL") {
      return config_->CL;
    }
    if (identifier == "CWL") {
      return config_->CWL;
    }
    if (identifier == "tRCD") {
      return config_->tRCD;
    }
    if (identifier == "tRCDRD" || identifier == "tRCDWR") {
      std::cerr << "Failed to read config parameter " 
                << identifier << " reason: not implemented." 
                << std::endl;
      return -1;
    }
    if (identifier == "tRAS") {
      return config_->tRAS;
    }
    if (identifier == "tRP") {
      return config_->tRP;
    }
    if (identifier == "tRFC") {
      return config_->tRFC;
    }
    if (identifier == "tREFI") {
      return config_->tREFI;
    }
    if (identifier == "tRPRE") {
      return config_->tRPRE;
    }
    if (identifier == "tWPRE") {
      return config_->tWPRE;
    }
    if (identifier == "tWTR_S") {
      return config_->tWTR_S;
    }
    if (identifier == "tWTR_L") {
      return config_->tWTR_L;
    }
    if (identifier == "tWR") {
      return config_->tWR;
    }
    if (identifier == "tRRD_S") {
      return config_->tRRD_S;
    }
    if (identifier == "tRRD_L") {
      return config_->tRRD_L;
    }
    if (identifier == "tXS") {
      return config_->tXS;
    }
    if (identifier == "tXP") {
      return config_->tXP;
    }
    if (identifier == "tRFCb") {
      return config_->tRFCb;
    }
    if (identifier == "tREFIb") {
      return config_->tREFIb;
    }
    if (identifier == "tRTRS") {
      return config_->tRTRS;
    }
    if (identifier == "tRTP") {
      return config_->tRTP;
    }
    if (identifier == "n_col") {
      return config_->columns;
    }
    if (identifier == "n_row") {
      return config_->rows;
    }
    return -1;
}

uint64_t MemorySystem::BankLocalToGlobalAddr(uint64_t channel, uint64_t rank,
                                        uint64_t bankgroup, uint64_t bank,
                                        uint64_t hex_addr) {
    uint64_t pos = count_ones(config_->co_mask);
    uint64_t local_mask = (config_->ro_mask << pos) + config_->co_mask;
    if ((hex_addr & local_mask) != hex_addr) {
        std::cerr << "Bank address out of bounds: " 
                  << std::hex << hex_addr << " (local address) > " 
                  << local_mask << " (maximum allowed)" << std::endl;
        AbruptExit(__FILE__, __LINE__);
    }
    // bounds masking is unnecessary here because we just did so above
    uint64_t row = hex_addr >> pos;
    uint64_t col = hex_addr & (config_->co_mask);

    uint64_t global_addr = (channel & (config_->ch_mask)) << config_->ch_pos;
    global_addr += (rank & (config_->ra_mask)) << config_->ra_pos;
    global_addr += (bankgroup & (config_->bg_mask)) << config_->bg_pos;
    global_addr += (bank & (config_->ba_mask)) << config_->ba_pos;
    global_addr += row << config_->ro_pos;
    global_addr += col << config_->co_pos;
    global_addr <<= config_->shift_bits;
    return global_addr;
}

uint64_t MemorySystem::GetSpatialGlobalAddr(uint64_t channel, uint64_t rank,
                                        uint64_t bankgroup, uint64_t bank,
                                        uint64_t hex_addr) {
    uint64_t pos = count_ones(config_->co_mask);
    uint64_t local_mask = (config_->ro_mask << pos) + config_->co_mask;
    if ((hex_addr & local_mask) != hex_addr) {
        std::cerr << "Bank address out of bounds: " 
                  << std::hex << hex_addr << " (local address) > " 
                  << local_mask << " (maximum allowed)" << std::endl;
        AbruptExit(__FILE__, __LINE__);
    }
    // bounds masking is unnecessary here because we just did so above
    uint64_t row = hex_addr >> pos;
    uint64_t col = hex_addr & (config_->co_mask);

    uint64_t global_addr = col;
    global_addr += row << pos;
    pos += count_ones(config_->ro_mask);
    global_addr += (bank & (config_->ba_mask)) << pos;
    pos += count_ones(config_->ba_mask);
    global_addr += (bankgroup & (config_->bg_mask)) << pos;
    pos += count_ones(config_->bg_mask);
    global_addr += (rank & (config_-> ra_mask)) << pos;
    pos += count_ones(config_->ra_mask);
    global_addr += (channel & (config_->ch_mask)) << pos;
    return global_addr;
}

int MemorySystem::GetActiveRow(uint64_t channel, uint64_t rank, uint64_t bankgroup, uint64_t bank) {
  return dram_system_->GetActiveRow(channel, rank, bankgroup, bank);
}

void MemorySystem::PrintStats() const { dram_system_->PrintStats(); }

void MemorySystem::ResetStats() { dram_system_->ResetStats(); }

MemorySystem* GetMemorySystem(const std::string &config_file, const std::string &output_dir,
                 std::function<void(uint64_t)> read_callback,
                 std::function<void(uint64_t)> write_callback) {
    return new MemorySystem(config_file, output_dir, read_callback, write_callback);
}
}  // namespace dramsim3

// This function can be used by autoconf AC_CHECK_LIB since
// apparently it can't detect C++ functions.
// Basically just an entry in the symbol table
extern "C" {
void libdramsim3_is_present(void) { ; }
}
