/*
 * Copyright (c) 2012-2014,2017-2018,2020-2021 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __CPU_O3_LSQ_UNIT_HH__
#define __CPU_O3_LSQ_UNIT_HH__

#include <algorithm>
#include <cstring>
#include <map>
#include <memory>
#include <queue>

#include "arch/generic/debugfaults.hh"
#include "arch/generic/vec_reg.hh"
#include "base/circular_queue.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/lsq.hh"
#include "cpu/timebuf.hh"
#include "debug/HtmCpu.hh"
#include "debug/LSQUnit.hh"
#include "mem/packet.hh"
#include "mem/port.hh"

namespace gem5
{

struct BaseO3CPUParams;

namespace o3
{

class IEW;

/**
 * Class that implements the actual LQ and SQ for each specific
 * thread.  Both are circular queues; load entries are freed upon
 * committing, while store entries are freed once they writeback. The
 * LSQUnit tracks if there are memory ordering violations, and also
 * detects partial load to store forwarding cases (a store only has
 * part of a load's data) that requires the load to wait until the
 * store writes back. In the former case it holds onto the instruction
 * until the dependence unit looks at it, and in the latter it stalls
 * the LSQ until the store writes back. At that point the load is
 * replayed.
 */
class LSQUnit
{
  public:
    static constexpr auto MaxDataBytes = MaxVecRegLenInBytes;

    using LSQRequest = LSQ::LSQRequest;
  private:
    class LSQEntry
    {
      private:
        /** The instruction. */
        DynInstPtr _inst;
        /** The request. */
        LSQRequest* _request = nullptr;
        /** The size of the operation. */
        uint32_t _size = 0;
        /** Valid entry. */
        bool _valid = false;

      public:
        ~LSQEntry()
        {
            if (_request != nullptr) {
                _request->freeLSQEntry();
                _request = nullptr;
            }
        }

        void
        clear()
        {
            _inst = nullptr;
            if (_request != nullptr) {
                _request->freeLSQEntry();
            }
            _request = nullptr;
            _valid = false;
            _size = 0;
        }

        void
        set(const DynInstPtr& new_inst)
        {
            assert(!_valid);
            _inst = new_inst;
            _valid = true;
            _size = 0;
        }

        LSQRequest* request() { return _request; }
        void setRequest(LSQRequest* r) { _request = r; }
        bool hasRequest() { return _request != nullptr; }
        /** Member accessors. */
        /** @{ */
        bool valid() const { return _valid; }
        uint32_t& size() { return _size; }
        const uint32_t& size() const { return _size; }
        const DynInstPtr& instruction() const { return _inst; }
        /** @} */
    };

    class SQEntry : public LSQEntry
    {
      private:
        /** The store data. */
        char _data[MaxDataBytes];
        /** Whether or not the store can writeback. */
        bool _canWB = false;
        /** Whether or not the store is committed. */
        bool _committed = false;
        /** Whether or not the store is completed. */
        bool _completed = false;
        /** Does this request write all zeros and thus doesn't
         * have any data attached to it. Used for cache block zero
         * style instructs (ARM DC ZVA; ALPHA WH64)
         */
        bool _isAllZeros = false;

      public:
        static constexpr size_t DataSize = sizeof(_data);
        /** Constructs an empty store queue entry. */
        SQEntry()
        {
            std::memset(_data, 0, DataSize);
        }

        void set(const DynInstPtr& inst) { LSQEntry::set(inst); }

        void
        clear()
        {
            LSQEntry::clear();
            _canWB = _completed = _committed = _isAllZeros = false;
        }

        /** Member accessors. */
        /** @{ */
        bool& canWB() { return _canWB; }
        const bool& canWB() const { return _canWB; }
        bool& completed() { return _completed; }
        const bool& completed() const { return _completed; }
        bool& committed() { return _committed; }
        const bool& committed() const { return _committed; }
        bool& isAllZeros() { return _isAllZeros; }
        const bool& isAllZeros() const { return _isAllZeros; }
        char* data() { return _data; }
        const char* data() const { return _data; }
        /** @} */
    };
    using LQEntry = LSQEntry;

    /** Coverage of one address range with another */
    enum class AddrRangeCoverage
    {
        PartialAddrRangeCoverage, /* Two ranges partly overlap */
        FullAddrRangeCoverage, /* One range fully covers another */
        NoAddrRangeCoverage /* Two ranges are disjoint */
    };

  public:
    using LoadQueue = CircularQueue<LQEntry>;
    using StoreQueue = CircularQueue<SQEntry>;

  public:
    /** Constructs an LSQ unit. init() must be called prior to use. */
    LSQUnit(uint32_t lqEntries, uint32_t sqEntries);

    /** We cannot copy LSQUnit because it has stats for which copy
     * contructor is deleted explicitly. However, STL vector requires
     * a valid copy constructor for the base type at compile time.
     */
    LSQUnit(const LSQUnit &l): stats(nullptr)
    {
        panic("LSQUnit is not copy-able");
    }

    /** Initializes the LSQ unit with the specified number of entries. */
    void init(CPU *cpu_ptr, IEW *iew_ptr, const BaseO3CPUParams &params,
            LSQ *lsq_ptr, unsigned id);

    /** Returns the name of the LSQ unit. */
    std::string name() const;

    /** Sets the pointer to the dcache port. */
    void setDcachePort(RequestPort *dcache_port);

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /** Inserts an instruction. */
    void insert(const DynInstPtr &inst);
    /** Inserts a load instruction. */
    void insertLoad(const DynInstPtr &load_inst);
    /** Inserts a store instruction. */
    void insertStore(const DynInstPtr &store_inst);

    /** Check for ordering violations in the LSQ. For a store squash if we
     * ever find a conflicting load. For a load, only squash if we
     * an external snoop invalidate has been seen for that load address
     * @param load_idx index to start checking at
     * @param inst the instruction to check
     */
    Fault checkViolations(typename LoadQueue::iterator& loadIt,
            const DynInstPtr& inst);

    /** Check if an incoming invalidate hits in the lsq on a load
     * that might have issued out of order wrt another load beacuse
     * of the intermediate invalidate.
     */
    void checkSnoop(PacketPtr pkt);

    // [InvisiSpec] check whether current request will hit in the
    // spec buffer or not
    int checkSpecBuffHit(const RequestPtr req, const int req_idx);
    void setSpecBuffState(const RequestPtr req);

    /** Executes a load instruction. */
    Fault executeLoad(const DynInstPtr &inst);

    Fault executeLoad(int lq_idx) { panic("Not implemented"); return NoFault; }
    /** Executes a store instruction. */
    Fault executeStore(const DynInstPtr &inst);

    /** Commits the head load. */
    void commitLoad();
    /** Commits loads older than a specific sequence number. */
    void commitLoads(InstSeqNum &youngest_inst);

    /** Commits stores older than a specific sequence number. */
    void commitStores(InstSeqNum &youngest_inst);

    /** Writes back stores. */
    void writebackStores();

    /** Completes the data access that has been returned from the
     * memory system. */
    void completeDataAccess(PacketPtr pkt);

    /** Squashes all instructions younger than a specific sequence number. */
    void squash(const InstSeqNum &squashed_num);

    /** Returns if there is a memory ordering violation. Value is reset upon
     * call to getMemDepViolator().
     */
    bool violation() { return memDepViolator; }

    /** Returns the memory ordering violator. */
    DynInstPtr getMemDepViolator();

    /** Returns the number of free LQ entries. */
    unsigned numFreeLoadEntries();

    /** Returns the number of free SQ entries. */
    unsigned numFreeStoreEntries();

    /** Returns the number of loads in the LQ. */
    int numLoads() { return loadQueue.size(); }

    /** Returns the number of stores in the SQ. */
    int numStores() { return storeQueue.size(); }

    // hardware transactional memory
    int numHtmStarts() const { return htmStarts; }
    int numHtmStops() const { return htmStops; }
    void resetHtmStartsStops() { htmStarts = htmStops = 0; }
    uint64_t getLatestHtmUid() const;
    void
    setLastRetiredHtmUid(uint64_t htm_uid)
    {
        assert(htm_uid >= lastRetiredHtmUid);
        lastRetiredHtmUid = htm_uid;
    }

    // Stale translation checks
    void startStaleTranslationFlush();
    bool checkStaleTranslations() const;

    /** Returns if either the LQ or SQ is full. */
    bool isFull() { return lqFull() || sqFull(); }

    /** Returns if both the LQ and SQ are empty. */
    bool isEmpty() const { return lqEmpty() && sqEmpty(); }

    /** Returns if the LQ is full. */
    bool lqFull() { return loadQueue.full(); }

    /** Returns if the SQ is full. */
    bool sqFull() { return storeQueue.full(); }

    /** Returns if the LQ is empty. */
    bool lqEmpty() const { return loadQueue.size() == 0; }

    /** Returns if the SQ is empty. */
    bool sqEmpty() const { return storeQueue.size() == 0; }

    /** Returns the number of instructions in the LSQ. */
    unsigned getCount() { return loadQueue.size() + storeQueue.size(); }

    /** Returns if there are any stores to writeback. */
    bool hasStoresToWB() { return storesToWB; }

    /** Returns the number of stores to writeback. */
    int numStoresToWB() { return storesToWB; }

    /** [InvisiSpec] Returns the number of loads to validate. */
    int numLoadsToVLD() { return loadsToVLD; }

    /** Returns if the LSQ unit will writeback on this cycle. */
    bool
    willWB()
    {
        return storeWBIt.dereferenceable() &&
                        storeWBIt->valid() &&
                        storeWBIt->canWB() &&
                        !storeWBIt->completed() &&
                        !isStoreBlocked;
    }

    /** Handles doing the retry. */
    void recvRetry();

    unsigned int cacheLineSize();
  private:
    /** Reset the LSQ state */
    void resetState();

    /** Writes back the instruction, sending it to IEW. */
    void writeback(const DynInstPtr &inst, PacketPtr pkt);

    // [InvisiSpec] complete Validates
    void completeValidate(DynInstPtr &inst, PacketPtr pkt);

    /** Try to finish a previously blocked write back attempt */
    void writebackBlockedStore();

    /** Completes the store at the specified index. */
    void completeStore(typename StoreQueue::iterator store_idx);

    /** Handles completing the send of a store to memory. */
    void storePostSend();

  public:
    /** Attempts to send a packet to the cache.
     * Check if there are ports available. Return true if
     * there are, false if there are not.
     */
    bool trySendPacket(bool isLoad, PacketPtr data_pkt);


    /** Debugging function to dump instructions in the LSQ. */
    void dumpInsts() const;

    /** Schedule event for the cpu. */
    void schedule(Event& ev, Tick when);

    BaseMMU *getMMUPtr();

  private:
    /** Pointer to the CPU. */
    CPU *cpu;

    /** Pointer to the IEW stage. */
    IEW *iewStage;

    /** Pointer to the LSQ. */
    LSQ *lsq;

    /** Pointer to the dcache port.  Used only for sending. */
    RequestPort *dcachePort;

    /** Writeback event, specifically for when stores forward data to loads. */
    class WritebackEvent : public Event
    {
      public:
        /** Constructs a writeback event. */
        WritebackEvent(const DynInstPtr &_inst, PacketPtr pkt,
                LSQUnit *lsq_ptr);

        /** Processes the writeback event. */
        void process();

        /** Returns the description of this event. */
        const char *description() const;

      private:
        /** Instruction whose results are being written back. */
        DynInstPtr inst;

        /** The packet that would have been sent to memory. */
        PacketPtr pkt;

        /** The pointer to the LSQ unit that issued the store. */
        LSQUnit *lsqPtr;
    };

  public:
    /**
     * Handles writing back and completing the load or store that has
     * returned from memory.
     *
     * @param pkt Response packet from the memory sub-system
     */
    bool recvTimingResp(PacketPtr pkt);

  private:
    /** The LSQUnit thread id. */
    ThreadID lsqID;
  public:
    /** The store queue. */
    StoreQueue storeQueue;

    /** The load queue. */
    LoadQueue loadQueue;

  private:
    /** The number of places to shift addresses in the LSQ before checking
     * for dependency violations
     */
    unsigned depCheckShift;

    /** Should loads be checked for dependency issues */
    bool checkLoads;

    /** The number of store instructions in the SQ waiting to writeback. */
    int storesToWB;

    // hardware transactional memory
    // nesting depth
    int htmStarts;
    int htmStops;
    // sanity checks and debugging
    uint64_t lastRetiredHtmUid;

    /** The index of the first instruction that may be ready to be
     * written back, and has not yet been written back.
     */
    typename StoreQueue::iterator storeWBIt;

    /** [InvisiSpec] The number of used cache ports in this cycle by stores. */
    int usedStorePorts;

    /** Address Mask for a cache block (e.g. ~(cache_block_size-1)) */
    Addr cacheBlockMask;

    /** Wire to read information from the issue stage time queue. */
    typename TimeBuffer<IssueStruct>::wire fromIssue;

    /** Whether or not the LSQ is stalled. */
    bool stalled;
    /** The store that causes the stall due to partial store to load
     * forwarding.
     */
    InstSeqNum stallingStoreIsn;
    /** The index of the above store. */
    ssize_t stallingLoadIdx;

    /** The packet that needs to be retried. */
    PacketPtr retryPkt;

    /** Whehter or not a store is blocked due to the memory system. */
    bool isStoreBlocked;

    /** Whether or not a store is in flight. */
    bool storeInFlight;

    /** The oldest load that caused a memory ordering violation. */
    DynInstPtr memDepViolator;

    /** Flag for memory model. */
    bool needsTSO;

  protected:
    // Will also need how many read/write ports the Dcache has.  Or keep track
    // of that in stage that is one level up, and only call executeLoad/Store
    // the appropriate number of times.
    struct LSQUnitStats : public statistics::Group
    {
        LSQUnitStats(statistics::Group *parent);

        /** Total number of loads forwaded from LSQ stores. */
        statistics::Scalar forwLoads;

        /** Total number of squashed loads. */
        statistics::Scalar squashedLoads;

        /** Total number of responses from the memory system that are
         * ignored due to the instruction already being squashed. */
        statistics::Scalar ignoredResponses;

        /** Tota number of memory ordering violations. */
        statistics::Scalar memOrderViolation;

        /** Total number of squashed stores. */
        statistics::Scalar squashedStores;

        /** Number of loads that were rescheduled. */
        statistics::Scalar rescheduledLoads;

        /** Number of times the LSQ is blocked due to the cache. */
        statistics::Scalar blockedByCache;

        /** Distribution of cycle latency between the first time a load
         * is issued and its completion */
        statistics::Distribution loadToUse;
    } stats;

  public:
    /** Executes the load at the given index. */
    Fault read(LSQRequest *request, ssize_t load_idx);

    /** Executes the store at the given index. */
    Fault write(LSQRequest *requst, uint8_t *data, ssize_t store_idx);

    /** Returns the index of the head load instruction. */
    int getLoadHead() { return loadQueue.head(); }

    /** Returns the sequence number of the head load instruction. */
    InstSeqNum getLoadHeadSeqNum();

    /** Returns the index of the head store instruction. */
    int getStoreHead() { return storeQueue.head(); }
    /** Returns the sequence number of the head store instruction. */
    InstSeqNum getStoreHeadSeqNum();

    /** Returns whether or not the LSQ unit is stalled. */
    bool isStalled()  { return stalled; }
  public:
    typedef typename CircularQueue<LQEntry>::iterator LQIterator;
    typedef typename CircularQueue<SQEntry>::iterator SQIterator;
};

} // namespace o3
} // namespace gem5


// IMPORTANT: the function to issue packets, interact with memory [mengjia]
template <class Impl>
Fault
LSQUnit<Impl>::read(Request *req, Request *sreqLow, Request *sreqHigh,
                    int load_idx)
{
    DynInstPtr load_inst = loadQueue[load_idx];

    assert(load_inst);

    assert(!load_inst->isExecuted());

    // Make sure this isn't a strictly ordered load
    // A bit of a hackish way to get strictly ordered accesses to work
    // only if they're at the head of the LSQ and are ready to commit
    // (at the head of the ROB too).
    if (req->isStrictlyOrdered() &&
        (load_idx != loadHead || !load_inst->isAtCommit())) {
        iewStage->rescheduleMemInst(load_inst);
        ++lsqRescheduledLoads;
        DPRINTF(LSQUnit, "Strictly ordered load [sn:%lli] PC %s\n",
                load_inst->seqNum, load_inst->pcState());

        // Must delete request now that it wasn't handed off to
        // memory.  This is quite ugly.  @todo: Figure out the proper
        // place to really handle request deletes.
        delete req;
        if (TheISA::HasUnalignedMemAcc && sreqLow) {
            delete sreqLow;
            delete sreqHigh;
        }
        return std::make_shared<GenericISA::M5PanicFault>(
            "Strictly ordered load [sn:%llx] PC %s\n",
            load_inst->seqNum, load_inst->pcState());
    }

    // Check the SQ for any previous stores that might lead to forwarding
    // why we have store queue index for a load operation? [mengjia]
    int store_idx = load_inst->sqIdx;

    int store_size = 0;

    DPRINTF(LSQUnit, "Read called, load idx: %i, store idx: %i, "
            "storeHead: %i addr: %#x%s\n",
            load_idx, store_idx, storeHead, req->getPaddr(),
            sreqLow ? " split" : "");

    // LLSC: load-link/store-conditional [mengjia]
    if (req->isLLSC()) {
        assert(!sreqLow);
        // Disable recording the result temporarily.  Writing to misc
        // regs normally updates the result, but this is not the
        // desired behavior when handling store conditionals.
        load_inst->recordResult(false);
        TheISA::handleLockedRead(load_inst.get(), req);
        load_inst->recordResult(true);
    }

    // request to memory mapped register [mengjia]
    if (req->isMmappedIpr()) {
        assert(!load_inst->memData);
        load_inst->memData = new uint8_t[64];

        ThreadContext *thread = cpu->tcBase(lsqID);
        Cycles delay(0);

        PacketPtr data_pkt = new Packet(req, MemCmd::ReadReq);

        data_pkt->dataStatic(load_inst->memData);
        if (!TheISA::HasUnalignedMemAcc || !sreqLow) {
            delay = TheISA::handleIprRead(thread, data_pkt);
        } else {
            assert(sreqLow->isMmappedIpr() && sreqHigh->isMmappedIpr());
            PacketPtr fst_data_pkt = new Packet(sreqLow, MemCmd::ReadReq);
            PacketPtr snd_data_pkt = new Packet(sreqHigh, MemCmd::ReadReq);

            fst_data_pkt->dataStatic(load_inst->memData);
            snd_data_pkt->dataStatic(load_inst->memData + sreqLow->getSize());

            delay = TheISA::handleIprRead(thread, fst_data_pkt);
            Cycles delay2 = TheISA::handleIprRead(thread, snd_data_pkt);
            if (delay2 > delay)
                delay = delay2;

            delete sreqLow;
            delete sreqHigh;
            delete fst_data_pkt;
            delete snd_data_pkt;
        }
        WritebackEvent *wb = new WritebackEvent(load_inst, data_pkt, this);
        cpu->schedule(wb, cpu->clockEdge(delay));
        return NoFault;
    }

    while (store_idx != -1) {
        // End once we've reached the top of the LSQ
        if (store_idx == storeWBIdx) {
            break;
        }

        // Move the index to one younger
        if (--store_idx < 0)
            store_idx += SQEntries;

        assert(storeQueue[store_idx].inst);

        store_size = storeQueue[store_idx].size;

        if (!store_size || storeQueue[store_idx].inst->strictlyOrdered() ||
            (storeQueue[store_idx].req &&
             storeQueue[store_idx].req->isCacheMaintenance())) {
            // Cache maintenance instructions go down via the store
            // path but they carry no data and they shouldn't be
            // considered for forwarding
            continue;
        }

        assert(storeQueue[store_idx].inst->effAddrValid());

        // Check if the store data is within the lower and upper bounds of
        // addresses that the request needs.
        bool store_has_lower_limit =
            req->getVaddr() >= storeQueue[store_idx].inst->effAddr;
        bool store_has_upper_limit =
            (req->getVaddr() + req->getSize()) <=
            (storeQueue[store_idx].inst->effAddr + store_size);
        bool lower_load_has_store_part =
            req->getVaddr() < (storeQueue[store_idx].inst->effAddr +
                           store_size);
        bool upper_load_has_store_part =
            (req->getVaddr() + req->getSize()) >
            storeQueue[store_idx].inst->effAddr;

        // If the store's data has all of the data needed and the load isn't
        // LLSC, we can forward.
        if (store_has_lower_limit && store_has_upper_limit && !req->isLLSC()) {
            // Get shift amount for offset into the store's data.
            int shift_amt = req->getVaddr() - storeQueue[store_idx].inst->effAddr;

            // Allocate memory if this is the first time a load is issued.
            if (!load_inst->memData) {
                load_inst->memData = new uint8_t[req->getSize()];
            }
            if (storeQueue[store_idx].isAllZeros)
                memset(load_inst->memData, 0, req->getSize());
            else
                memcpy(load_inst->memData,
                    storeQueue[store_idx].data + shift_amt, req->getSize());

            DPRINTF(LSQUnit, "Forwarding from store idx %i to load to "
                    "addr %#x\n", store_idx, req->getVaddr());

            PacketPtr data_pkt = new Packet(req, MemCmd::ReadReq);
            data_pkt->dataStatic(load_inst->memData);

            WritebackEvent *wb = new WritebackEvent(load_inst, data_pkt, this);

            // We'll say this has a 1 cycle load-store forwarding latency
            // for now.
            // @todo: Need to make this a parameter.
            cpu->schedule(wb, curTick());

            // Don't need to do anything special for split loads.
            if (TheISA::HasUnalignedMemAcc && sreqLow) {
                delete sreqLow;
                delete sreqHigh;
            }

            ++lsqForwLoads;
            return NoFault;
        } else if (
                (!req->isLLSC() &&
                 ((store_has_lower_limit && lower_load_has_store_part) ||
                  (store_has_upper_limit && upper_load_has_store_part) ||
                  (lower_load_has_store_part && upper_load_has_store_part))) ||
                (req->isLLSC() &&
                 ((store_has_lower_limit || upper_load_has_store_part) &&
                  (store_has_upper_limit || lower_load_has_store_part)))) {
            // This is the partial store-load forwarding case where a store
            // has only part of the load's data and the load isn't LLSC or
            // the load is LLSC and the store has all or part of the load's
            // data

            // If it's already been written back, then don't worry about
            // stalling on it.
            if (storeQueue[store_idx].completed) {
                panic("Should not check one of these");
                continue;
            }

            // Must stall load and force it to retry, so long as it's the oldest
            // load that needs to do so.
            if (!stalled ||
                (stalled &&
                 load_inst->seqNum <
                 loadQueue[stallingLoadIdx]->seqNum)) {
                stalled = true;
                stallingStoreIsn = storeQueue[store_idx].inst->seqNum;
                stallingLoadIdx = load_idx;
            }

            // Tell IQ/mem dep unit that this instruction will need to be
            // rescheduled eventually
            iewStage->rescheduleMemInst(load_inst);
            load_inst->clearIssued();
            ++lsqRescheduledLoads;

            // Do not generate a writeback event as this instruction is not
            // complete.
            DPRINTF(LSQUnit, "Load-store forwarding mis-match. "
                    "Store idx %i to load addr %#x\n",
                    store_idx, req->getVaddr());

            // Must delete request now that it wasn't handed off to
            // memory.  This is quite ugly.  @todo: Figure out the
            // proper place to really handle request deletes.
            delete req;
            if (TheISA::HasUnalignedMemAcc && sreqLow) {
                delete sreqLow;
                delete sreqHigh;
            }

            return NoFault;
        }
    }

    // If there's no forwarding case, then go access memory
    DPRINTF(LSQUnit, "Doing memory access for inst [sn:%lli] PC %s\n",
            load_inst->seqNum, load_inst->pcState());


    // Allocate memory if this is the first time a load is issued.
    if (!load_inst->memData) {
        load_inst->memData = new uint8_t[req->getSize()];
    }

    // if we the cache is not blocked, do cache access
    bool completedFirst = false;

    PacketPtr data_pkt = NULL;
    PacketPtr fst_data_pkt = NULL;
    PacketPtr snd_data_pkt = NULL;

    // According to the isInsivisibleSpec variable to create
    // corresponding type of packets [mengjia]
    bool sendSpecRead = false;
    if(isInvisibleSpec){
        if(!load_inst->readyToExpose()){
            assert(!req->isLLSC());
            assert(!req->isStrictlyOrdered());
            assert(!req->isMmappedIpr());
            sendSpecRead = true;
            DPRINTF(LSQUnit, "send a spec read for inst [sn:%lli]\n",
                    load_inst->seqNum);
        }

    }

    assert( !(sendSpecRead && load_inst->isSpecCompleted()) &&
            "Sending specRead twice for the same load insts");

    if(sendSpecRead){
       data_pkt = Packet::createReadSpec(req);
    }else{
        data_pkt = Packet::createRead(req);
    }

    data_pkt->dataStatic(load_inst->memData);

    LSQSenderState *state = new LSQSenderState;
    state->isLoad = true;
    state->idx = load_idx;
    state->inst = load_inst;
    data_pkt->senderState = state;

    if (!TheISA::HasUnalignedMemAcc || !sreqLow) {
        // Point the first packet at the main data packet.
        fst_data_pkt = data_pkt;

        fst_data_pkt->setFirst();
        if (sendSpecRead){
            int src_idx = checkSpecBuffHit(req, load_idx);
            if (src_idx != -1) {
                if (allowSpecBuffHit){
                    data_pkt->setOnlyAccessSpecBuff();
                }
                data_pkt->srcIdx = src_idx;
                specBuffHits++;
            }else{
                specBuffMisses++;
            }
        }
        fst_data_pkt->reqIdx = load_idx;
    } else {
        // Create the split packets.
        if(sendSpecRead){

            fst_data_pkt = Packet::createReadSpec(sreqLow);
            int fst_src_idx = checkSpecBuffHit(sreqLow, load_idx);
            if ( fst_src_idx != -1 ) {
                if (allowSpecBuffHit){
                    fst_data_pkt->setOnlyAccessSpecBuff();
                }
                fst_data_pkt->srcIdx = fst_src_idx;
                specBuffHits++;
            } else {
                specBuffMisses++;
            }

            snd_data_pkt = Packet::createReadSpec(sreqHigh);
            int snd_src_idx = checkSpecBuffHit(sreqHigh, load_idx);
            if ( snd_src_idx != -1 ) {
                if (allowSpecBuffHit){
                    snd_data_pkt->setOnlyAccessSpecBuff();
                }
                snd_data_pkt->srcIdx = snd_src_idx;
                specBuffHits++;
            } else {
                specBuffMisses++;
            }
        }else{
            fst_data_pkt = Packet::createRead(sreqLow);
            snd_data_pkt = Packet::createRead(sreqHigh);
        }

        fst_data_pkt->setFirst();
        fst_data_pkt->dataStatic(load_inst->memData);
        snd_data_pkt->dataStatic(load_inst->memData + sreqLow->getSize());

        fst_data_pkt->senderState = state;
        snd_data_pkt->senderState = state;
        fst_data_pkt->reqIdx = load_idx;
        snd_data_pkt->reqIdx = load_idx;

        fst_data_pkt->isSplit = true;
        snd_data_pkt->isSplit = true;
        state->isSplit = true;
        state->outstanding = 2;
        state->mainPkt = data_pkt;
    }

    // For now, load throughput is constrained by the number of
    // load FUs only, and loads do not consume a cache port (only
    // stores do).
    // @todo We should account for cache port contention
    // and arbitrate between loads and stores.
    bool successful_load = true;
    // MARK: here is the place memory request of read is sent [mengjia]
    // [InvisiSpec] Sending out a memory request
    if (!dcachePort->sendTimingReq(fst_data_pkt)) {
        successful_load = false;
    } else if (TheISA::HasUnalignedMemAcc && sreqLow) {
        completedFirst = true;

        // The first packet was sent without problems, so send this one
        // too. If there is a problem with this packet then the whole
        // load will be squashed, so indicate this to the state object.
        // The first packet will return in completeDataAccess and be
        // handled there.
        // @todo We should also account for cache port contention
        // here.
        if (!dcachePort->sendTimingReq(snd_data_pkt)) {
            // The main packet will be deleted in completeDataAccess.
            state->complete();
            // Signify to 1st half that the 2nd half was blocked via state
            state->cacheBlocked = true;
            successful_load = false;
        }
    }

    // If the cache was blocked, or has become blocked due to the access,
    // handle it.
    if (!successful_load) {
        if (!sreqLow) {
            // Packet wasn't split, just delete main packet info
            delete state;
            delete req;
            delete data_pkt;
        }

        if (TheISA::HasUnalignedMemAcc && sreqLow) {
            if (!completedFirst) {
                // Split packet, but first failed.  Delete all state.
                delete state;
                delete req;
                delete data_pkt;
                delete fst_data_pkt;
                delete snd_data_pkt;
                delete sreqLow;
                delete sreqHigh;
                sreqLow = NULL;
                sreqHigh = NULL;
            } else {
                // Can't delete main packet data or state because first packet
                // was sent to the memory system
                delete data_pkt;
                delete req;
                delete sreqHigh;
                delete snd_data_pkt;
                sreqHigh = NULL;
            }
        }

        ++lsqCacheBlocked;

        iewStage->blockMemInst(load_inst);

        // No fault occurred, even though the interface is blocked.
        return NoFault;
    }

    DPRINTF(LSQUnit, "successfully sent out packet(s) for inst [sn:%lli]\n",
            load_inst->seqNum);
    // Set everything ready for expose/validation after the read is
    // successfully sent out
    if(sendSpecRead){ // sending actual request

            // [mengjia] Here we set the needExposeOnly flag
            if (needsTSO && !load_inst->isDataPrefetch()){
                // need to check whether previous load_instructions specComplete or not
                if ( checkPrevLoadsExecuted(load_idx) ){
                    load_inst->needExposeOnly(true);
                    DPRINTF(LSQUnit, "Set load PC %s, [sn:%lli] as "
                            "needExposeOnly\n",
                        load_inst->pcState(), load_inst->seqNum);
                } else {
                    DPRINTF(LSQUnit, "Set load PC %s, [sn:%lli] as "
                            "needValidation\n",
                        load_inst->pcState(), load_inst->seqNum);
                }
            }else{
                //if RC, always only need expose
                load_inst->needExposeOnly(true);
                DPRINTF(LSQUnit, "Set load PC %s, [sn:%lli] as needExposeOnly\n",
                    load_inst->pcState(), load_inst->seqNum);
            }

            load_inst->needPostFetch(true);
            assert(!req->isMmappedIpr());
            //save expose requestPtr
            if (TheISA::HasUnalignedMemAcc && sreqLow) {
                load_inst->postSreqLow = new Request(*sreqLow);
                load_inst->postSreqHigh = new Request(*sreqHigh);
                load_inst->postReq = NULL;
            }else{
                load_inst->postReq = new Request(*req);
                load_inst->postSreqLow = NULL;
                load_inst->postSreqHigh = NULL;
            }
            load_inst->needDeletePostReq(true);
            DPRINTF(LSQUnit, "created validation/expose"
                    " request for inst [sn:%lli]"
                    "req=%#x, reqLow=%#x, reqHigh=%#x\n",
                load_inst->seqNum, (Addr)(load_inst->postReq),
                (Addr)(load_inst->postSreqLow),
                (Addr)(load_inst->postSreqHigh));
    } else {
        load_inst->setExposeCompleted();
        load_inst->needPostFetch(false);
        if (TheISA::HasUnalignedMemAcc && sreqLow) {
            setSpecBuffState(sreqLow);
            setSpecBuffState(sreqHigh);
        } else {
            setSpecBuffState(req);
        }
    }

    return NoFault;
}

template <class Impl>
Fault
LSQUnit<Impl>::write(Request *req, Request *sreqLow, Request *sreqHigh,
                     uint8_t *data, int store_idx)
{
    assert(storeQueue[store_idx].inst);

    DPRINTF(LSQUnit, "Doing write to store idx %i, addr %#x"
            " | storeHead:%i [sn:%i]\n",
            store_idx, req->getPaddr(), storeHead,
            storeQueue[store_idx].inst->seqNum);

    storeQueue[store_idx].req = req;
    storeQueue[store_idx].sreqLow = sreqLow;
    storeQueue[store_idx].sreqHigh = sreqHigh;
    unsigned size = req->getSize();
    storeQueue[store_idx].size = size;
    bool store_no_data = req->getFlags() & Request::STORE_NO_DATA;
    storeQueue[store_idx].isAllZeros = store_no_data;
    assert(size <= sizeof(storeQueue[store_idx].data) || store_no_data);

    // Split stores can only occur in ISAs with unaligned memory accesses.  If
    // a store request has been split, sreqLow and sreqHigh will be non-null.
    if (TheISA::HasUnalignedMemAcc && sreqLow) {
        storeQueue[store_idx].isSplit = true;
    }

    if (!(req->getFlags() & Request::CACHE_BLOCK_ZERO) && \
        !req->isCacheMaintenance())
        memcpy(storeQueue[store_idx].data, data, size);

    // This function only writes the data to the store queue, so no fault
    // can happen here.
    return NoFault;
}

#endif // __CPU_O3_LSQ_UNIT_HH__
