/*
 * Copyright (c) 2017,2019-2021 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2011-2014 Mark D. Hill and David A. Wood
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

#include "mem/ruby/slicc_interface/AbstractController.hh"

#include "debug/RubyQueue.hh"
#include "debug/MemSpecBuffer.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/protocol/MemoryMsg.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "sim/system.hh"

namespace gem5
{

namespace ruby
{

AbstractController::AbstractController(const Params &p)
    : ClockedObject(p), Consumer(this), m_version(p.version),
      m_clusterID(p.cluster_id),
      m_id(p.system->getRequestorId(this)), m_is_blocking(false),
      m_number_of_TBEs(p.number_of_TBEs),
      m_transitions_per_cycle(p.transitions_per_cycle),
      m_buffer_size(p.buffer_size), m_recycle_latency(p.recycle_latency),
      m_mandatory_queue_latency(p.mandatory_queue_latency),
      m_waiting_mem_retry(false),
      memoryPort(csprintf("%s.memory", name()), this),
      addrRanges(p.addr_ranges.begin(), p.addr_ranges.end()),
      stats(this)
{
    if (m_version == 0) {
        // Combine the statistics from all controllers
        // of this particular type.
        statistics::registerDumpCallback([this]() { collateStats(); });
    }
}

void
AbstractController::init()
{
    stats.delayHistogram.init(10);
    uint32_t size = Network::getNumberOfVirtualNetworks();
    for (uint32_t i = 0; i < size; i++) {
        stats.delayVCHistogram.push_back(new statistics::Histogram(this));
        stats.delayVCHistogram[i]->init(10);
    }

    if (getMemReqQueue()) {
        getMemReqQueue()->setConsumer(this);
    }

    // Initialize the addr->downstream machine mappings. Multiple machines
    // in downstream_destinations can have the same address range if they have
    // different types. If this is the case, mapAddressToDownstreamMachine
    // needs to specify the machine type
    downstreamDestinations.resize();
    for (auto abs_cntrl : params().downstream_destinations) {
        MachineID mid = abs_cntrl->getMachineID();
        const AddrRangeList &ranges = abs_cntrl->getAddrRanges();
        for (const auto &addr_range : ranges) {
            auto i = downstreamAddrMap.intersects(addr_range);
            if (i == downstreamAddrMap.end()) {
                i = downstreamAddrMap.insert(addr_range, AddrMapEntry());
            }
            AddrMapEntry &entry = i->second;
            fatal_if(entry.count(mid.getType()) > 0,
                     "%s: %s mapped to multiple machines of the same type\n",
                     name(), addr_range.to_string());
            entry[mid.getType()] = mid;
        }
        downstreamDestinations.add(mid);
    }
    // Initialize the addr->upstream machine list.
    // We do not need to map address -> upstream machine,
    // so we don't examine the address ranges
    upstreamDestinations.resize();
    for (auto abs_cntrl : params().upstream_destinations) {
        upstreamDestinations.add(abs_cntrl->getMachineID());
    }
}

void
AbstractController::resetStats()
{
    stats.delayHistogram.reset();
    uint32_t size = Network::getNumberOfVirtualNetworks();
    for (uint32_t i = 0; i < size; i++) {
        stats.delayVCHistogram[i]->reset();
    }
}

void
AbstractController::regStats()
{
    ClockedObject::regStats();

    m_expose_hits
        .name(name() + ".expose_hits")
        .desc("number of expose hits at LLC spec buffer")
        .flags(Stats::nozero);
    m_expose_misses
        .name(name() + ".expose_misses")
        .desc("number of expose misses at LLC spec buffer")
        .flags(Stats::nozero);
}

void
AbstractController::profileMsgDelay(uint32_t virtualNetwork, Cycles delay)
{
    assert(virtualNetwork < stats.delayVCHistogram.size());
    stats.delayHistogram.sample(delay);
    stats.delayVCHistogram[virtualNetwork]->sample(delay);
}

void
AbstractController::stallBuffer(MessageBuffer* buf, Addr addr)
{
    if (m_waiting_buffers.count(addr) == 0) {
        MsgVecType* msgVec = new MsgVecType;
        msgVec->resize(m_in_ports, NULL);
        m_waiting_buffers[addr] = msgVec;
    }
    DPRINTF(RubyQueue, "stalling %s port %d addr %#x\n", buf, m_cur_in_port,
            addr);
    assert(m_in_ports > m_cur_in_port);
    (*(m_waiting_buffers[addr]))[m_cur_in_port] = buf;
}

void
AbstractController::wakeUpBuffer(MessageBuffer* buf, Addr addr)
{
    auto iter = m_waiting_buffers.find(addr);
    if (iter != m_waiting_buffers.end()) {
        bool has_other_msgs = false;
        MsgVecType* msgVec = iter->second;
        for (unsigned int port = 0; port < msgVec->size(); ++port) {
            if ((*msgVec)[port] == buf) {
                buf->reanalyzeMessages(addr, clockEdge());
                (*msgVec)[port] = NULL;
            } else if ((*msgVec)[port] != NULL) {
                has_other_msgs = true;
            }
        }
        if (!has_other_msgs) {
            delete msgVec;
            m_waiting_buffers.erase(iter);
        }
    }
}

void
AbstractController::wakeUpBuffers(Addr addr)
{
    if (m_waiting_buffers.count(addr) > 0) {
        //
        // Wake up all possible lower rank (i.e. lower priority) buffers that could
        // be waiting on this message.
        //
        for (int in_port_rank = m_cur_in_port - 1;
             in_port_rank >= 0;
             in_port_rank--) {
            if ((*(m_waiting_buffers[addr]))[in_port_rank] != NULL) {
                (*(m_waiting_buffers[addr]))[in_port_rank]->
                    reanalyzeMessages(addr, clockEdge());
            }
        }
        delete m_waiting_buffers[addr];
        m_waiting_buffers.erase(addr);
    }
}

void
AbstractController::wakeUpAllBuffers(Addr addr)
{
    if (m_waiting_buffers.count(addr) > 0) {
        //
        // Wake up all possible buffers that could be waiting on this message.
        //
        for (int in_port_rank = m_in_ports - 1;
             in_port_rank >= 0;
             in_port_rank--) {
            if ((*(m_waiting_buffers[addr]))[in_port_rank] != NULL) {
                (*(m_waiting_buffers[addr]))[in_port_rank]->
                    reanalyzeMessages(addr, clockEdge());
            }
        }
        delete m_waiting_buffers[addr];
        m_waiting_buffers.erase(addr);
    }
}

void
AbstractController::wakeUpAllBuffers()
{
    //
    // Wake up all possible buffers that could be waiting on any message.
    //

    std::vector<MsgVecType*> wokeUpMsgVecs;
    MsgBufType wokeUpMsgBufs;

    if (m_waiting_buffers.size() > 0) {
        for (WaitingBufType::iterator buf_iter = m_waiting_buffers.begin();
             buf_iter != m_waiting_buffers.end();
             ++buf_iter) {
             for (MsgVecType::iterator vec_iter = buf_iter->second->begin();
                  vec_iter != buf_iter->second->end();
                  ++vec_iter) {
                  //
                  // Make sure the MessageBuffer has not already be reanalyzed
                  //
                  if (*vec_iter != NULL &&
                      (wokeUpMsgBufs.count(*vec_iter) == 0)) {
                      (*vec_iter)->reanalyzeAllMessages(clockEdge());
                      wokeUpMsgBufs.insert(*vec_iter);
                  }
             }
             wokeUpMsgVecs.push_back(buf_iter->second);
        }

        for (std::vector<MsgVecType*>::iterator wb_iter = wokeUpMsgVecs.begin();
             wb_iter != wokeUpMsgVecs.end();
             ++wb_iter) {
             delete (*wb_iter);
        }

        m_waiting_buffers.clear();
    }
}

bool
AbstractController::serviceMemoryQueue()
{
    auto mem_queue = getMemReqQueue();
    assert(mem_queue);
    if (m_waiting_mem_retry || !mem_queue->isReady(clockEdge())) {
        return false;
    }

    const MemoryMsg *mem_msg = (const MemoryMsg*)mem_queue->peek();
    unsigned int req_size = RubySystem::getBlockSizeBytes();
    if (mem_msg->m_Len > 0) {
        req_size = mem_msg->m_Len;
    }

    RequestPtr req
        = std::make_shared<Request>(mem_msg->m_addr, req_size, 0, m_id);
    PacketPtr pkt;
    if (mem_msg->getType() == MemoryRequestType_MEMORY_WB) {
        pkt = Packet::createWrite(req);
        pkt->allocate();
        pkt->setData(mem_msg->m_DataBlk.getData(getOffset(mem_msg->m_addr),
            req_size));
    } else if (mem_msg->getType() == MemoryRequestType_MEMORY_READ) {
        pkt = Packet::createRead(req);
        uint8_t *newData = new uint8_t[req_size];
        pkt->dataDynamic(newData);
    } else {
        panic("Unknown memory request type (%s) for addr %p",
              MemoryRequestType_to_string(mem_msg->getType()),
              mem_msg->m_addr);
    }

    SenderState *s = new SenderState(mem_msg->m_Sender);
    pkt->pushSenderState(s);

    if (RubySystem::getWarmupEnabled()) {
        // Use functional rather than timing accesses during warmup
        mem_queue->dequeue(clockEdge());
        memoryPort.sendFunctional(pkt);
        // Since the queue was popped the controller may be able
        // to make more progress. Make sure it wakes up
        scheduleEvent(Cycles(1));
        recvTimingResp(pkt);
    } else if (memoryPort.sendTimingReq(pkt)) {
        mem_queue->dequeue(clockEdge());
        // Since the queue was popped the controller may be able
        // to make more progress. Make sure it wakes up
        scheduleEvent(Cycles(1));
    } else {
        scheduleEvent(Cycles(1));
        m_waiting_mem_retry = true;
        delete pkt;
        delete s;
    }

    return true;
}

void
AbstractController::blockOnQueue(Addr addr, MessageBuffer* port)
{
    m_is_blocking = true;
    m_block_map[addr] = port;
}

bool
AbstractController::isBlocked(Addr addr) const
{
    return m_is_blocking && (m_block_map.find(addr) != m_block_map.end());
}

void
AbstractController::unblock(Addr addr)
{
    m_block_map.erase(addr);
    if (m_block_map.size() == 0) {
       m_is_blocking = false;
    }
}

bool
AbstractController::isBlocked(Addr addr)
{
    return (m_block_map.count(addr) > 0);
}

Port &
AbstractController::getPort(const std::string &if_name, PortID idx)
{
    return memoryPort;
}

void
AbstractController::functionalMemoryRead(PacketPtr pkt)
{
    // read from mem. req. queue if write data is pending there
    MessageBuffer *req_queue = getMemReqQueue();
    if (!req_queue || !req_queue->functionalRead(pkt))
        memoryPort.sendFunctional(pkt);
}

int
AbstractController::functionalMemoryWrite(PacketPtr pkt)
{
    int num_functional_writes = 0;

    // Update memory itself.
    memoryPort.sendFunctional(pkt);
    return num_functional_writes + 1;
}

// [InvisiSpec]
void
AbstractController::queueMemoryRead(const MachineID &id, Addr addr,
                                    Cycles latency, MachineID origin, int idx, int type)
{
    int coreId = origin.num;
    int sbeId = idx;
    // type 0: non-spec 1: spec 2: expose
    // DPRINTFR(MemSpecBuffer, "%10s MemRead (core=%d, type=%d, idx=%d, addr=%#x)\n", curTick(), coreId, type, sbeId, printAddress(addr));
    // if idx == -1, it is a write request which cannot be spec or expose.
    assert(!(type != 0 && sbeId == -1));
    assert(sbeId >= -1 && sbeId <= 65);
    assert(coreId < 8);
    assert(type >=0 && type <= 2);
    if (type == 0) {
        for (int c = 0; c < 8; ++c) {
            for (int i = 0; i < 66; ++i) {
                if (m_specBuf[c][i].address == addr) {
                    DPRINTFR(MemSpecBuffer, "%10s Cleared by Read (core=%d, type=%d, idx=%d, addr=%#x)\n", curTick(), c, type, i, printAddress(addr));
                    m_specBuf[c][i].address = 0;
                    m_specBuf[c][i].data.clear();
                }
            }
        }
    } else if (type == 1) {

    } else if (type == 2) {
        if (m_specBuf[coreId][sbeId].address == addr) {
            DPRINTFR(MemSpecBuffer, "%10s Expose Hit (core=%d, type=%d, idx=%d, addr=%#x)\n", curTick(), coreId, type, sbeId, printAddress(addr));
            ++m_expose_hits;
            assert(getMemoryQueue());
            std::shared_ptr<MemoryMsg> msg = std::make_shared<MemoryMsg>(clockEdge());
            (*msg).m_addr = addr;
            (*msg).m_Sender = m_machineID;
            (*msg).m_OriginalRequestorMachId = id;
            (*msg).m_Type = MemoryRequestType_MEMORY_READ;
            (*msg).m_MessageSize = MessageSizeType_Response_Data;
            (*msg).m_DataBlk = m_specBuf[coreId][sbeId].data;
            getMemoryQueue()->enqueue(msg, clockEdge(), cyclesToTicks(Cycles(1)));
            for (int c = 0; c < 8; ++c) {
                for (int i = 0; i < 66; ++i) {
                    if (m_specBuf[c][i].address == addr) {
                        DPRINTFR(MemSpecBuffer, "%10s Cleared by Expose Hit (core=%d, type=%d, idx=%d, addr=%#x)\n", curTick(), c, type, i, printAddress(addr));
                        m_specBuf[c][i].address = 0;
                        m_specBuf[c][i].data.clear();
                    }
                }
            }
            return;
        } else {
            DPRINTFR(MemSpecBuffer, "%10s Expose Miss (core=%d, type=%d, idx=%d, addr=%#x)\n", curTick(), coreId, type, sbeId, printAddress(addr));
            ++m_expose_misses;
            for (int c = 0; c < 8; ++c) {
                for (int i = 0; i < 66; ++i) {
                    if (m_specBuf[c][i].address == addr) {
                        DPRINTFR(MemSpecBuffer, "%10s Cleared by Expose Miss (core=%d, type=%d, idx=%d, addr=%#x)\n", curTick(), c, type, i, printAddress(addr));
                        m_specBuf[c][i].address = 0;
                        m_specBuf[c][i].data.clear();
                    }
                }
            }
        }
    }
    
    RequestPtr req = new Request(addr, RubySystem::getBlockSizeBytes(), 0,
                                 m_masterId);

    PacketPtr pkt = Packet::createRead(req);
    uint8_t *newData = new uint8_t[RubySystem::getBlockSizeBytes()];
    pkt->dataDynamic(newData);

    SenderState *s = new SenderState(id);
    s->type = type;
    s->coreId = coreId;
    s->sbeId = sbeId;
    pkt->pushSenderState(s);

    // Use functional rather than timing accesses during warmup
    if (RubySystem::getWarmupEnabled()) {
        memoryPort.sendFunctional(pkt);
        recvTimingResp(pkt);
        return;
    }

    memoryPort.schedTimingReq(pkt, clockEdge(latency));
}

void
AbstractController::recvTimingResp(PacketPtr pkt)
{
    assert(getMemoryQueue());
    assert(pkt->isResponse());

    std::shared_ptr<MemoryMsg> msg = std::make_shared<MemoryMsg>(clockEdge());
    (*msg).m_addr = pkt->getAddr();
    (*msg).m_Sender = m_machineID;

    SenderState *s = dynamic_cast<SenderState *>(pkt->senderState);
    (*msg).m_OriginalRequestorMachId = s->id;
    int type = s->type;
    int coreId = s->coreId;
    int sbeId = s->sbeId;
    delete s;

    if (pkt->isRead()) {
        (*msg).m_Type = MemoryRequestType_MEMORY_READ;
        (*msg).m_MessageSize = MessageSizeType_Response_Data;

        // Copy data from the packet
        (*msg).m_DataBlk.setData(pkt->getPtr<uint8_t>(), 0,
                                 RubySystem::getBlockSizeBytes());
        if (type == 1) {
            DPRINTFR(MemSpecBuffer, "%10s Updated by ReadSpec (core=%d, type=%d, idx=%d, addr=%#x)\n", curTick(), coreId, type, sbeId, printAddress(pkt->getAddr()));
            m_specBuf[coreId][sbeId].address = pkt->getAddr();
            m_specBuf[coreId][sbeId].data.setData(pkt->getPtr<uint8_t>(), 0,
                                                  RubySystem::getBlockSizeBytes());
        }
    } else if (pkt->isWrite()) {
        (*msg).m_Type = MemoryRequestType_MEMORY_WB;
        (*msg).m_MessageSize = MessageSizeType_Writeback_Control;
    } else {
        panic("Incorrect packet type received from memory controller!");
    }

    getMemoryQueue()->enqueue(msg, clockEdge(), cyclesToTicks(Cycles(1)));
    delete pkt->req;
    delete pkt;
}

Tick
AbstractController::recvAtomic(PacketPtr pkt)
{
   return ticksToCycles(memoryPort.sendAtomic(pkt));
}

MachineID
AbstractController::mapAddressToMachine(Addr addr, MachineType mtype) const
{
    NodeID node = m_net_ptr->addressToNodeID(addr, mtype);
    MachineID mach = {mtype, node};
    return mach;
}

MachineID
AbstractController::mapAddressToDownstreamMachine(Addr addr, MachineType mtype)
const
{
    const auto i = downstreamAddrMap.contains(addr);
    fatal_if(i == downstreamAddrMap.end(),
      "%s: couldn't find mapping for address %x\n", name(), addr);

    const AddrMapEntry &entry = i->second;
    assert(!entry.empty());

    if (mtype == MachineType_NUM) {
        fatal_if(entry.size() > 1,
          "%s: address %x mapped to multiple machine types.\n", name(), addr);
        return entry.begin()->second;
    } else {
        auto j = entry.find(mtype);
        fatal_if(j == entry.end(),
          "%s: couldn't find mapping for address %x\n", name(), addr);
        return j->second;
    }
}


bool
AbstractController::MemoryPort::recvTimingResp(PacketPtr pkt)
{
    controller->recvTimingResp(pkt);
    return true;
}

void
AbstractController::MemoryPort::recvReqRetry()
{
    controller->m_waiting_mem_retry = false;
    controller->serviceMemoryQueue();
}

AbstractController::MemoryPort::MemoryPort(const std::string &_name,
                                           AbstractController *_controller,
                                           PortID id)
    : RequestPort(_name, _controller, id), controller(_controller)
{
}

AbstractController::
ControllerStats::ControllerStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(fullyBusyCycles,
               "cycles for which number of transistions == max transitions"),
      ADD_STAT(delayHistogram, "delay_histogram")
{
    fullyBusyCycles
        .flags(statistics::nozero);
    delayHistogram
        .flags(statistics::nozero);
}

} // namespace ruby
} // namespace gem5
