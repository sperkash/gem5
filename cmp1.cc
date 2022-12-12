void
Sequencer::hitCallback(SequencerRequest* srequest, DataBlock& data,
                       bool llscSuccess,
                       const MachineType mach, const bool externalHit,
                       const Cycles initialRequestTime,
                       const Cycles forwardRequestTime,
                       const Cycles firstResponseTime,
                       const bool was_coalesced)
{
    warn_once("Replacement policy updates recently became the responsibility "
              "of SLICC state machines. Make sure to setMRU() near callbacks "
              "in .sm files!");

    PacketPtr pkt = srequest->pkt;
    Addr request_address(pkt->getAddr());
    RubyRequestType type = srequest->m_type;

    if (was_coalesced) {
        // Notify the controller about a coalesced request so it can properly
        // account for it in its hit/miss stats and/or train prefetchers
        // (this is protocol-dependent)
        m_controller->notifyCoalesced(request_address, type, pkt->req,
                                      data, externalHit);
    }

    // Load-linked handling
    if (type == RubyRequestType_Load_Linked) {
        Addr line_addr = makeLineAddress(request_address);
        llscLoadLinked(line_addr);
    }

    // update the data unless it is a non-data-carrying flush
    if (RubySystem::getWarmupEnabled()) {
        data.setData(pkt);
    } else if (!pkt->isFlush()) {
        if ((type == RubyRequestType_LD) ||
            (type == RubyRequestType_IFETCH) ||
            (type == RubyRequestType_RMW_Read) ||
            (type == RubyRequestType_Locked_RMW_Read) ||
            (type == RubyRequestType_Load_Linked)) {
            pkt->setData(
                data.getData(getOffset(request_address), pkt->getSize()));
            DPRINTF(RubySequencer, "read data %s\n", data);
        } else if (pkt->req->isSwap()) {
            assert(!pkt->isMaskedWrite());
            std::vector<uint8_t> overwrite_val(pkt->getSize());
            pkt->writeData(&overwrite_val[0]);
            pkt->setData(
                data.getData(getOffset(request_address), pkt->getSize()));
            data.setData(&overwrite_val[0],
                         getOffset(request_address), pkt->getSize());
            DPRINTF(RubySequencer, "swap data %s\n", data);
        } else if (pkt->isAtomicOp()) {
            // Set the data in the packet to the old value in the cache
            pkt->setData(
                data.getData(getOffset(request_address), pkt->getSize()));
            DPRINTF(RubySequencer, "AMO original data %s\n", data);
            // execute AMO operation
            (*(pkt->getAtomicOp()))(
                data.getDataMod(getOffset(request_address)));
            DPRINTF(RubySequencer, "AMO new data %s\n", data);
        } else if (type != RubyRequestType_Store_Conditional || llscSuccess) {
            // Types of stores set the actual data here, apart from
            // failed Store Conditional requests
            data.setData(pkt);
            DPRINTF(RubySequencer, "set data %s\n", data);
        }
    }

    // If using the RubyTester, update the RubyTester sender state's
    // subBlock with the recieved data.  The tester will later access
    // this state.
    if (m_usingRubyTester) {
        DPRINTF(RubySequencer, "hitCallback %s 0x%x using RubyTester\n",
                pkt->cmdString(), pkt->getAddr());
        RubyTester::SenderState* testerSenderState =
            pkt->findNextSenderState<RubyTester::SenderState>();
        assert(testerSenderState);
        testerSenderState->subBlock.mergeFrom(data);
    }

    RubySystem *rs = m_ruby_system;
    if (RubySystem::getWarmupEnabled()) {
        assert(pkt->req);
        delete pkt;
        rs->m_cache_recorder->enqueueNextFetchRequest();
    } else if (RubySystem::getCooldownEnabled()) {
        delete pkt;
        rs->m_cache_recorder->enqueueNextFlushRequest();
    } else {
        ruby_hit_callback(pkt);
        testDrainComplete();
    }
}