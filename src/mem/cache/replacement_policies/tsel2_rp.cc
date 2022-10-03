/**
 * Copyright (c) 2019, 2020 Inria
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

#include "mem/cache/replacement_policies/tsel2_rp.hh"

#include "base/logging.hh"
#include "params/TSel2RP.hh"

namespace gem5
{

namespace replacement_policy
{

TSel2::TSel2(const Params &p)
  : Base(p),
    replPolicyA(p.replacement_policy_a),
    replPolicyB(p.replacement_policy_b),
    indexPolicyA(p.index_policy_a),
    indexPolicyB(p.index_policy_b),
    // atdA(p.atd_a),
    // atdB(p.atd_b),
    numCounterBits(p.num_counter_bits)
{
    // std::dynamic_pointer_cast<BaseSetAss>(indexPolicyA)->tagsInit();
    // indexPolicyB->tagsInit();
    fatal_if((replPolicyA == nullptr) || (replPolicyB == nullptr),
        "All replacement policies must be instantiated");

    fatal_if(p.num_counter_bits > 16,
            "Maximum number of counter bits must be <= 16");


    // Construct the list of counters
    for (uint32_t i = 0; i < indexPolicyA->getNumSets(); i++)
    {
        SCTRs.push_back(SatCounter16(p.num_counter_bits));
    }

    std::vector<std::vector<ReplaceableEntry*>> setA = indexPolicyA->getSets();
    std::vector<std::vector<ReplaceableEntry*>> setB = indexPolicyB->getSets();

    uint32_t num_blks = indexPolicyA->getNumSets() * indexPolicyB->getAssoc();
    for (uint32_t blk_index = 0; blk_index < num_blks; blk_index++) {
            CacheBlk * blk = new CacheBlk();
            // blk->invalidate();
            indexPolicyA->setEntry(blk, blk_index);
            blk->replacementData = replPolicyA->instantiateEntry();
            assert(blk->replacementData != nullptr);
    }

    for (uint32_t blk_index = 0; blk_index < num_blks; blk_index++) {
            CacheBlk * blk = new CacheBlk();
            // blk->invalidate();
            indexPolicyB->setEntry(blk, blk_index);
            blk->replacementData = replPolicyB->instantiateEntry();
            assert(blk->replacementData != nullptr);
    }
}

void
TSel2::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    std::shared_ptr<TSel2ReplData> casted_replacement_data =
        std::static_pointer_cast<TSel2ReplData>(replacement_data);
    replPolicyA->invalidate(casted_replacement_data->replDataA);
    replPolicyB->invalidate(casted_replacement_data->replDataB);
}

void
TSel2::touch(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    std::shared_ptr<TSel2ReplData> casted_replacement_data =
        std::static_pointer_cast<TSel2ReplData>(replacement_data);
    replPolicyA->touch(casted_replacement_data->replDataA, pkt);
    replPolicyB->touch(casted_replacement_data->replDataB, pkt);

    // Find the blk in the MTD that was accessed based on the packet address
    BaseTags *MTD = cache->getTags();
    CacheBlk *blk = MTD->findBlock(pkt->getAddr(), pkt->isSecure());

    // Retrieve the cost for the accessed block
    uint8_t costq = blk->costq;

    updateAuxiliaryDirectories(pkt->getAddr(), costq);
}

void
TSel2::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    std::shared_ptr<TSel2ReplData> casted_replacement_data =
        std::static_pointer_cast<TSel2ReplData>(replacement_data);
    replPolicyA->touch(casted_replacement_data->replDataA);
    replPolicyB->touch(casted_replacement_data->replDataB);

    // replPolicyA->touch()
}

void
TSel2::reset(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    std::shared_ptr<TSel2ReplData> casted_replacement_data =
        std::static_pointer_cast<TSel2ReplData>(replacement_data);

    std::shared_ptr<ReplacementData> replDataA
        = casted_replacement_data->replDataA;
    std::shared_ptr<ReplacementData> replDataB
        = casted_replacement_data->replDataB;

    replPolicyA->reset(replDataA);
    replPolicyB->reset(replDataB);
}

void
TSel2::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    std::shared_ptr<TSel2ReplData> casted_replacement_data =
        std::static_pointer_cast<TSel2ReplData>(replacement_data);
    replPolicyA->reset(casted_replacement_data->replDataA);
    replPolicyB->reset(casted_replacement_data->replDataB);
}

ReplaceableEntry*
TSel2::getVictim(const ReplacementCandidates& candidates) const
{
    ReplaceableEntry* victim;

    // Choose the victim based on the current counter
    // SatCounter16 counter = getCounter(addr);
    double threshold = 1.0;
    // We use the counter's current saturation to calculate who
    // is winning. A threshold of 0.5 is equivalent to checking
    // the MSB of the counter.
    // (MSB = 0 -> choose repl A, MSB = 1 -> choose repl B)
    if (threshold <= 0.5) { // replacement policy A is winning
        victim =  replPolicyA->getVictim(candidates);
    }
    else {
        victim =  replPolicyB->getVictim(candidates);
    }

    return victim;
}

ReplaceableEntry*
TSel2::getVictim(const ReplacementCandidates& candidates, Addr addr)
{
    ReplaceableEntry* victim;

    // Choose the victim based on the current counter
    SatCounter16 counter = getCounter(addr);
    double threshold = counter.calcSaturation();
    // We use the counter's current saturation to calculate who
    // is winning. A threshold of 0.5 is equivalent to checking
    // the MSB of the counter.
    // (MSB = 0 -> choose repl A, MSB = 1 -> choose repl B)
    if (threshold <= 0.5) { // replacement policy A is winning
        victim =  replPolicyA->getVictim(candidates);
    }
    else {
        victim =  replPolicyB->getVictim(candidates);
    }

    // Extract the miss cost from the victim in the MTD
    uint8_t costq = static_cast<CacheBlk *>(victim)->costq;

    // Update the auxiliary directories with the cost if necessary
    updateAuxiliaryDirectories(addr, costq);

    return victim;
}

std::shared_ptr<ReplacementData>
TSel2::instantiateEntry()
{
    std::shared_ptr<ReplacementData> replDataA =
        replPolicyA->instantiateEntry();
    assert(replDataA != nullptr);
    std::shared_ptr<ReplacementData> replDataB =
        replPolicyB->instantiateEntry();
    assert(replDataB != nullptr);
    TSel2ReplData* replacement_data = new TSel2ReplData(
        replDataA, replDataB);
    return std::shared_ptr<ReplacementData>(replacement_data);
}

// Non gem5 replacement policy functions

// Updates auxiliary directories based on what would have happened
void
TSel2::updateAuxiliaryDirectories(Addr addr, uint8_t costq) {

    // === update ATD for replacement policy A ===
    bool hitA = false;
    const std::vector<ReplaceableEntry*> candidatesA =
            indexPolicyA->getPossibleEntries(addr);

    // Update the costq for the auxiliary a
    // updateBlockReplacementData(mtdCandidates, candidatesA);
    // is the address a hit in ATD A?
    hitA = isAddressInEntries(addr, candidatesA);

    // Miss in ATD A
    if (!hitA)
    {
        CacheBlk *victim;
        // if (candidatesA[0] != nullptr) {
            // Find a victim to replace
        victim = static_cast<CacheBlk*>(
        replPolicyA->getVictim(candidatesA));

        // if (victim->valid) {
        assert(victim->replacementData.get() != nullptr);
        replPolicyA->reset(victim->replacementData);
        // }

        // }
        // victim->validate();
        // Need to replace the victim with the new tag
        // Should is_secure be true here?? I think it's fine
        // since we don't care about any coherence bits in the
        // ATDs, just the tags
        victim->invalidate();
        victim->insert(indexPolicyA->extractTag(addr), true);
    }

    // === update ATD for replacement policy B ===
    bool hitB = false;
    const std::vector<ReplaceableEntry*> candidatesB =
            indexPolicyB->getPossibleEntries(addr);

    // Update the costq for the auxiliary b
    // updateBlockReplacementData(mtdCandidates, candidatesB);

    // is the address a hit in ATD B?
    hitB = isAddressInEntries(addr, candidatesB);

    // Miss in ATD B
    if (!hitB)
    {
        CacheBlk *victim;
        //  if (candidatesB[0] != nullptr) {
        // Find a victim to replace
        victim = static_cast<CacheBlk*>(
            replPolicyB->getVictim(candidatesB));

        // if (victim->valid) {
        assert(victim->replacementData.get() != nullptr);
        replPolicyB->reset(victim->replacementData);
        //  }
        // }
        // victim->validate();
        // Need to replace the victim with the new tag
        // Should is_secure be true here?? I think it's fine
        // since we don't care about any coherence bits in the
        // ATDs, just the tags
        victim->invalidate();
        victim->insert(indexPolicyB->extractTag(addr), true);
    }

    // Update the saturating counter for this set
    SatCounter16 counter = getCounter(addr);

    // Hit in ATD A and miss in ATD B
    if (hitA && !hitB) {
        // ATD A is performing better, bias counter towards 0
        counter.operator-=(costq);
    }
    // Miss in ATD A, hit in ATD B
    else if (!hitA && hitB) {
        // ATD B is performing better, bias counter toward max value
        counter.operator+=(costq);
    }
}

SatCounter16
TSel2::getCounter(Addr addr)
{
    // Extract the set from the address
    // int setShift = indexPolicyA->getSetShift();
    // int setMask = indexPolicyA->getSetMask();
    // int set = (addr >> setShift) & setMask;
    SatCounter16 *counter = new SatCounter16(3);
    return *counter;
    // return SCTRs[set];
}

// Determine if address is within list of entries
bool TSel2::isAddressInEntries(Addr addr, const ReplacementCandidates& entries)
{
    // Extract tag from address
    Addr tag = indexPolicyA->extractTag(addr);

    // Search for block
    for (const auto& location : entries) {

        // Extract block from entries
        CacheBlk* blk = static_cast<CacheBlk*>(location);
        if (blk != nullptr)
        {
            bool is_secure = blk->isSecure();

            if (blk->matchTag(tag, is_secure)) {
                return true;
            }
        }
    }

    // Did not find block
    return false;
}

// void
// TSel2::tagsInit()
// {
//     // Initialize all blocks
//     int numBlocks = index_policy_a->size / index_policy_a->entry_size;
//     for (unsigned blk_index = 0; blk_index < numBlocks; blk_index++) {

//         // Link block to indexing policy
//         indexingPolicy->setEntry(blk, blk_index);

//         // Associate a data chunk to the block
//         blk->data = &dataBlks[blkSize*blk_index];

//         // Associate a replacement data entry to the block
//         blk->replacementData = replacementPolicy->instantiateEntry();
//     }
// }



} // namespace replacement_policy
} // namespace gem5
