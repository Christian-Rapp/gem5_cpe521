/**
 * Alexander Knapen, Christian Rapp
 * Implementation similar to Tournament Selection
 */

#include "mem/cache/replacement_policies/TSel_rp.hh"

namespace gem5
{

namespace replacement_policy
{

TSel::TSel(const Params &p)
  : Base(p), replPolicyA(p.replacement_policy_a),
    replPolicyB(p.replacement_policy_b)
{
    // Not sure if here is the place to initialize
    // Need to keep a running saturated counter
    // Need to keep an IndexPolicy for RP 0
    // Need to keep an IndexPolicy for RP 1
    fatal_if((replPolicyA == nullptr) || (replPolicyB == nullptr),
        "All replacement policies must be instantiated");
}

void
TSel::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    // std::shared_ptr<DuelerReplData> casted_replacement_data =
    //     std::static_pointer_cast<DuelerReplData>(replacement_data);
    // replPolicyA->invalidate(casted_replacement_data->replDataA);
    // replPolicyB->invalidate(casted_replacement_data->replDataB);
}

// On touch (MTB Hit) we need to update the auxiliary directories
void
TSel::touch(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    update_aux_dir(pkt->addr);
}

void
TSel::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // std::shared_ptr<DuelerReplData> casted_replacement_data =
    //     std::static_pointer_cast<DuelerReplData>(replacement_data);
    // replPolicyA->reset(casted_replacement_data->replDataA);
    // replPolicyB->reset(casted_replacement_data->replDataB);
}

ReplaceableEntry*
TSel::getVictim(const ReplacementCandidates& candidates, Addr addr) const
{
    ReplaceableEntry* victim;
    // Update the auxiliary directories
    update_aux_dir(addr);

    // Choose the victim based on the current counter
    if (cost > THRESHOLD) {
        victim =  replPolicyA->getVictim(candidates);
    }
    else {
        victim =  replPolicyB->getVictim(candidates);
    }

    return victim;
}

// Instantiate all the replacement data we need
std::shared_ptr<ReplacementData>
TSel::instantiateEntry()
{
    // Don't know what to do here exactly :O

    // DuelerReplData* replacement_data = new DuelerReplData(
    //     replPolicyA->instantiateEntry(), replPolicyB->instantiateEntry());
    // duelingMonitor.initEntry(static_cast<Dueler*>(replacement_data));
    // return std::shared_ptr<DuelerReplData>(replacement_data);
}

// Updates auxiliary directories based on what would have happened
update_aux_dir(addr) {
    int cost = 1;

    // Update Auxiliary for RP 0
    bool hit_0 = false;
    const std::vector<ReplaceableEntry*> candiates_0 =
            indexingPolicy_0->getPossibleEntries(addr);

    hit_0 = address_in_entries(addr, candiates_0);
    // Miss for RP 0
    if (!hit_0) {
        ReplaceableEntry* victim_0;
        replPolicyA->getVictim(candiates_0);

        // Need to replace the victim with the new addr
    }

    // Update Auxiliary for RP 1
    bool hit_1 = false;
    const std::vector<ReplaceableEntry*> candiates_1 =
            indexingPolicy_1->getPossibleEntries(addr);

    hit_1 = address_in_entries(addr, candiates_1);
    // Miss for RP 0
    if (!hit_1) {
        ReplaceableEntry* victim_1;
        replPolicyA->getVictim(candiates_1);

        // Need to replace the victim with the new addr
    }

    // Update the "TSEL Counter"
    if (hit_0 && !hit_1) {
        count += -1*cost;
    }

    if (!hit_0 && hit_1) {
        count += cost;
    }
}

// Determine if address is within list of entries
bool address_in_entries(Addr addr, const ReplacementCandidates& entries) {
    // Extract block tag
    Addr tag = extractTag(addr);

    // IDk what is_secure is
    bool is_secure = true;

    // Search for block
    for (const auto& location : entries) {
        CacheBlk* blk = static_cast<CacheBlk*>(location);
        if (blk->matchTag(tag, is_secure)) {
            return true;
        }
    }

    // Did not find block
    return false;
}

} // namespace replacement_policy
} // namespace gem5
