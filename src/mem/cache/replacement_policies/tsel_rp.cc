/**
 * Alexander Knapen, Christian Rapp, Joel Valdovinos
 * Implementation similar to Tournament Selection
 */
#include "mem/cache/replacement_policies/tsel_rp.hh"

#include "base/intmath.hh"
#include "mem/cache/cache_blk.hh"

namespace gem5
{

namespace replacement_policy
{

/** TSel class constructor */
TSel::TSel(const Params &p)
  : Base(p), replPolicyA(p.replacement_policy_a),
    indexPolicyA(p.index_policy_a),
    replPolicyB(p.replacement_policy_b),
    indexPolicyB(p.index_policy_b),
    numSets(p.index_policy_a->size /
            (p.index_policy_a->entry_size*p.index_policy_a->assoc)),
    numWays(p.index_policy_a->assoc),
    entrySize(p.index_policy_a->entry_size)
{
    fatal_if((replPolicyA == nullptr) || (replPolicyB == nullptr),
        "All replacement policies must be instantiated");

    fatal_if(p.num_counter_bits > 16,
            "Maximum number of counter bits must be <= 16");

    // Construct the list of counters
    for (int i = 0; i < numSets; i++)
    {
        SCTRs.push_back(SatCounter16(p.num_counter_bits));
    }


}

void
TSel::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    std::shared_ptr<TSelReplData> casted_replacement_data =
        std::static_pointer_cast<TSelReplData>(replacement_data);
    replPolicyA->invalidate(casted_replacement_data->replDataA);
    replPolicyB->invalidate(casted_replacement_data->replDataB);
}

// On touch (MTD Hit) we need to update the auxiliary directories
void
TSel::touch(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    uint8_t costq = 1; // @todo: retrieve actual costq from MTD!!
                       // PROBLEM: the costq is in the MTD -> outside this
                       //          replacement policy!! How do we get it?
    updateAuxiliaryDirectories(pkt->getAddr(), costq);
}

void
TSel::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    std::shared_ptr<TSelReplData> casted_replacement_data =
        std::static_pointer_cast<TSelReplData>(replacement_data);
    replPolicyA->reset(casted_replacement_data->replDataA);
    replPolicyB->reset(casted_replacement_data->replDataB);
}

ReplaceableEntry*
TSel::getVictim(const ReplacementCandidates& candidates, Addr addr)
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

    // Update the auxiliary directories
    uint8_t costq = 1; // @todo: Extract actual costq from MTD!!!
    updateAuxiliaryDirectories(addr, costq);
    // @todo: ^ pass the victim into this function? we shouldn't be
    //   recalculating it since it needs to be consistent
    return victim;
}

// Instantiate all the replacement data we need
std::shared_ptr<ReplacementData>
TSel::instantiateEntry()
{
    // Since TSel doesn't have its own replacement data,
    // we are only responsible for instantiating the
    // replacement data for the two sub-replacement policies
    TSelReplData* replacement_data = new TSelReplData(
        replPolicyA->instantiateEntry(), replPolicyB->instantiateEntry());

    return std::shared_ptr<TSelReplData>(replacement_data);
}

// Updates auxiliary directories based on what would have happened
void
TSel::updateAuxiliaryDirectories(Addr addr, uint8_t costq) {

    // === update ATD for replacement policy A ===
    bool hitA = false;
    const std::vector<ReplaceableEntry*> candidatesA =
            indexPolicyA->getPossibleEntries(addr);

    // is the address a hit in ATD A?
    hitA = isAddressInEntries(addr, candidatesA);

    // Miss in ATD A
    if (!hitA)
    {
        // Find a victim to replace
        CacheBlk *victim = static_cast<CacheBlk*>(
            replPolicyA->getVictim(candidatesA));

        // Need to replace the victim with the new tag
        // Should is_secure be true here?? I think it's fine
        // since we don't care about any coherence bits in the
        // ATDs, just the tags
        victim->insert(indexPolicyA->extractTag(addr), true);
    }

    // === update ATD for replacement policy B ===
    bool hitB = false;
    const std::vector<ReplaceableEntry*> candidatesB =
            indexPolicyB->getPossibleEntries(addr);

    // is the address a hit in ATD B?
    hitB = isAddressInEntries(addr, candidatesB);

    // Miss in ATD B
    if (!hitB)
    {
        // Find a victim to replace
        CacheBlk *victim = static_cast<CacheBlk*>(
            replPolicyB->getVictim(candidatesB));

        // Need to replace the victim with the new tag
        // Should is_secure be true here??
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
TSel::getCounter(Addr addr)
{
    // Extract the set from the address
    int setShift = floorLog2(entrySize);
    int setMask = numSets - 1;
    int set = (addr >> setShift) & setMask;

    return SCTRs[set];
}

// Determine if address is within list of entries
bool TSel::isAddressInEntries(Addr addr, const ReplacementCandidates& entries)
{
    // Extract tag from address
    Addr tag = indexPolicyA->extractTag(addr);

    // Search for block
    for (const auto& location : entries) {

        // Extract block from entries
        CacheBlk* blk = static_cast<CacheBlk*>(location);
        bool is_secure = blk->isSecure();

        if (blk->matchTag(tag, is_secure)) {
            return true;
        }
    }

    // Did not find block
    return false;
}

} // namespace replacement_policy
} // namespace gem5