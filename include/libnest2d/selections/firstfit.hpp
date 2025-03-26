#ifndef FIRSTFIT_HPP
#define FIRSTFIT_HPP

#include "selection_boilerplate.hpp"

namespace libnest2d { namespace selections {

template<class RawShape>
class _FirstFitSelection: public SelectionBoilerplate<RawShape> {
    using Base = SelectionBoilerplate<RawShape>;

public:
    /**
     * @brief The Config for FirstFit heuristic.
     */
    struct Config {

        /*
        * If true, checks all items to see if they can be packed into a texture. Can take a long time when there are a lot of items.
        */
        bool verify_items = true;
           
        /*
        * If this option is enabled, then before starting to calculate position in current bucket, 
        * it also starts calculating position on next texture in asynchronous mode. 
        * This helps when you have a lot of items and you are guaranteed to get several buckets.
        * In this way, if current bucket is too small, we can save time and get result from next bucket, 
        * after which a new asynchronous operation is launched again, and so on until desired bucket is received or a new bucket is created.
        */
        bool texture_parallel = false;

        /*
        * Created for the same purpose as "texture_parallel", but in this case asynchronous calculation operations called for each existing bucket. 
        * Be careful with this, it can consume a lot of resources, but at the same time it can be the fastest solution.
        */
        bool texture_parallel_hard = false;
    };

public:
    using typename Base::Item;

private:
    using Base::packed_bins_;
    using typename Base::ItemGroup;
    using Container = ItemGroup;//typename std::vector<_Item<RawShape>>;

    Container store_;
    Config m_config;

public:

	inline void configure(const Config& config) {
		m_config = config;
	}

    template<class TPlacer, class TIterator,
             class TBin = typename PlacementStrategyLike<TPlacer>::BinType,
             class PConfig = typename PlacementStrategyLike<TPlacer>::Config>
    void packItems(TIterator first,
                   TIterator last,
                   TBin&& bin,
                   PConfig&& pconfig = PConfig())
    {

        using Placer = PlacementStrategyLike<TPlacer>;

        store_.clear();
        store_.reserve(last-first);

        std::vector<Placer> placers;
        placers.reserve(last-first);
        
        std::for_each(first, last, [this](Item& itm) {
            if(itm.isFixed()) {
                if (itm.binId() < 0) itm.binId(0);
                auto binidx = size_t(itm.binId());
                
                while(packed_bins_.size() <= binidx)
                    packed_bins_.emplace_back();
                
                packed_bins_[binidx].emplace_back(itm);
            } else {
                store_.emplace_back(itm);
            }
        });

        // If the packed_items array is not empty we have to create as many
        // placers as there are elements in packed bins and preload each item
        // into the appropriate placer
        for(ItemGroup& ig : packed_bins_) {
            placers.emplace_back(bin);
            placers.back().configure(pconfig);
            placers.back().preload(ig);
        }
        
        auto sortfunc = [](Item& i1, Item& i2) {
            int p1 = i1.priority(), p2 = i2.priority();
            return p1 == p2 ? i1.area() > i2.area() : p1 > p2;
        };

        std::sort(std::execution::par, store_.begin(), store_.end(), sortfunc);

        auto total = last-first;
        auto makeProgress = [this, &total](Placer& placer, size_t bin_idx) {
            packed_bins_[bin_idx] = placer.getItems();
            this->last_packed_bin_id_ = int(bin_idx);
            this->progress_(static_cast<unsigned>(--total));
        };

        auto& cancelled = this->stopcond_;
        
        if (m_config.verify_items)
        {
            this->template remove_unpackable_items<Placer>(store_, bin, pconfig);
        }

        auto it = store_.begin();

        while(it != store_.end() && !cancelled()) {
            bool was_packed = false;
            size_t j = 0;
			auto& item = *it;
			auto remains = rem(it, store_);

            Placer::PackResult candidate;
            while(!was_packed && !cancelled()) {
                auto do_accept = [&makeProgress, &placers, &candidate, &item, &was_packed](Placer& placer, size_t index)
                    {
                        was_packed = candidate;
                        if (!was_packed) return;

                        item.get().binId(index);
                        placer.accept(candidate);
						makeProgress(placers[index], index);
                    };

				auto do_pack = [&item, &remains](Placer& placer) -> Placer::PackResult // TODO: add some kind of stop condition to stop async operations when was_packed already is true
					{
						return placer.trypack(item, remains);
					};

                if (m_config.texture_parallel)
                {
                    std::future<Placer::PackResult> next_texture_result;
					for (; j < placers.size() && !was_packed && !cancelled(); j++) {
						Placer& current_placer = placers[j];

						// if not first or last placer
						if (j != placers.size() - 1)
						{
							next_texture_result = std::async(std::launch::deferred | std::launch::async, do_pack, placers[j + 1]);
						}

						// if placer is first then use sync packaging
						if (j == 0)
						{
							candidate = do_pack(current_placer);
						}
						else
						{
							next_texture_result.wait();
							candidate = next_texture_result.get();
						}

						do_accept(current_placer, j);
					}
                }
                else if (m_config.texture_parallel_hard)
                {
					std::vector<std::future<Placer::PackResult>> placer_results(placers.size());

					for (size_t i = 0; placer_results.size() > i; i++)
					{
                        if (placers[i].canPack(item))
                        {
                            placer_results[i] = std::async(std::launch::deferred | std::launch::async, do_pack, placers[i]);
                        }
					}

					for (size_t i = 0; placer_results.size() > i; i++)
					{
						auto& result = placer_results[i];
                        if (!result.valid()) continue;

						result.wait();
						candidate = result.get();
						do_accept(placers[i], i);
						if (was_packed) break;
					}
                }
                else
                {
					for (; j < placers.size() && !was_packed && !cancelled(); j++) { // Original way to process items
                        
                        candidate = do_pack(placers[j]);
                        do_accept(placers[j], j);
					}
                }

				if (!was_packed) {
					Placer& new_placer = placers.emplace_back(bin);
					placers.back().configure(pconfig);
					packed_bins_.emplace_back();
					candidate = do_pack(new_placer);

					do_accept(new_placer, placers.size() - 1);
				}
            }
            ++it;
        }
    }
};

}
}

#endif // FIRSTFIT_HPP
