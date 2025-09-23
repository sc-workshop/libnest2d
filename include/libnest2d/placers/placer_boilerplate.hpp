#ifndef PLACER_BOILERPLATE_HPP
#define PLACER_BOILERPLATE_HPP

#include <libnest2d/nester.hpp>

namespace libnest2d { namespace placers {

struct EmptyConfig {};

template<class Subclass, class RawShape, class TBin, class Cfg = EmptyConfig>
class PlacerBoilerplate {
    mutable double farea_ = 0.0;
    mutable double binarea_ = 0.0;
public:
    using ShapeType = RawShape;
    using Item = _Item<RawShape>;
    using Vertex = TPoint<RawShape>;
    using Segment = _Segment<Vertex>;
    using BinType = TBin;
    using Coord = TCoord<Vertex>;
    using Config = Cfg;
    using ItemGroup = _ItemGroup<RawShape>;
    using DefaultIter = typename ItemGroup::const_iterator;

    class PackResult {
    private:
        Item *item_ptr_;
        Vertex move_;
        Radians rot_;
        double overfit_ = 0.0;
        friend class PlacerBoilerplate;
        friend Subclass;

    public:
        PackResult(Item& item):
            item_ptr_(&item),
            move_(item.translation()),
            rot_(item.rotation()) {}

        PackResult(double overfit = 1.0):
            item_ptr_(nullptr), overfit_(overfit) {}

    public:
        operator bool() { return item_ptr_ != nullptr; }
        double overfit() const { return overfit_; }
    };

    inline PlacerBoilerplate(const BinType& bin, unsigned cap = 50): m_bin(bin)
    {
        m_items.reserve(cap);
        binarea_ = sl::area(bin);
    }

    inline const BinType& bin() const BP2D_NOEXCEPT { return m_bin; }

    template<class TB> inline void bin(TB&& b) {
        m_bin = std::forward<BinType>(b);
        binarea_ = sl::area(m_bin);
    }

    inline void configure(const Config& config) BP2D_NOEXCEPT {
        m_config = config;
    }

    template<class Range = ConstItemRange<DefaultIter>>
    bool pack(Item& item, const Range& rem = Range()) {
        if (item.area() >= freeArea()) return false;

        PackResult&& r = static_cast<Subclass*>(this)->trypack(item, rem);
        if(r) {
            m_items.emplace_back(*(r.item_ptr_));
            farea_ += item.area();
        }
        return r;
    }

	template<class Range = ConstItemRange<DefaultIter>>
	PackResult trypack(Item& item, const Range& rem = Range()) {
		return static_cast<Subclass*>(this)->trypack(item, rem);
	}

    void preload(const ItemGroup& packeditems) {
        m_items.insert(m_items.end(), packeditems.begin(), packeditems.end());
    }

    void accept(PackResult& r) {
        if(r) {
            r.item_ptr_->translation(r.move_);
            r.item_ptr_->rotation(r.rot_);
            m_items.emplace_back(*(r.item_ptr_));
            farea_ += (*(r.item_ptr_)).area();

            static_cast<Subclass*>(this)->acceptResult(r);
        }
    }

    bool canPack(Item& item) const
    {
        if (item.area() < freeArea()) return true;

        return false;
    }

    void unpackLast() {
        m_items.pop_back();
    }

    inline const ItemGroup& getItems() const { return m_items; }

    inline void clearItems() {
        m_items.clear();
    }

    inline double filledArea() const {
        return farea_;
    }

    inline double freeArea() const
    {
        return binarea_ - farea_;
    }

protected:

    BinType m_bin;
    ItemGroup m_items;
    Cfg m_config;
};


#define DECLARE_PLACER(Base) \
using Base::m_bin;                 \
using Base::m_items;               \
using Base::m_config;              \
public:                           \
using typename Base::ShapeType;   \
using typename Base::Item;        \
using typename Base::ItemGroup;   \
using typename Base::BinType;     \
using typename Base::Config;      \
using typename Base::Vertex;      \
using typename Base::Segment;     \
using typename Base::PackResult;  \
using typename Base::Coord;       \
private:

}
}

#endif // PLACER_BOILERPLATE_HPP
