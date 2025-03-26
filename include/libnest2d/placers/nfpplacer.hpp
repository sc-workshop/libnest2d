#ifndef NOFITPOLY_HPP
#define NOFITPOLY_HPP

#include <cassert>

// For parallel for
#include <functional>
#include <iterator>
#include <future>
#include <atomic>

#ifndef NDEBUG
#include <iostream>
#endif
#include <libnest2d/geometry_traits_nfp.hpp>
#include <libnest2d/optimizer.hpp>

#include "placer_boilerplate.hpp"

#include <libnest2d/parallel.hpp>

namespace libnest2d {
    namespace placers {

        template<class RawShape>
        struct NfpPConfig {

            using ItemGroup = _ItemGroup<RawShape>;
            using ObjectCallback = std::function<void(const nfp::Shapes<RawShape>&, // merged pile
                const ItemGroup&,                                                   // packed items
                const ItemGroup&                                                    // remaining items
            )>;

            using RotationCallback = std::function<std::vector<Radians>(const _Item<RawShape>&)>;

            enum class Alignment {
                CENTER,
                BOTTOM_LEFT,
                BOTTOM_RIGHT,
                TOP_LEFT,
                TOP_RIGHT,
                DONT_ALIGN      //!> Warning: parts may end up outside the bin with the
                                //! default object function.
            };

            /// Which angles to try out for better results.
            std::vector<Radians> rotations;

            /// Where to align the resulting packed pile.
            Alignment alignment;

            /// Where to start putting objects in the bin.
            Alignment starting_point;

            /**
             * @brief A function object representing the fitting function in the
             * placement optimization process. (Optional)
             *
             * This is the most versatile tool to configure the placer. The fitting
             * function is evaluated many times when a new item is being placed into the
             * bin. The output should be a rated score of the new item's position.
             *
             * This is not a mandatory option as there is a default fitting function
             * that will optimize for the best pack efficiency. With a custom fitting
             * function you can e.g. influence the shape of the arranged pile.
             *
             * \param item The only parameter is the candidate item which has info
             * about its current position. Your job is to rate this position compared to
             * the already packed items.
             *
             */
            std::function<double(const _Item<RawShape>&)> object_function;

            /**
             * @brief The quality of search for an optimal placement.
             * This is a compromise slider between quality and speed. Zero is the
             * fast and poor solution while 1.0 is the slowest but most accurate.
             */
            float accuracy = 0.65f;

            /**
             * @brief If you want to see items inside other item's holes, you have to
             * turn this switch on.
             *
             * This will only work if a suitable nfp implementation is provided.
             * The library has no such implementation right now.
             */
            bool explore_holes = false;

            /**
             * @brief If true, use all CPUs available. Run on a single core otherwise.
             */
            bool parallel = true;

            /**
             * @brief before_packing Callback that is called just before a search for
             * a new item's position is started. You can use this to create various
             * cache structures and update them between subsequent packings.
             *
             * \param merged pile A polygon that is the union of all items in the bin.
             *
             * \param pile The items parameter is a container with all the placed
             * polygons excluding the current candidate. You can for instance check the
             * alignment with the candidate item or do anything else.
             *
             * \param remaining A container with the remaining items waiting to be
             * placed. You can use some features about the remaining items to alter to
             * score of the current placement. If you know that you have to leave place
             * for other items as well, that might influence your decision about where
             * the current candidate should be placed. E.g. imagine three big circles
             * which you want to place into a box: you might place them in a triangle
             * shape which has the maximum pack density. But if there is a 4th big
             * circle than you won't be able to pack it. If you knew apriori that
             * there four circles are to be placed, you would have placed the first 3
             * into an L shape. This parameter can be used to make these kind of
             * decisions (for you or a more intelligent AI).
             */
            ObjectCallback before_packing;
            ObjectCallback after_packing;

            RotationCallback rotation_callback;

            std::function<void(const ItemGroup&, NfpPConfig& config)> on_preload;

            NfpPConfig() : rotations({ 0.0, Pi / 2.0, Pi, 3 * Pi / 2 }),
                alignment(Alignment::CENTER), starting_point(Alignment::CENTER) {
            }
        };

        /**
         * A class for getting a point on the circumference of the polygon (in log time)
         *
         * This is a transformation of the provided polygon to be able to pinpoint
         * locations on the circumference. The optimizer will pass a floating point
         * value e.g. within <0,1> and we have to transform this value quickly into a
         * coordinate on the circumference. By definition 0 should yield the first
         * vertex and 1.0 would be the last (which should coincide with first).
         *
         * We also have to make this work for the holes of the captured polygon.
         */
        template<class RawShape> 
        class EdgeCache {
            using Vertex = TPoint<RawShape>;
            using Coord = TCoord<Vertex>;
            using Edge = _Segment<Vertex>;

            struct ContourCache {
                mutable std::vector<double> corners;
                std::vector<Edge> emap;
                std::vector<double> distances;
                double full_distance = 0;
            } contour_;

            std::vector<ContourCache> holes_;

            double accuracy_ = 1.0;

            static double length(const Edge& e)
            {
                return std::sqrt(e.template sqlength<double>());
            }

            void createCache(const RawShape& sh) {
                {   // For the contour
                    auto first = shapelike::cbegin(sh);
                    auto next = std::next(first);
                    auto endit = shapelike::cend(sh);

                    contour_.distances.reserve(shapelike::contourVertexCount(sh));

                    while (next != endit) {
                        contour_.emap.emplace_back(*(first++), *(next++));
                        contour_.full_distance += length(contour_.emap.back());
                        contour_.distances.emplace_back(contour_.full_distance);
                    }
                }

                for (auto& h : shapelike::holes(sh)) { // For the holes
                    auto first = h.begin();
                    auto next = std::next(first);
                    auto endit = h.end();

                    ContourCache hc;
                    hc.distances.reserve(endit - first);

                    while (next != endit) {
                        hc.emap.emplace_back(*(first++), *(next++));
                        hc.full_distance += length(hc.emap.back());
                        hc.distances.emplace_back(hc.full_distance);
                    }

                    holes_.emplace_back(std::move(hc));
                }
            }

            size_t stride(const size_t N) const {
                using std::round;
                using std::pow;

                return static_cast<Coord>(
                    round(N / pow(N, pow(accuracy_, 1.0 / 3.0)))
                    );
            }

            void fetchCorners() const {
                if (!contour_.corners.empty()) return;

                const auto N = contour_.distances.size();
                const auto S = stride(N);

                contour_.corners.reserve(N / S + 1);
                contour_.corners.emplace_back(0.0);
                auto N_1 = N - 1;
                contour_.corners.emplace_back(0.0);
                for (size_t i = 0; i < N_1; i += S) {
                    contour_.corners.emplace_back(
                        contour_.distances.at(i) / contour_.full_distance);
                }
            }

            void fetchHoleCorners(unsigned hidx) const {
                auto& hc = holes_[hidx];
                if (!hc.corners.empty()) return;

                const auto N = hc.distances.size();
                auto N_1 = N - 1;
                const auto S = stride(N);
                hc.corners.reserve(N / S + 1);
                hc.corners.emplace_back(0.0);
                for (size_t i = 0; i < N_1; i += S) {
                    hc.corners.emplace_back(
                        hc.distances.at(i) / hc.full_distance);
                }
            }

            inline Vertex coords(const ContourCache& cache, double distance) const {
                assert(distance >= .0 && distance <= 1.0);
                if (cache.distances.empty() || cache.emap.empty()) return Vertex{};
                if (distance > 1.0) distance = std::fmod(distance, 1.0);

                // distance is from 0.0 to 1.0, we scale it up to the full length of
                // the circumference
                double d = distance * cache.full_distance;

                auto& distances = cache.distances;

                // Magic: we find the right edge in log time
                auto it = std::lower_bound(distances.begin(), distances.end(), d);
                auto idx = it - distances.begin();      // get the index of the edge
                auto edge = cache.emap[idx];         // extrac the edge

                // Get the remaining distance on the target edge
                auto ed = d - (idx > 0 ? *std::prev(it) : 0);
                auto angle = edge.angleToXaxis();
                Vertex ret = edge.first();

                // Get the point on the edge which lies in ed distance from the start
                ret += { static_cast<Coord>(std::round(ed* std::cos(angle))),
                    static_cast<Coord>(std::round(ed* std::sin(angle))) };

                return ret;
            }

        public:

            using iterator = std::vector<double>::iterator;
            using const_iterator = std::vector<double>::const_iterator;

            inline EdgeCache() = default;

            inline EdgeCache(const _Item<RawShape>& item)
            {
                createCache(item.transformedShape());
            }

            inline EdgeCache(const RawShape& sh)
            {
                createCache(sh);
            }

            /// Resolution of returned corners. The stride is derived from this value.
            void accuracy(double a /* within <0.0, 1.0>*/) { accuracy_ = a; }

            /**
             * @brief Get a point on the circumference of a polygon.
             * @param distance A relative distance from the starting point to the end.
             * Can be from 0.0 to 1.0 where 0.0 is the starting point and 1.0 is the
             * closing point (which should be eqvivalent with the starting point with
             * closed polygons).
             * @return Returns the coordinates of the point lying on the polygon
             * circumference.
             */
            inline Vertex coords(double distance) const {
                return coords(contour_, distance);
            }

            inline Vertex coords(unsigned hidx, double distance) const {
                assert(hidx < holes_.size());
                return coords(holes_[hidx], distance);
            }

            inline double circumference() const BP2D_NOEXCEPT {
                return contour_.full_distance;
            }

            inline double circumference(unsigned hidx) const BP2D_NOEXCEPT {
                return holes_[hidx].full_distance;
            }

            /// Get the normalized distance values for each vertex
            inline const std::vector<double>& corners() const BP2D_NOEXCEPT {
                fetchCorners();
                return contour_.corners;
            }

            /// corners for a specific hole
            inline const std::vector<double>&
                corners(unsigned holeidx) const BP2D_NOEXCEPT {
                fetchHoleCorners(holeidx);
                return holes_[holeidx].corners;
            }

            /// The number of holes in the abstracted polygon
            inline size_t holeCount() const BP2D_NOEXCEPT { return holes_.size(); }

        };

        template<nfp::NfpLevel lvl>
        struct Lvl { static const nfp::NfpLevel value = lvl; };

        template<class RawShape>
        inline void correctNfpTransform(
            nfp::NfpResult<RawShape>& nfp,
            const _Item<RawShape>& stationary,
            const _Item<RawShape>& orbiter)
        {
            // The provided nfp is somewhere in the dark. We need to get it
            // to the right position around the stationary shape.
            // This is done by choosing the leftmost lowest vertex of the
            // orbiting polygon to be touched with the rightmost upper
            // vertex of the stationary polygon. In this configuration, the
            // reference vertex of the orbiting polygon (which can be dragged around
            // the nfp) will be its rightmost upper vertex that coincides with the
            // rightmost upper vertex of the nfp. No proof provided other than Jonas
            // Lindmark's reasoning about the reference vertex of nfp in his thesis
            // ("No fit polygon problem" - section 2.1.9)

            auto touch_sh = stationary.rightmostTopVertex();
            auto touch_other = orbiter.leftmostBottomVertex();
            auto dtouch = touch_sh - touch_other;
            auto top_other = orbiter.rightmostTopVertex() + dtouch;
            auto dnfp = top_other - nfp.second; // nfp.second is the nfp reference point
            shapelike::translate(nfp.first, dnfp);
        }

        template<class RawShape>
        inline void correctNfpTransform(
            nfp::NfpResult<RawShape>& nfp,
            const RawShape& stationary,
            const _Item<RawShape>& orbiter)
        {
            auto touch_sh = nfp::rightmostUpVertex(stationary);
            auto touch_other = orbiter.leftmostBottomVertex();
            auto dtouch = touch_sh - touch_other;
            auto top_other = orbiter.rightmostTopVertex() + dtouch;
            auto dnfp = top_other - nfp.second;
            shapelike::translate(nfp.first, dnfp);
        }

        template<class RawShape, class Circle = _Circle<TPoint<RawShape>> >
        Circle minimizeCircle(const RawShape& sh) {
            using Point = TPoint<RawShape>;
            using Coord = TCoord<Point>;

            auto& ctr = sl::contour(sh);
            if (ctr.empty()) return { {0, 0}, 0 };

            auto bb = sl::boundingBox(sh);
            auto capprx = bb.center();
            auto rapprx = pl::distance(bb.minCorner(), bb.maxCorner());


            opt::StopCriteria stopcr;
            stopcr.max_iterations = 30;
            stopcr.relative_score_difference = 1e-3;
            opt::TOptimizer<opt::Method::L_SUBPLEX> solver(stopcr);

            std::vector<double> dists(ctr.size(), 0);

            auto result = solver.optimize_min(
                [capprx, rapprx, &ctr, &dists](double xf, double yf) {
                    auto xt = Coord(std::round(getX(capprx) + rapprx * xf));
                    auto yt = Coord(std::round(getY(capprx) + rapprx * yf));

                    Point centr(xt, yt);

                    unsigned i = 0;
                    for (auto v : ctr) {
                        dists[i++] = pl::distance(v, centr);
                    }

                    auto mit = std::max_element(dists.begin(), dists.end());

                    assert(mit != dists.end());

                    return *mit;
                },
                opt::initvals(0.0, 0.0),
                opt::bound(-1.0, 1.0), opt::bound(-1.0, 1.0)
            );

            double oxf = std::get<0>(result.optimum);
            double oyf = std::get<1>(result.optimum);
            auto xt = Coord(std::round(getX(capprx) + rapprx * oxf));
            auto yt = Coord(std::round(getY(capprx) + rapprx * oyf));

            Point cc(xt, yt);
            auto r = result.score;

            return { cc, r };
        }

        template<class RawShape>
        _Circle<TPoint<RawShape>> boundingCircle(const RawShape& sh) {
            return minimizeCircle(sh);
        }

        template<class RawShape, class TBin = _Box<TPoint<RawShape>>>
        class _NofitPolyPlacer : 
            public PlacerBoilerplate<_NofitPolyPlacer<RawShape, TBin>, RawShape, TBin, NfpPConfig<RawShape>> 
        {

            using Base = PlacerBoilerplate<_NofitPolyPlacer<RawShape, TBin>,
                RawShape, TBin, NfpPConfig<RawShape>>;

            DECLARE_PLACER(Base)

            using Box = _Box<TPoint<RawShape>>;
            using MaxNfpLevel = nfp::MaxNfpLevel<RawShape>;

        public:

            using Pile = nfp::Shapes<RawShape>;

        private:

            // Norming factor for the optimization function
            const double m_norming_factor;
            Pile m_merged_pile;

        public:

            inline explicit _NofitPolyPlacer(const BinType& bin) :
                Base(bin),
                m_norming_factor(std::sqrt(sl::area(bin)))
            {
            }

            _NofitPolyPlacer(const _NofitPolyPlacer&) = default;
            _NofitPolyPlacer& operator=(const _NofitPolyPlacer&) = default;

#ifndef BP2D_COMPILER_MSVC12 // MSVC2013 does not support default move ctors
            _NofitPolyPlacer(_NofitPolyPlacer&&) = default;
            _NofitPolyPlacer& operator=(_NofitPolyPlacer&&) = default;
#endif

            static inline double overfit(const Box& bb, const RawShape& bin) {
                auto hull_bb = sl::boundingBox(bin);
                auto difference = hull_bb.center() - bb.center();
                _Rectangle<RawShape> rect(bb.width(), bb.height());
                rect.translate(bb.minCorner() + difference);
                return sl::isInside(rect.transformedShape(), bin) ? -1.0 : 1.0;
            }

            static inline double overfit(const RawShape& chull, const RawShape& bin) {
                auto hull_bb = sl::boundingBox(chull);
                auto bin_bb = sl::boundingBox(bin);
                auto difference = hull_bb.center() - bin_bb.center();
                auto hull_copy = chull;
                sl::translate(hull_copy, difference);
                return sl::isInside(hull_copy, bin) ? -1.0 : 1.0;
            }

            static inline double overfit(const RawShape& chull, const Box& bin)
            {
                auto bbch = sl::boundingBox(chull);
                return overfit(bbch, bin);
            }

            static inline double overfit(const Box& bb, const Box& bin)
            {
                auto wdiff = TCompute<RawShape>(bb.width()) - bin.width();
                auto hdiff = TCompute<RawShape>(bb.height()) - bin.height();
                double diff = .0;
                if (wdiff > 0) diff += double(wdiff);
                if (hdiff > 0) diff += double(hdiff);

                return diff;
            }

            static inline double overfit(const Box& bb, const _Circle<Vertex>& bin)
            {
                double boxr = 0.5 * pl::distance(bb.minCorner(), bb.maxCorner());
                double diff = boxr - bin.radius();
                return diff;
            }

            static inline double overfit(const RawShape& chull, const _Circle<Vertex>& bin)
            {
                double r = boundingCircle(chull).radius();
                double diff = r - bin.radius();
                return diff;
            }

            template<class Range = ConstItemRange<typename Base::DefaultIter>>
            PackResult trypack(Item& item,
                const Range& remaining = Range()) {
                auto result = _trypack(item, remaining);

                // Experimental
                // if(!result) repack(item, result);

                return result;
            }

            ~_NofitPolyPlacer() {
                clearItems();
            }

            inline void clearItems() {
                finalAlign(m_bin);
                Base::clearItems();
            }

            void preload(const ItemGroup& packeditems) {
                Base::preload(packeditems);
                if (m_config.on_preload)
                    m_config.on_preload(packeditems, m_config);
            }

            void acceptResult (PackResult& r)
            {
                if (r.overfit() == 0.0)
                {
                    m_merged_pile = nfp::merge(m_merged_pile, (*r.item_ptr_).transformedShape());
                }
            }

        private:
            using Shapes = TMultiShape<RawShape>;
            using Edges = EdgeCache<RawShape>;

            Shapes calculateNfp(const Item& input, Lvl<nfp::NfpLevel::CONVEX_ONLY>)
            {
                using namespace nfp;

                Shapes nfps(m_items.size());

                // /////////////////////////////////////////////////////////////////////
                // TODO: this is a workaround and should be solved in Item with mutexes
                // guarding the mutable members when writing them.
                // /////////////////////////////////////////////////////////////////////
                input.transformedShape();
                input.referenceVertex();
                input.rightmostTopVertex();
                input.leftmostBottomVertex();

                for (Item& item : m_items) {
                    item.transformedShape();
                    item.referenceVertex();
                    item.rightmostTopVertex();
                    item.leftmostBottomVertex();
                }
                // /////////////////////////////////////////////////////////////////////

                __parallel::enumerate(m_items.begin(), m_items.end(),
                    [&nfps, &input](const Item& item, size_t n)
                    {
                        auto& fixedp = item.transformedShape();
                        auto& orbp = input.transformedShape();
                        auto subnfp_r = noFitPolygon<NfpLevel::CONVEX_ONLY>(fixedp, orbp);
                        correctNfpTransform(subnfp_r, item, input);
                        nfps[n] = subnfp_r.first;
                    }, m_config.parallel);

                return nfp::merge(nfps);
            }

            template<class Level>
            Shapes calculateNfp(const Item& trsh, Level)
            { // Function for arbitrary level of nfp implementation

                // TODO: implement
                return {};
            }

            struct Optimum {
                double relpos;
                unsigned nfpidx;
                int hidx;
                Optimum(double pos, unsigned nidx) :
                    relpos(pos), nfpidx(nidx), hidx(-1) {
                }
                Optimum(double pos, unsigned nidx, int holeidx) :
                    relpos(pos), nfpidx(nidx), hidx(holeidx) {
                }
            };

            struct ResultCandidate
            {
                double score = std::numeric_limits<double>::max();
				Vertex final_translation = { 0, 0 };
				Radians final_rotation = 0;
                double best_overfit = 0;
            };

            class Optimizer : public opt::TOptimizer<opt::Method::L_SUBPLEX> {
            public:
                Optimizer(float accuracy = 1.f) {
                    opt::StopCriteria stopcr;
                    stopcr.max_iterations = unsigned(std::floor(1000 * accuracy));
                    stopcr.relative_score_difference = 1e-20;
                    this->stopcr_ = stopcr;
                }
            };

            template<class Range = ConstItemRange<typename Base::DefaultIter>>
            PackResult _trypack(
                Item& item,
                const Range& remains_range = Range()) {

                PackResult ret;

                bool can_pack = false;

                ItemGroup remains{};
                if (remains_range.valid) {
                    remains.insert(remains.end(), remains_range.from, remains_range.to);
                }

                double global_score = std::numeric_limits<double>::max();
                double best_overfit = std::numeric_limits<double>::max();
                auto initial_translation = item.translation();
                auto initial_rotation = item.rotation();
                Vertex final_translation = { 0, 0 };
                Radians final_rotation = initial_rotation;

                auto& bin = m_bin;
                double norm = m_norming_factor;
                auto pilebb = sl::boundingBox(m_merged_pile);
                auto binbb = sl::boundingBox(bin);

                // This is the kernel part of the object function that is
                // customizable by the library client
                std::function<double(const Item&)> object_function;
                if (m_config.object_function) object_function = m_config.object_function;
                else {

                    // Inside check has to be strict if no alignment was enabled
                    std::function<double(const Box&)> ins_check;
                    if (m_config.alignment == Config::Alignment::DONT_ALIGN)
                        ins_check = [&binbb, norm](const Box& fullbb) {
                        double result = 0;
                        if (!sl::isInside(fullbb, binbb))
                            result += norm;
                        return result;
                        };
                    else
                        ins_check = [&bin](const Box& fullbb) {
                        double miss = overfit(fullbb, bin);
                        miss = miss > 0 ? miss : 0;
                        return std::pow(miss, 2);
                        };

                    object_function = [norm, binbb, pilebb, ins_check](const Item& item)
                        {
                            auto itembb = item.boundingBox();
                            auto fullbb = sl::boundingBox(pilebb, itembb);

                            double score = pl::distance(itembb.minCorner(),
                                binbb.minCorner());
                            score = (fullbb.width() + fullbb.height()) / norm;

                            score += ins_check(fullbb);


                            return score;
                        };
                }

				if (m_config.before_packing)
					m_config.before_packing(m_merged_pile, m_items, remains);

                if (m_items.empty()) {
                    setInitialPosition(item);
                    auto best_translation = item.translation();
                    auto best_rotation = item.rotation();
                    best_overfit = overfit(item.transformedShape(), m_bin);

                    for (auto rot : m_config.rotations) {
                        item.translation(initial_translation);
                        item.rotation(initial_rotation + rot);
                        setInitialPosition(item);
                        double current_overfit = 0.;
                        if ((current_overfit = overfit(item.transformedShape(), m_bin)) < best_overfit) {
                            best_overfit = current_overfit;
                            best_translation = item.translation();
                            best_rotation = item.rotation();
                        }
                    }

                    can_pack = best_overfit <= 0;
                    item.rotation(best_rotation);
                    item.translation(best_translation);
                }
                else {
                    std::vector<Radians> rotations = m_config.rotation_callback ? m_config.rotation_callback(item) : m_config.rotations;

                    std::vector<ResultCandidate> candidates;
                    candidates.resize(rotations.size());

                    __parallel::enumerate(rotations.begin(), rotations.end(),
                        [&object_function, &bin, &item, &candidates, &initial_rotation, &initial_translation, this](double rotation, size_t n)
                        {
                            Pile merged_pile = m_merged_pile;
                            double best_local_overfit = std::numeric_limits<double>::max();

                            Item current_item = item;
                            current_item.translation(initial_translation);
                            current_item.rotation(initial_rotation + rotation);
                            current_item.boundingBox(); // fill the bb cache

							Shapes nfps = calculateNfp(current_item, Lvl<MaxNfpLevel::value>());

							auto nearest_vertex = current_item.referenceVertex();
							auto start_position = current_item.translation();

							std::vector<Edges> ecache;
							ecache.reserve(nfps.size());

							for (auto& nfp : nfps) {
								ecache.emplace_back(nfp);
								ecache.back().accuracy(m_config.accuracy);
							}

							// Our object function for placement
							auto rawobjfunc = [object_function, nearest_vertex, start_position]
							(Vertex v, Item& itm)
								{
									auto d = v - nearest_vertex;
									d += start_position;
									itm.translation(d);
									return object_function(itm);
								};

							auto getNfpPoint = [&ecache](const Optimum& opt)
								{
									return opt.hidx < 0 ? ecache[opt.nfpidx].coords(opt.relpos) :
										ecache[opt.nfpidx].coords(opt.hidx, opt.relpos);
								};

							auto alignment = m_config.alignment;

							auto boundaryCheck = [alignment, &merged_pile, &getNfpPoint,
								&current_item, &bin, &nearest_vertex, &start_position](const Optimum& o)
								{
									auto v = getNfpPoint(o);
									auto d = v - nearest_vertex;
									d += start_position;
                                    current_item.translation(d);

									merged_pile.emplace_back(current_item.transformedShape());
									auto chull = sl::convexHull(merged_pile);
									merged_pile.pop_back();

									double miss = 0;
									if (alignment == Config::Alignment::DONT_ALIGN)
										miss = sl::isInside(chull, bin) ? -1.0 : 1.0;
									else miss = overfit(chull, bin);

									return miss;
								};

							Optimum optimum(0, 0);
							double best_score = std::numeric_limits<double>::max();

							using OptResult = opt::Result<double>;
							using OptResults = std::vector<OptResult>;

							// Local optimization with the four polygon corners as
							// starting points
							for (unsigned ch = 0; ch < ecache.size(); ch++) {
								auto& cache = ecache[ch];

								OptResults results(cache.corners().size());

								auto& rofn = rawobjfunc;
								auto& nfpoint = getNfpPoint;
								float accuracy = m_config.accuracy;

                                __parallel::enumerate(
                                    cache.corners().begin(),
                                    cache.corners().end(),
                                    [&results, &current_item, &rofn, &nfpoint, ch, accuracy]
                                    (double pos, size_t n)
                                    {
                                        Optimizer solver(accuracy);

                                        Item itemcpy = current_item;
                                        auto contour_ofn = [&rofn, &nfpoint, ch, &itemcpy]
                                        (double relpos)
                                            {
                                                Optimum op(relpos, ch);
                                                return rofn(nfpoint(op), itemcpy);
                                            };

                                        try {
                                            results[n] = solver.optimize_min(contour_ofn,
                                                opt::initvals<double>(pos),
                                                opt::bound<double>(0, 1.0)
                                            );
                                        }
                                        catch (std::exception& e) {
                                            derr() << "ERROR: " << e.what() << "\n";
                                        }
                                    }, m_config.parallel);

                                auto resultcomp =
                                    [](const OptResult& r1, const OptResult& r2) {
                                    return r1.score < r2.score;
                                    };

                                auto minimal_result = *std::min_element(results.begin(), results.end(),
                                    resultcomp);

                                if (minimal_result.score < best_score) {
                                    Optimum o(std::get<0>(minimal_result.optimum), ch, -1);
                                    double miss = boundaryCheck(o);
                                    if (miss <= 0) {
                                        best_score = minimal_result.score;
                                        optimum = o;
                                    }
                                    else {
                                        best_local_overfit = std::min(miss, best_local_overfit);
                                    }
                                }

                                for (unsigned hidx = 0; hidx < cache.holeCount(); ++hidx) {
                                    results.clear();
                                    results.resize(cache.corners(hidx).size());

                                    // TODO : use parallel for
                                    __parallel::enumerate(cache.corners(hidx).begin(),
                                        cache.corners(hidx).end(),
                                        [&results, &current_item, &nfpoint,
                                        &rofn, ch, hidx, accuracy]
                                        (double pos, size_t n)
                                        {
                                            Optimizer solver(accuracy);

                                            Item itmcpy = current_item;
                                            auto hole_ofn =
                                                [&rofn, &nfpoint, ch, hidx, &itmcpy]
                                                (double pos)
                                                {
                                                    Optimum opt(pos, ch, hidx);
                                                    return rofn(nfpoint(opt), itmcpy);
                                                };

                                            try {
                                                results[n] = solver.optimize_min(hole_ofn,
                                                    opt::initvals<double>(pos),
                                                    opt::bound<double>(0, 1.0)
                                                );

                                            }
                                            catch (std::exception& e) {
                                                derr() << "ERROR: " << e.what() << "\n";
                                            }
                                        }, m_config.parallel);

                                    auto hmr = *std::min_element(results.begin(),
                                        results.end(),
                                        resultcomp);

                                    if (hmr.score < best_score) {
                                        Optimum o(std::get<0>(hmr.optimum),
                                            ch, hidx);
                                        double miss = boundaryCheck(o);
                                        if (miss <= 0.0) {
                                            best_score = hmr.score;
                                            optimum = o;
                                        }
                                        else {
                                            best_local_overfit = std::min(miss, best_local_overfit);
                                        }
                                    }
                                }
                            }

                            {
								auto nfp_point = getNfpPoint(optimum) - nearest_vertex;
								nfp_point += start_position;

                                ResultCandidate& candidate = candidates[n];
                                candidate.score = best_score;
                                candidate.final_translation = nfp_point;
                                candidate.final_rotation = initial_rotation + rotation;
                                candidate.best_overfit = best_local_overfit;
                            }
                        }, m_config.parallel);

                    auto best_candidate = std::min_element(candidates.begin(), candidates.end(),
                        [](const ResultCandidate& r1, const ResultCandidate& r2) {
                            return r1.score < r2.score;
                        });

                    if (best_candidate->score >= global_score)
                    {
						best_overfit = (*std::min_element(candidates.begin(), candidates.end(),
							[](const ResultCandidate& r1, const ResultCandidate& r2) {
								return r1.best_overfit < r2.best_overfit;
							})).best_overfit;
                    }
                    else
                    {
						final_translation = best_candidate->final_translation;
						final_rotation = best_candidate->final_rotation;

						item.translation(final_translation);
						item.rotation(final_rotation);
                        can_pack = true;
                    }
                }

                if (can_pack) {
                    ret = PackResult(item);
                }
                else {
                    ret = PackResult(best_overfit);
                }

				if (m_config.after_packing)
					m_config.after_packing(m_merged_pile, m_items, remains);

                return ret;
            }

            // Polygon Align
            inline void finalAlign(const RawShape& pbin) {
                auto bbin = sl::boundingBox(pbin);
                finalAlign(bbin);
            }

            // Circle Align
            inline void finalAlign(_Circle<TPoint<RawShape>> cbin) {
                if (m_items.empty() ||
                    m_config.alignment == Config::Alignment::DONT_ALIGN) return;

                nfp::Shapes<RawShape> m;
                m.reserve(m_items.size());
                for (Item& item : m_items) m.emplace_back(item.transformedShape());

                auto c = boundingCircle(sl::convexHull(m));

                auto d = cbin.center() - c.center();
                for (Item& item : m_items) item.translate(d);
            }

            // Box Align
            inline void finalAlign(Box bbin) {
                if (m_items.empty() ||
                    m_config.alignment == Config::Alignment::DONT_ALIGN) return;

                Box bb = m_items.front().get().boundingBox();
                for (Item& item : m_items)
                    bb = sl::boundingBox(item.boundingBox(), bb);

                Vertex ci, cb;

                switch (m_config.alignment) {
                case Config::Alignment::CENTER: {
                    ci = bb.center();
                    cb = bbin.center();
                    break;
                }
                case Config::Alignment::BOTTOM_LEFT: {
                    ci = bb.minCorner();
                    cb = bbin.minCorner();
                    break;
                }
                case Config::Alignment::BOTTOM_RIGHT: {
                    ci = { getX(bb.maxCorner()), getY(bb.minCorner()) };
                    cb = { getX(bbin.maxCorner()), getY(bbin.minCorner()) };
                    break;
                }
                case Config::Alignment::TOP_LEFT: {
                    ci = { getX(bb.minCorner()), getY(bb.maxCorner()) };
                    cb = { getX(bbin.minCorner()), getY(bbin.maxCorner()) };
                    break;
                }
                case Config::Alignment::TOP_RIGHT: {
                    ci = bb.maxCorner();
                    cb = bbin.maxCorner();
                    break;
                }
                default:; // DONT_ALIGN
                }

                auto d = cb - ci;
                for (Item& item : m_items) item.translate(d);
            }

            void setInitialPosition(Item& item) {
                Box bb = item.boundingBox();

                Vertex ci, cb;
                auto bbin = sl::boundingBox(m_bin);

                switch (m_config.starting_point) {
                case Config::Alignment::CENTER: {
                    ci = bb.center();
                    cb = bbin.center();
                    break;
                }
                case Config::Alignment::BOTTOM_LEFT: {
                    ci = bb.minCorner();
                    cb = bbin.minCorner();
                    break;
                }
                case Config::Alignment::BOTTOM_RIGHT: {
                    ci = { getX(bb.maxCorner()), getY(bb.minCorner()) };
                    cb = { getX(bbin.maxCorner()), getY(bbin.minCorner()) };
                    break;
                }
                case Config::Alignment::TOP_LEFT: {
                    ci = { getX(bb.minCorner()), getY(bb.maxCorner()) };
                    cb = { getX(bbin.minCorner()), getY(bbin.maxCorner()) };
                    break;
                }
                case Config::Alignment::TOP_RIGHT: {
                    ci = bb.maxCorner();
                    cb = bbin.maxCorner();
                    break;
                }
                default:;
                }

                auto d = cb - ci;
                item.translate(d);
            }

            void placeOutsideOfBin(Item& item) {
                auto&& bb = item.boundingBox();
                Box binbb = sl::boundingBox(m_bin);

                Vertex v = { getX(bb.maxCorner()), getY(bb.minCorner()) };

                Coord dx = getX(binbb.maxCorner()) - getX(v);
                Coord dy = getY(binbb.maxCorner()) - getY(v);

                item.translate({ dx, dy });
            }
        };
    }
}

#endif // NOFITPOLY_H
