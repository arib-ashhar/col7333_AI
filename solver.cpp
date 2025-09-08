#include "solver.h"
#include <iostream>
#include <chrono>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <optional>

#define ll long long

using namespace std;
// You can add any helper functions or classes you need here.

double calculateScore(const ProblemData& data, const Solution& solution);

struct PackageWeights {
    double wDry, wPerish, wOther;
};

static PackageWeights getPackageWeights(const ProblemData& P) {
    PackageWeights W;
    W.wDry    = P.packages[0].weight;
    W.wPerish = P.packages[1].weight;
    W.wOther  = P.packages[2].weight;
    return W;
}


// Helper Functions

double calDistance(const Point& a, const Point& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

const Helicopter& heliById(const ProblemData& P, int id) { 
    return P.helicopters[id-1]; 
}

double tripDistance(const ProblemData& P, const Helicopter& H, const Trip& T) {
    if (T.drops.empty()) return 0.0;
    const Point home = P.cities[H.home_city_id - 1];
    double d = 0.0; 
    Point cur = home;
    for (const auto& drop : T.drops) { 
        const Point v = P.villages[drop.village_id - 1].coords; 
        d += calDistance(cur, v); cur = v; 
    }
    d += calDistance(cur, home);
    return d;
}

double tripWeight(const ProblemData& P, const Trip& T) {
    return T.dry_food_pickup * P.packages[0].weight
         + T.perishable_food_pickup * P.packages[1].weight
         + T.other_supplies_pickup * P.packages[2].weight;
}

void setDropValues(Trip& T){
    long long d=0,p=0,o=0; 
    for (auto& r:T.drops) { 
        d+=r.dry_food; 
        p+=r.perishable_food; 
        o+=r.other_supplies; 
    }
    T.dry_food_pickup=(int)d;
    T.perishable_food_pickup=(int)p;
    T.other_supplies_pickup=(int)o;
}

bool isTripFeasible(const ProblemData& P, const Helicopter& H, Trip& T){
    setDropValues(T);
    if (tripWeight(P,T) > H.weight_capacity) return false;
    if (tripDistance(P,H,T) > H.distance_capacity) return false;
    return true;
}

double calTotalRouteDist(vector<int>& vIdx, const Helicopter& h, const ProblemData& P) {
    Point home = P.cities[h.home_city_id - 1];
    double dist = 0.0;

    if (vIdx.empty()) return 0.0;

    // home -> first village
    dist += calDistance(home, P.villages[vIdx.front()].coords);
    for (size_t i = 1; i < vIdx.size(); i++) {
        dist += calDistance(P.villages[vIdx[i - 1]].coords, P.villages[vIdx[i]].coords);
    }
    // last village -> home
    dist += calDistance(P.villages[vIdx.back()].coords, home);
    return dist;
}

double calUpdatedRouteDist(vector<int>& vIdx, int appendIdx, const Helicopter& h, const ProblemData& P) {
    vector<int> tmp = vIdx;
    tmp.push_back(appendIdx);
    return calTotalRouteDist(tmp, h, P);
}

/*
 Construct Trip with drops and weights for the assigned villages for Heli h.
 */
static Trip constructTripForRoute(vector<int>& routeIdx, const Helicopter& h, const ProblemData& P) {
    Trip trip{};
    if (routeIdx.empty()) return trip;

    PackageWeights W = getPackageWeights(P);
    double weightLeft = h.weight_capacity;

    trip.drops.reserve(routeIdx.size());
    for (ll vi : routeIdx) {
        Drop d{};
        d.village_id = P.villages[vi].id;
        d.dry_food = d.perishable_food = d.other_supplies = 0;
        trip.drops.push_back(d);
    }

    // Helper lambda to try to drop one unit of a package for each village in order
    auto distribute_one_round = [&](char type, double w) {
        bool dropped_any = false;
        for (auto& d : trip.drops) {
            if (weightLeft < w) break;
            switch (type) {
                case 'P': d.perishable_food += 1; trip.perishable_food_pickup += 1; break;
                case 'D': d.dry_food        += 1; trip.dry_food_pickup        += 1; break;
                case 'O': d.other_supplies  += 1; trip.other_supplies_pickup  += 1; break;
            }
            weightLeft -= w;
            dropped_any = true;
            if (weightLeft < min({W.wDry, W.wPerish, W.wOther})) break; // no more full units
        }
        return dropped_any;
    };

    // Priority: Perishable -> Dry -> Other
    while (weightLeft >= min({W.wDry, W.wPerish, W.wOther})) {
        bool anyPackageAdded = false;
        if (weightLeft >= W.wPerish) anyPackageAdded |= distribute_one_round('P', W.wPerish);
        if (weightLeft >= W.wDry)    anyPackageAdded |= distribute_one_round('D', W.wDry);
        if (weightLeft >= W.wOther)  anyPackageAdded |= distribute_one_round('O', W.wOther);
        if (!anyPackageAdded) break;
    }

    return trip;
}

// Random Restart for Solution state
Solution randomRestart(const ProblemData& problem) {

    Solution solution;
    if (problem.helicopters.empty()) return solution;

    // RNG
    std::mt19937_64 rng(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> heliDist(0, (int)problem.helicopters.size() - 1);

    vector<vector<int>> assigned;
    assigned.resize(problem.helicopters.size());
    vector<int> vIndices(problem.villages.size());
    std::iota(vIndices.begin(), vIndices.end(), 0);
    std::shuffle(vIndices.begin(), vIndices.end(), rng);
    
    // Randomly assign each village to a helicopter
    for (ll vi : vIndices) {
        int hidx = heliDist(rng);
        assigned[hidx].push_back(vi);
    }

    // For each helicopter, build trips that satisfy distance_capacity and cumulative DMax
    for (size_t hidx = 0; hidx < problem.helicopters.size(); hidx++) {
        const Helicopter& H = problem.helicopters[hidx];
        HelicopterPlan plan{};
        plan.helicopter_id = H.id;

        auto& myVillages = assigned[hidx];
        std::shuffle(myVillages.begin(), myVillages.end(), rng);

        double remainingDmax = problem.d_max; // total distance for all trips for helicopter 

        vector<int> currentRoute;
        auto insertCurrentRoute = [&]() {
            if (currentRoute.empty()) return;
            double tripDist = calTotalRouteDist(currentRoute, H, problem);
            //cout<<"Trip Distance"<<tripDist<<endl;
            if (tripDist <= H.distance_capacity && tripDist <= remainingDmax + 1e-9) {
                Trip t = constructTripForRoute(currentRoute, H, problem);
                if (!t.drops.empty()) {
                    plan.trips.push_back(std::move(t));
                    remainingDmax -= tripDist;
                }
            }
            currentRoute.clear();
        };

        for (int vi : myVillages) {
            double newDist = calUpdatedRouteDist(currentRoute, vi, H, problem);

            // If adding this village would exceed per-trip capacity or remainingDmax,
            // if not add it to the plan
            if (!currentRoute.empty() && (newDist > H.distance_capacity || newDist > remainingDmax)) {
                insertCurrentRoute();
            }

            // If current route is empty, check if this village alone is feasible.
            if (currentRoute.empty()) {
                vector<int> singleVillage{vi};
                double soloDist = calTotalRouteDist(singleVillage, H, problem);
                if (soloDist <= H.distance_capacity && soloDist <= remainingDmax) {
                    currentRoute.push_back(vi);
                } else {
                    continue;
                }
            } else {
                // add this randomly selected village to the trip 
                currentRoute.push_back(vi);
            }
        }

        // Finalize any trailing route
        insertCurrentRoute();

        solution.push_back(std::move(plan));
    }

    return solution;
}


// Local Search
bool tryReorderingDropsInEachTrip(const ProblemData& P, Solution& S, double& bestScore){
    for (auto& plan : S) {
        const Helicopter& H = heliById(P, plan.helicopter_id);
        for (size_t ti=0; ti<plan.trips.size(); ++ti){
            Trip& T = plan.trips[ti];
            const size_t n = T.drops.size();
            if (n < 3) continue;
            for (size_t i=0; i+1<n; ++i){
                for (size_t k=i+1; k<n; ++k){
                    Trip cand = T;
                    std::reverse(cand.drops.begin()+i, cand.drops.begin()+k+1);
                    if (!isTripFeasible(P,H,cand)) continue;

                    Solution S2 = S;
                    S2[ &plan - &S[0] ].trips[ti] = std::move(cand);
                    double newScore = calculateScore(P, S2);
                    // Update if found better solution
                    if (newScore > bestScore) { 
                        S = std::move(S2);
                        bestScore = newScore;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool localSearch(const ProblemData& P, Solution& S, double& bestScore, int rounds=3){
    bool improved = false;
    for (int r=0; r<rounds; r++){
        while (tryReorderingDropsInEachTrip(P, S, bestScore))
            improved = true;
    }
    return improved;
}

/**
 * @brief The main function to implement your search/optimization algorithm.
 * * This is a placeholder implementation. It creates a simple, likely invalid,
 * plan to demonstrate how to build the Solution object. 
 * * TODO: REPLACE THIS ENTIRE FUNCTION WITH YOUR ALGORITHM.
 */
Solution solve(const ProblemData& problem) {

    // --- START OF PLACEHOLDER LOGIC ---
    // This is a naive example: send each helicopter on one trip to the first village.
    // This will definitely violate constraints but shows the structure.
    
    using clock = std::chrono::steady_clock;
    cout << "Starting solver..." << endl;

    const double run_minutes = std::max(0.0, problem.time_limit_minutes * 0.9);
    // taking runtime = 0.45 * MaxTime allowed for Random Start
    const auto runtime_ms = std::chrono::milliseconds(
        static_cast<ll>(run_minutes * 60.0 * 1000.0)
    );

    const auto start = clock::now();
    auto deadline = start + runtime_ms;

    Solution champion;
    double champScore = -1;
    int iter=0;

    while (clock::now() < deadline) {
        iter++;
        Solution candidateSol = randomRestart(problem);
        double newScore = calculateScore(problem, candidateSol);

        if(newScore > champScore) {
            champion = std::move(candidateSol);
            champScore = newScore;
            cout<< "New Champ: "<<champScore<<endl;
            cout<< "Performing Local Search: "<<endl;
            localSearch(problem, champion, champScore, 25);
        }
    }
    //cout<<"iterations: "<<iter<<endl;
    
    // --- END OF PLACEHOLDER LOGIC ---

    cout << "Solver finished." << endl;
    return champion;
}



// Calculates objective for a constructed Solution.
// Returns -1.0 if any constraint is violated.
double calculateScore(const ProblemData& data, const Solution& solution) {
    bool constraint_violated = false;

    std::vector<double> food_delivered(data.villages.size() + 1, 0.0);
    std::vector<double> other_delivered(data.villages.size() + 1, 0.0);
    std::vector<double> village_values(data.villages.size() + 1, 0.0);
    std::vector<double> helicopter_total_distances(data.helicopters.size() + 1, 0.0);

    double total_trip_cost = 0.0;

    std::unordered_map<int, const HelicopterPlan*> planById;
    for (const auto& plan : solution) {
        if (plan.helicopter_id <= 0 || plan.helicopter_id > (int)data.helicopters.size()) {
            //std::cerr << "Error: Invalid helicopter ID " << plan.helicopter_id << " in Solution.\n";
            return -1.0;
        }
        if (planById.count(plan.helicopter_id)) {
            //std::cerr << "*** WARNING: Duplicate plan provided for helicopter " << plan.helicopter_id << ".\n";
            constraint_violated = true;
        }
        planById[plan.helicopter_id] = &plan;
    }

    for (int helicopter_id = 1; helicopter_id <= (int)data.helicopters.size(); ++helicopter_id) {
        const auto& helicopter = data.helicopters[helicopter_id - 1];

        Point home_city_coords = data.cities[helicopter.home_city_id - 1];

        const HelicopterPlan* planPtr = nullptr;
        if (auto it = planById.find(helicopter_id); it != planById.end()) {
            planPtr = it->second;
        }

        if (!planPtr) {
            continue;
        }

        // Evaluate each trip
        for (size_t t = 0; t < planPtr->trips.size(); ++t) {
            const Trip& trip = planPtr->trips[t];

            int d = trip.dry_food_pickup;
            int p = trip.perishable_food_pickup;
            int o = trip.other_supplies_pickup;

            double trip_weight =
                d * data.packages[0].weight +
                p * data.packages[1].weight +
                o * data.packages[2].weight;

            if (trip_weight > helicopter.weight_capacity + 1e-9) {
                // std::cout << "*** WARNING: Heli " << helicopter_id << ", Trip " << (t + 1)
                //           << " exceeds weight capacity (" << trip_weight
                //           << " > " << helicopter.weight_capacity << ").\n";
                constraint_violated = true;
            }

            Point current_location = home_city_coords;
            double trip_distance = 0.0;

            int total_d_dropped = 0, total_p_dropped = 0, total_o_dropped = 0;

            for (const Drop& drop : trip.drops) {
                int village_id = drop.village_id;
                int vd = drop.dry_food, vp = drop.perishable_food, vo = drop.other_supplies;

                total_d_dropped += vd;
                total_p_dropped += vp;
                total_o_dropped += vo;

                if (village_id <= 0 || village_id > (int)data.villages.size()) {
                    // std::cerr << "Error: Invalid village ID " << village_id
                    //           << " in Heli " << helicopter_id
                    //           << ", Trip " << (t + 1) << ".\n";
                    return -1.0;
                }
                const auto& village = data.villages[village_id - 1];

                // --- Value capping ---
                double max_food_needed = village.population * 9.0;
                double food_room_left = std::max(0.0, max_food_needed - food_delivered[village_id]);
                double food_in_this_drop = (double)vd + (double)vp;
                double effective_food_this_drop = std::min(food_in_this_drop, food_room_left);
                double effective_vp = std::min((double)vp, effective_food_this_drop);
                double value_from_p = effective_vp * data.packages[1].value;
                double remaining_effective_food = effective_food_this_drop - effective_vp;
                double effective_vd = std::min((double)vd, remaining_effective_food);
                double value_from_d = effective_vd * data.packages[0].value;

                double max_other_needed = village.population * 1.0;
                double other_room_left = std::max(0.0, max_other_needed - other_delivered[village_id]);
                double effective_vo = std::min((double)vo, other_room_left);
                double value_from_o = effective_vo * data.packages[2].value;

                village_values[village_id] += value_from_p + value_from_d + value_from_o;

                food_delivered[village_id] += food_in_this_drop;
                other_delivered[village_id] += vo;

                trip_distance += distance(current_location, village.coords);
                current_location = village.coords;
            }

            if (total_d_dropped > d || total_p_dropped > p || total_o_dropped > o) {
                // std::cout << "*** WARNING: Heli " << helicopter_id << ", Trip " << (t + 1)
                //           << " drops more packages than picked up.\n";
                constraint_violated = true;
            }

            if (!trip.drops.empty()) {
                trip_distance += distance(current_location, home_city_coords);
            }

            if (trip_distance > helicopter.distance_capacity + 1e-9) {
                // std::cout << "*** WARNING: Heli " << helicopter_id << ", Trip " << (t + 1)
                //           << " exceeds trip distance capacity (" << trip_distance
                //           << " > " << helicopter.distance_capacity << ").\n";
                constraint_violated = true;
            }

            helicopter_total_distances[helicopter_id] += trip_distance;

            double trip_cost = (!trip.drops.empty())
                                   ? (helicopter.fixed_cost + helicopter.alpha * trip_distance)
                                   : 0.0;
            total_trip_cost += trip_cost;
        }

        if (helicopter_total_distances[helicopter_id] > data.d_max + 1e-9) {
            // std::cout << "*** WARNING: Heli " << helicopter_id
            //           << " exceeds DMax (" << helicopter_total_distances[helicopter_id]
            //           << " > " << data.d_max << ").\n";
            constraint_violated = true;
        }
    }

    double total_value = std::accumulate(village_values.begin(), village_values.end(), 0.0);
    double final_score = total_value - total_trip_cost;

    // std::cout << "\n--- Final Calculation ---\n";
    // std::cout << "Total Value Gained: " << total_value << "\n";
    // std::cout << "Total Trip Cost   : " << total_trip_cost << "\n";
    // std::cout << "Objective Score   = " << total_value << " - " << total_trip_cost
    //           << " = " << final_score << "\n";

    if (constraint_violated) {
        //std::cout << "\n*** WARNING: CONSTRAINTS VIOLATED. Score is invalid. ***\n";
        return -1.0;
    }
    //std::cout << "\n--- All constraints satisfied. ---\n";
    return final_score;
}

