package baobab;

import java.io.*;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.*;

public class A {

    public static void main(String[] args) {
        Solver solver = new Solver();
    }

    static class Solver {
        IO io;

        public Solver() {
            this.io = new IO();
            try {
                solve();
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                io.close();
            }
        }

        /****************************** START READING HERE ********************************/

        // Constants from problem statement
        double SANTA_HOME_LATITUDE = 68.073611;
        double SANTA_HOME_LONGITUDE = 29.315278;
        int MAX_TRIP_WEIGHT = 10000000;

        // Hyperparameters
        double randomSize = 0.58; // At one point this was local walked to 0.58 (TODO again)
        double multiplierToSlightlyExpandMaxSparsity = 1.1; // Optimal between 1.0 - 1.1
        double UTZ_CLUSTER_GOAL = 0.92; // Optimal between 0.92-0.96 (if UTZ_TRIP_GOAL is locked to 0.98)
        double UTZ_TRIP_GOAL = 0.98; // 0.989 would be slightly better, but it's much slower and sometimes stuck at infinite loop

        // Reduce print spam
        boolean verbose = false;
        double startTime = System.currentTimeMillis();
        double lastPrintedScore = Double.POSITIVE_INFINITY;
        double lowestKnownScore = Double.POSITIVE_INFINITY;
        long lastScorePrintTime = 0;
        long printIntervalSeconds = 1;

        // Reduce file spam
        double bestSavedVal = Double.POSITIVE_INFINITY;
        double lastScoreSaveTime = 0;
        double saveIntervalSeconds = 120;

        // Current solution state
        List<Trip> trips;

        // Auxiliary
        Random rng;

        void solve() throws Exception {
            readInput();
            preCalcAllDistances();

            long seed = System.currentTimeMillis();
            rng = new Random(seed);
            System.out.println("Seeding random with " + seed);

            int actionType = 5;
                 if (actionType == 1) loadAndComparePreviousSolutions();
            else if (actionType == 2) createRouteFromScratch();
            else if (actionType == 3) foreverCreateRoutesFromScratch();
            else if (actionType == 4) foreverImproveAnExistingRoute();
            else if (actionType == 5) randomRouteHillClimb();
            //else if (actionType == 7) randomRouteSimulatedAnnealing();
        }

        void randomRouteHillClimb() {
            createBadRouteRandomly();
            while (true) {
                periodicals();
                //localWalkImprovementsToTrips(trips);
                //localWalkSingleDestinationTransfers();
                proposeRandomSwap();
                proposeRandomSteal();
            }
        }

        void foreverImproveAnExistingRoute() throws FileNotFoundException {
            Double val = loadPreviouslyFoundSolution(getFilePath("santamap706"));
            if (val == null) return;
            while (true) {
                periodicals();
                //probabilisticDetachment();
                //localWalkSingleDestinationTransfers();
                //shuffleAndSortIndividualTrips();
                proposeRandomSwap();
                proposeRandomSteal();
            }
            // TODO simulated annealing
        }

        void proposeRandomSwap() {
            Trip trip1 = trips.get(rng.nextInt(trips.size()));
            Trip trip2 = trips.get(rng.nextInt(trips.size()));
            if (trip1.isEmpty() || trip2.isEmpty()) return;
            int stop1index = rng.nextInt(trip1.ids.size());
            int stop2index = rng.nextInt(trip2.ids.size());
            int stop1id = trip1.ids.get(stop1index);
            int stop2id = trip2.ids.get(stop2index);
            if (stop1id == stop2id) return;
            double valThreshold = 0;
            if (trip1 != trip2) {
                if (trip1.weightSum + w[stop2id] - w[stop1id] > MAX_TRIP_WEIGHT) return;
                if (trip2.weightSum + w[stop1id] - w[stop2id] > MAX_TRIP_WEIGHT) return;
                double replacementVal1 = getReplacementVal(stop1id, stop2index, trip2);
                double replacementVal2 = getReplacementVal(stop2id, stop1index, trip1);
                double proposalVal = replacementVal1+replacementVal2;
                if (proposalVal >= valThreshold) {
                    trip1.addStop(stop1index, stop2id);
                    trip2.addStop(stop2index, stop1id);
                    trip1.removeId(stop1id);
                    trip2.removeId(stop2id);
                }
            } else {
                Trip trip = trip1;
                double proposalVal = getSwapVal(stop1index, stop2index, trip);
                if (proposalVal >= valThreshold) {
                    trip.removeIndex(stop1index);
                    trip.addStop(stop1index, stop2id);
                    trip.removeIndex(stop2index);
                    trip.addStop(stop2index, stop1id);
                }
            }


        }

        void proposeRandomSteal() {
            Trip giver = trips.get(rng.nextInt(trips.size()));
            Trip taker = trips.get(rng.nextInt(trips.size()));
            if (giver.isEmpty()) return;
            if (giver == taker) return;
            int giverIndex = rng.nextInt(giver.ids.size());
            int takerIndex = rng.nextInt(taker.ids.size()+1);
            int stopId = giver.ids.get(giverIndex);
            if (taker.weightSum + w[stopId] > MAX_TRIP_WEIGHT) return;
            double removalVal = getRemovalVal(giverIndex, giver);
            double insertionVal = getInsertionVal(stopId, takerIndex, taker);
            double proposalVal = removalVal + insertionVal;
            if (proposalVal >= 0) {
                taker.addStop(takerIndex, stopId);
                giver.removeIndex(giverIndex);
            }
        }

        double getRemovalVal(int index, Trip trip) {
            int removeId = trip.ids.get(index);
            int prevId = (index>0 ? trip.ids.get(index-1) : 1);
            int nextId = (index<trip.ids.size()-1 ? trip.ids.get(index+1) : 1);
            return (dist[prevId][removeId] + dist[removeId][nextId]) - dist[prevId][nextId];
        }

        double getReplacementVal(int newId, int index, Trip trip) {
            int prevId = (index>0 ? trip.ids.get(index-1) : 1);
            int currId = trip.ids.get(index);
            int nextId = (index<trip.ids.size()-1 ? trip.ids.get(index+1) : 1);
            return dist[prevId][currId] + dist[currId][nextId] - (dist[prevId][newId] + dist[newId][nextId]);
        }

        double getInsertionVal(int newId, int index, Trip trip) {
            int prevId = (index>0 ? trip.ids.get(index-1) : 1);
            int nextId = (index<trip.ids.size() ? trip.ids.get(index) : 1);
            return dist[prevId][nextId] - (dist[prevId][newId] + dist[newId][nextId]);
        }

        double getSwapVal(int index1, int index2, Trip trip) {
            if (index1 > index2) {
                int helper = index1;
                index1 = index2;
                index2 = helper;
            }
            int id1 = trip.ids.get(index1);
            int id2 = trip.ids.get(index2);
            if (index1+1 == index2) {
                int prev = (index1>0 ? trip.ids.get(index1-1) : 1);
                int next = (index2<trip.ids.size()-1 ? trip.ids.get(index2+1) : 1);
                return dist[prev][id1] + dist[id2][next] - (dist[prev][id2] + dist[id1][next]);
            } else {
                return getReplacementVal(id2, index1, trip) + getReplacementVal(id1, index2, trip);
            }
        }

        void createBadRouteRandomly() {
            System.out.println("Generating a random solution from scratch...");
            trips = new ArrayList<>();
            for (int stop=2; stop<endId; stop++) {
                Collections.shuffle(trips);
                for (int tripIndex=0 ;; tripIndex++) {
                    if (tripIndex >= trips.size()) trips.add(new Trip());
                    Trip trip = trips.get(tripIndex);
                    if (trip.weightSum + w[stop] <= MAX_TRIP_WEIGHT) {
                        trip.addStop(stop);
                        break;
                    }
                }
            }
            // These provide some flexibility when moving between solutions
            trips.add(new Trip());
            trips.add(new Trip());
        }







        void shuffleAndSortIndividualTrips() {
            // This seems to absolutely murder results
            double valAtStart = calcScore(trips);
            for (Trip trip : trips) {
                Collections.shuffle(trip.ids);
                nnTSP(trip);
                localWalkImprovementsToTrip(trip);
            }
            double valAtEnd = calcScore(trips);
            double diff = valAtStart - valAtEnd;
            System.out.println(formatAnsValue(valAtEnd) + "(" + formatAnsValue(diff) + ") by shuffleAndSort");
        }

        void nnTSP(Trip trip) {
            List<Integer> newOrder = new ArrayList<>();
            boolean[] newUsed = new boolean[endId];
            int prevId = 1;
            while (newOrder.size() < trip.ids.size()) {
                int closestId = -1;
                Double minDist = Double.POSITIVE_INFINITY;
                for (int id : trip.ids) {
                    if (newUsed[id]) continue;
                    if (dist[prevId][id] < minDist) {
                        minDist = dist[prevId][id];
                        closestId = id;
                    }
                }
                newUsed[closestId] = true;
                newOrder.add(closestId);
                prevId = closestId;
            }
            trip.used = newUsed;
            trip.ids = newOrder;
            // meters will be updated later. weightSum doesn't change.
        }

        // Detach all nodes whose "detour" value exceeds some threshold. Insert them back to good trips in random order.
        void probabilisticDetachment() {
            // TODO: revert when unsuccessful
            double valBeforeDetachment = calcScore(trips);
            List<DetachmentCandidate> d = new ArrayList<>();
            for (Trip trip : trips) {
                for (int i=0; i<trip.ids.size(); i++) {
                    int currId = trip.ids.get(i);
                    if (w[currId] > 50000) continue; // Only detach small items
                    int prevId = (i > 0 ? trip.ids.get(i-1) : 1);
                    int nextId = (i < trip.ids.size()-1 ? trip.ids.get(i+1) : 1);
                    double val = dist[prevId][currId] + dist[currId][nextId] - dist[prevId][nextId];
                    val *= (1 + 2 * rng.nextDouble()); // probabilistically so some random nodes get detached at the same time, too
                    d.add(new DetachmentCandidate(trip, i, val));
                }
            }
            Collections.sort(d, Collections.reverseOrder());
            List<Integer> detachedIds = new ArrayList<>();
            HashMap<Trip, List<Integer>> removeIndices = new HashMap<>();
            for (int i=0; i<d.size() && d.get(i).val > 20000; i++) {
                DetachmentCandidate detach = d.get(i);
                int id = detach.trip.ids.get(detach.index);
                List<Integer> removeList = removeIndices.get(detach.trip);
                if (removeList == null) {
                    removeList = new ArrayList<>();
                    removeIndices.put(detach.trip, removeList);
                }
                removeList.add(detach.index);
                detachedIds.add(id);
            }
            for (Map.Entry<Trip, List<Integer>> entry : removeIndices.entrySet()) {
                Trip trip = entry.getKey();
                List<Integer> list = entry.getValue();
                Collections.sort(list, Collections.reverseOrder());
                for (int i : list) {
                    trip.removeIndex(i); // No need to update meters yet
                }
            }

            Collections.shuffle(detachedIds);
            for (int currId : detachedIds) {

                //System.out.println("Placing id " + currId);

                // These will be filled
                Trip bestCandidate = null;
                double bestCandidateInsertionVal = Double.NEGATIVE_INFINITY;
                int bestCandidateInsertionPos = 0;

                for (Trip taker : trips) {
                    if (taker.weightSum + w[currId] > MAX_TRIP_WEIGHT) continue;
                    int takerBestPos = -1;
                    double takerBestPosInsertionVal = Integer.MIN_VALUE;
                    for (int newPos=0; newPos<=taker.ids.size(); newPos++) {
                        int prevId = (newPos>0 ? taker.ids.get(newPos-1) : 1); // trip starts and ends at 1
                        int nextId = (newPos<taker.ids.size() ? taker.ids.get(newPos) : 1); // item which will be at newPos+1 after displacement
                        double insertionVal = dist[prevId][nextId] - (dist[prevId][currId] + dist[currId][nextId]);
                        if (insertionVal > takerBestPosInsertionVal) {
                            takerBestPosInsertionVal = insertionVal;
                            takerBestPos = newPos;
                        }
                    }
                    if (bestCandidate == null || takerBestPosInsertionVal > bestCandidateInsertionVal) {
                        bestCandidate = taker;
                        bestCandidateInsertionVal = takerBestPosInsertionVal;
                        bestCandidateInsertionPos = takerBestPos;
                    }
                }

                if (bestCandidate == null) {
                    System.out.println("Unable to find placement for id="+currId+", creating new trip for it.");
                    bestCandidate = new Trip();
                    bestCandidateInsertionPos = 0;
                    trips.add(bestCandidate);
                }
                bestCandidate.addStop(bestCandidateInsertionPos, currId); // No need to update meters yet
            }
            double valAfterDetachment = calcScore(trips);
            double diff = valBeforeDetachment - valAfterDetachment;
            System.out.println(formatAnsValue(valAfterDetachment) + " Detached " + detachedIds.size() + " destinations (" + Math.round(100.0*detachedIds.size()/(endId-2)) + "%) (diff " + diff +")");
        }

        class DetachmentCandidate implements Comparable<DetachmentCandidate> {

            Trip trip;
            int index;
            double val;

            public DetachmentCandidate(Trip trip, int index, double val) {
                this.trip = trip;
                this.index = index;
                this.val = val;
            }

            @Override
            public int compareTo(DetachmentCandidate o) {
                if (this.val - o.val < 0) return -1;
                if (this.val - o.val > 0) return 1;
                return 0;
            }
        }

        void localWalkSingleDestinationTransfers() {
            long lastSaveTime = System.currentTimeMillis();
            while (true) {
                boolean alive = false;
                for (Trip taker : trips) {
                    while (utz(taker) < 0.99) {
                        // Try to steal routes from other trips
                        boolean takerImproved = false;
                        for (Trip giver : trips) {
                            if (taker == giver) continue;
                            for (int i=0; i<giver.ids.size(); i++) {
                                if (transferIndex(i, giver, taker)) {
                                    periodicals();
                                    takerImproved = true;
                                }
                            }
                        }
                        if (!takerImproved) break;
                        else alive = true;
                    }
                }

                if (!alive) {
                    //System.out.println(formatAnsValue(val) + " LocalWalkSingleDestinationTransfers is done");
                    return;
                }
            }
        }

        boolean transferIndex(int index, Trip giver, Trip taker) {
            int currId = giver.ids.get(index);
            if (taker.weightSum + w[currId] > MAX_TRIP_WEIGHT) return false;
            int prevId = (index>0 ? giver.ids.get(index-1) : 1); // trip starts and ends at 1
            int nextId = (index<giver.ids.size()-1 ? giver.ids.get(index+1) : 1);
            double removalVal = (dist[prevId][currId] + dist[currId][nextId]) - dist[prevId][nextId];

            int bestPos = -1;
            double bestPosInsertionVal = Integer.MIN_VALUE;
            for (int newPos=0; newPos<=taker.ids.size(); newPos++) {
                prevId = (newPos>0 ? taker.ids.get(newPos-1) : 1); // trip starts and ends at 1
                nextId = (newPos<taker.ids.size() ? taker.ids.get(newPos) : 1); // item which will be at newPos+1 after displacement
                double insertionVal = dist[prevId][nextId] - (dist[prevId][currId] + dist[currId][nextId]);
                if (insertionVal > bestPosInsertionVal) {
                    bestPosInsertionVal = insertionVal;
                    bestPos = newPos;
                }
            }
            if (removalVal + bestPosInsertionVal < 0) {
                return false;
            } else {
                giver.meters += removalVal;
                taker.meters += bestPosInsertionVal;
                taker.addStop(bestPos, currId);
                giver.removeIndex(index);
                //System.out.println("        Moving id="+currId+" for a gain of " + (bestPosInsertionVal+removalVal));
                return true;
            }
        }










        List<Integer> candidates;
        long weightRemaining;

        void foreverCreateRoutesFromScratch() {
            while (true) {
                createRouteFromScratch();
            }
        }

        void createRouteFromScratch() {
            System.out.print("Creating route from scratch... ");
            weightRemaining = sumOfAllWeights;

            // These will be filled
            trips = new ArrayList<>();
            candidates = new ArrayList<>();

            // Order candidates mainly based on distance from Santa
            double[] loneliness = getLoneliness();
            List<IDval> sortHelper = new ArrayList<>();
            for (int id=2; id<endId; id++) {
                // Experimenting with more advanced heuristic here
                double heuristicVal = dist[1][id];

                // Previously discovered challenging nodes should be given priority
                //if (challengingNodes.contains(id)) heuristicVal += 100000L; // was making results slightly worse
                //if (loneliness[id] > 1000000) heuristicVal += 100000000;  // was making results slightly worse

                sortHelper.add(new IDval(id, heuristicVal));
            }
            Collections.sort(sortHelper);
            for (IDval pair : sortHelper) {
                candidates.add(pair.id);
            }

            // Create trips
            while (!candidates.isEmpty()) {
                createTrip();
            }

            writeAnsToFile(trips);
        }

        double getSmallestPossibleClusterSparsity() {
            double min = 0;
            double max = 10000000.0;
            if (possibleToQuicklyFillClusterOfSize(300000)) max = 299999; // minor speedup
            else min = 300000;
            while (min+5 < max) {
                double mid = min+(max-min)/2;
                if (possibleToQuicklyFillClusterOfSize(mid)) {
                    max = mid;
                } else {
                    min = mid+5;
                }
            }
            return min+5;
        }

        // This method intentionally overfills (due to use case where this is used to set a lower bound)
        boolean possibleToQuicklyFillClusterOfSize(double maxSparsity) {

            currTrip = new Trip();

            // Lock down the furthest target
            int currId = candidates.get(candidates.size()-1);
            currTrip.addStop(currId);

            double goal = UTZ_CLUSTER_GOAL * MAX_TRIP_WEIGHT;
            while (true) {
                boolean change = false;
                for (int candidateId : candidates) {
                    if (currTrip.used[candidateId]) continue;

                    // Is candidate within maxSparsity?
                    double sparsity = Double.POSITIVE_INFINITY;
                    for (int id : currTrip.ids) {
                        sparsity = Math.min(sparsity, dist[id][candidateId]);
                    }
                    if (sparsity > maxSparsity) continue;

                    // Add candidate to cluster
                    currTrip.addStop(candidateId);
                    if (currTrip.weightSum >= goal) return true;
                    change = true;
                }
                if (!change) return false;
            }
        }

        Trip currTrip;
        List<Integer> indicesForCurrTrip;
        double targetDist;

        void createTrip() {
            // A trip consists of a cluster, and detours on the way to/from cluster.
            // We want this trip to have high utz with minimal cluster sparsity and minimal detours.
            // First, binary search a lower bound for cluster sparsity. Use 1.1 * that sparsity (to allow more room for random).
            // Try many different (greedy, slightly randomized) ways of creating a trip.
            // Out of these trip options, greedily choose the one with lowest meters.

            double sparsityMin = getSmallestPossibleClusterSparsity();

            // These will be filled
            currTrip = new Trip();
            indicesForCurrTrip = null;

            // Where to build cluster
            int clusterTarget = candidates.get(candidates.size()-1);
            targetDist = dist[1][clusterTarget];

            // these variables will hold the best trip we find
            Trip bestTrip = null;
            List<Integer> bestIndicesForRemoval = null;

            // Create several (greedy, slightly randomized) options for trip to target
            double sparsity = multiplierToSlightlyExpandMaxSparsity * sparsityMin;
            for (; bestTrip == null; sparsity *= 1.04) { // Usually only 1 iteration. In rare cases the initial sparsity is not enough (because we allowed overfill when calculating lower bound for sparsity)
                double detourModifier = 0.01;
                for (int tripOption=1; tripOption<=50; tripOption++) {

                    currTrip = new Trip();
                    indicesForCurrTrip = new ArrayList<>();

                    // Lock down the furthest target
                    currTrip.used[clusterTarget] = true;
                    currTrip.addStop(clusterTarget);
                    indicesForCurrTrip.add(candidates.size()-1);

                    // collect cluster (at least) up to UTZ_CLUSTER_GOAL and then zigzag to/from cluster to fill remaining capacity.
                    collectClusterAroundTarget(sparsity);

                    if (sparsity > 50*sparsityMin) {
                        // We need this to avoid infinite loop in 2 special cases:
                        // Case 1: last trip may sometimes have low UTZ because there simply isn't enough weight available
                        // Case 2: some late trip may end up impossible due to weight restrictions (e.g. pick up up to 0.97 UTZ, but there's only large items available after that)
                        localWalkImprovementsToTrip(currTrip);
                        bestTrip = currTrip;
                        bestIndicesForRemoval = indicesForCurrTrip;
                        break;
                    }

                    if (utz(currTrip) < UTZ_CLUSTER_GOAL) {
                        if (tripOption < 2) break; // to speedup
                        continue;
                    }

                    // Add detours on the way to/from cluster
                    if (true) {
                        // TODO make sure zigzag is always able to complete the route
                        // TODO detours with higher max detour, but with preference by detour length (similar to clustering)
                        // Find first/last entry in order to discover detours to/from cluster
                        localWalkImprovementsToTrip(currTrip);
                        collectDetours(currTrip.firstEntry(), currTrip.lastEntry(), sparsity, detourModifier);
                    }

                    // Is this best tripOption for current target?
                    if (utz(currTrip) >= UTZ_TRIP_GOAL) {
                        localWalkImprovementsToTrip(currTrip);
                        currTrip.updateMeters();
                        if (bestTrip == null || currTrip.meters < bestTrip.meters) {
                            // TODO when choosing best among many options, place some value on high utz as well?
                            bestTrip = currTrip;
                            bestIndicesForRemoval = indicesForCurrTrip;
                        }
                    } else {
                        // We didn't reach enough detours so next tripOption let's be willing to accept longer detours
                        detourModifier *= 1.1;
                    }

                }
            }

            // Add current trip to route.
            trips.add(bestTrip);
            weightRemaining -= bestTrip.weightSum;

            // Print statistics of this trip
            if (verbose) {
                System.out.println(
                        "Trip #" + trips.size() +
                                " overall " + Math.round(bestTrip.meters / 1000) + "km, " +
                                "target " + Math.round(targetDist / 1000) + "km, " +
                                "detours " + Math.round((bestTrip.meters - 2 * targetDist) / 1000) + "km, " +
                                bestTrip.ids.size() + " stops, " +
                                "utz " + utz(bestTrip) +
                                ", acceptableClusterSparsity " + Math.round(sparsity / 1000) + "km " +
                                "(min bound " + Math.round(sparsityMin / 1000) + "km) " +
                                "trip target " + clusterTarget
                );
            }

            // Remove selected from candidates
            Collections.sort(bestIndicesForRemoval, Collections.reverseOrder()); // Need to delete in reverse order
            for (int index : bestIndicesForRemoval) {
                candidates.remove(index);
            }
        }

        void collectClusterAroundTarget(double maxSparsity) {
            while (true) {
                int bestIndex = -1;
                double bestHeuristicVal = Double.POSITIVE_INFINITY;
                for (int candidateIndex = candidates.size() - 2; candidateIndex >= 0; candidateIndex--) {
                    int candidateId = candidates.get(candidateIndex);

                    // Is candidate already used within this trip?
                    if (currTrip.used[candidateId]) continue;

                    // Does candidate fit within weight bounds?
                    if (currTrip.weightSum + w[candidateId] > MAX_TRIP_WEIGHT) continue;

                    // Does candidate fit within sparsity bounds? Sparsity == Min of dists to any previous stop within trip
                    double sparsity = Double.POSITIVE_INFINITY;
                    for (int id : currTrip.ids) {
                        sparsity = Math.min(sparsity, dist[id][candidateId]);
                    }

                    if (sparsity > maxSparsity) continue;

                    // Calculate heuristic value for candidate
                    double heuristicVal = sparsity;

                    // Add random to heuristic in order to explore many different options for this trip as this function is called many times
                    if (currTrip.weightSum + w[candidateId] < UTZ_TRIP_GOAL * MAX_TRIP_WEIGHT) {
                        // Add random unless we are able to "complete" a trip with this stop
                        heuristicVal *= (1 + randomSize*rng.nextDouble());
                    }

                    // TODO more advanced heuristic which gives some consideration to WEIGHT
                    // as well as possibly other factors, such as problem-childness-from-previous rounds or number
                    // of close neighbors still available as candidates

                    if (sparsity <= maxSparsity && heuristicVal < bestHeuristicVal) {
                        bestHeuristicVal = heuristicVal;
                        bestIndex = candidateIndex;
                    }

                }
                if (bestIndex < 0) break; // Impossible to expand cluster further (due to weight/sparsity)

                // Add closest node to cluster
                indicesForCurrTrip.add(bestIndex);
                int candidateId = candidates.get(bestIndex);
                currTrip.addStop(candidateId);
            }
        }

        // Add detours on the way to/from cluster.
        // Iterate candidates in order of furthest to closest.
        // If a candidate fits on the trip and the detour isn't too much, then we include it.
        // We always choose greedily whether we want to add to the beginning or end of our trip.
        void collectDetours(int closest1, int closest2, double sparsity, double detourModifier) {
            for (int candidateIndex = candidates.size() - 2; candidateIndex >= 0; candidateIndex--) {
                int candidateId = candidates.get(candidateIndex);
                if (currTrip.used[candidateId]) continue;
                if (currTrip.weightSum + w[candidateId] > MAX_TRIP_WEIGHT) continue;

                double detourCost1 = dist[closest1][candidateId] + dist[candidateId][1] - dist[closest1][1];
                double detourCost2 = dist[closest2][candidateId] + dist[candidateId][1] - dist[closest2][1];
                double detourCost = Math.min(detourCost1, detourCost2);

                if (detourCost <= detourModifier * sparsity) {
                    if (detourCost1 < detourCost2) {
                        // Case: add to beginning ("on the way to cluster")
                        closest1 = candidateId;
                        currTrip.addStop(0, candidateId);
                        indicesForCurrTrip.add(candidateIndex);
                    }
                    else {
                        // Case: add to end ("on the way from cluster")
                        closest2 = candidateId;
                        currTrip.addStop(candidateId);
                        indicesForCurrTrip.add(candidateIndex);
                    }
                }

            }
        }

        void localWalkImprovementsToTrips(List<Trip> trips) {
            for (Trip trip : trips) {
                localWalkImprovementsToTrip(trip);
            }
        }

        // This method will local walk the order within a single trip to a local optima.
        void localWalkImprovementsToTrip(Trip trip) {
            if (trip.isEmpty()) return;
            int n = trip.ids.size();
            int curr = 0;
            for (int countWithoutUpdate=0; countWithoutUpdate<=n;) {
                int currId = trip.ids.get(curr);
                int prevId = (curr>0 ? trip.ids.get(curr-1) : 1); // trip starts and ends at 1
                int nextId = (curr<n-1 ? trip.ids.get(curr+1) : 1);
                double removalVal = (dist[prevId][currId] + dist[currId][nextId]) - dist[prevId][nextId];

                int bestPos=curr;
                double bestPosVal=0;
                for (int newPos=0; newPos<=n; newPos++) {
                    if (newPos == curr) continue; // displaces itself and causes bugs
                    if (newPos == curr+1) continue; // this would also displace itself and cause bugs
                    prevId = (newPos>0 ? trip.ids.get(newPos-1) : 1); // trip starts and ends at 1
                    nextId = (newPos<n ? trip.ids.get(newPos) : 1); // item which will be at newPos+1 after displacement
                    double insertionVal = dist[prevId][nextId] - (dist[prevId][currId] + dist[currId][nextId]);
                    double val = insertionVal + removalVal;
                    //System.out.println("Val to move currId="+currId+" from " + curr + " to " + newPos + " == " + val);
                    if (val > bestPosVal) {
                        bestPosVal = val;
                        bestPos = newPos;
                    }
                }
                if (bestPos == curr) {
                    countWithoutUpdate++;
                } else {
                    countWithoutUpdate = 0;
                    trip.ids.add(bestPos, currId);
                    if (bestPos > curr) trip.ids.remove(curr);
                    else trip.ids.remove(curr+1);
                    //System.out.println("Moving id="+currId+" from " + curr + " to " + bestPos);
                }

                curr = (bestPos+1) % n;
            }
        }











        int endId;
        double[][] c; // coordinate values for destinations. [id][0/1] where 0==latitude, 1==longitude
        int[] w;
        long sumOfAllWeights;
        double[][] dist;

        void readInput() throws FileNotFoundException {
            System.out.println("Reading input...");

            // Find max id
            endId = Integer.MIN_VALUE;
            Scanner scanner = new Scanner(new File("input.txt"));
            while (scanner.hasNext()) {
                String[] line = scanner.nextLine().split(";");
                int id = Integer.parseInt(line[0]);
                endId = max(endId, id+1);
            }
            scanner.close();

            // Read file to memory
            c = new double[endId][2];
            w = new int[endId];
            sumOfAllWeights = 0;
            scanner = new Scanner(new File("input.txt"));
            while (scanner.hasNext()) {
                String[] line = scanner.nextLine().split(";");
                int id = Integer.parseInt(line[0]);
                double c1 = Double.parseDouble(line[1]);
                double c2 = Double.parseDouble(line[2]);
                int weight = Integer.parseInt(line[3]);
                c[id][0] = c1;
                c[id][1] = c2;
                w[id] = weight;
                sumOfAllWeights += weight;
                //System.out.println(id + ";" + c1 + ";" + c2 + ";" + weight);
            }

            // Add santa's coordinates to input at id 1
            c[1][0] = SANTA_HOME_LATITUDE;
            c[1][1] = SANTA_HOME_LONGITUDE;

            //readChallengingNodes();

            scanner.close();
        }

        Set<Integer> challengingNodes = new HashSet<>();

        void readChallengingNodes() throws FileNotFoundException {
            System.out.println("Reading challenging nodes...");
            challengingNodes = new HashSet<>();
            Scanner scanner = new Scanner(new File("challenging_nodes.txt"));
            while (scanner.hasNext()) {
                String line = scanner.nextLine();
                String[] sep = line.split(" ");
                try {
                    int id = Integer.parseInt(sep[sep.length-1]);
                    int tripNum = Integer.parseInt(sep[1].substring(1));
                    if (tripNum < 500) challengingNodes.add(id);
                } catch (Exception ex) {
                    //System.out.println("    Skipping row " + line);
                }
            }
        }

        void loadAndComparePreviousSolutions() throws FileNotFoundException {
            int bestI = 0;
            double bestVal = Double.POSITIVE_INFINITY;
            for (int i=1 ;; i++) {
                String filePath = getFilePath("santamap" + i);
                if (!new File(filePath).exists()) break;
                Double val = loadPreviouslyFoundSolution(filePath);
                if (val != null && val < bestVal) {
                    bestVal = val;
                    bestI = i;
                }
            }
            System.out.println("Best solution i="+bestI+" val=" + formatAnsValue(bestVal));
        }

        String getFilePath(String fileName) {
            return "outputs" + File.separator + fileName + ".txt";
        }

        Double loadPreviouslyFoundSolution(String filePath) throws FileNotFoundException {
            System.out.print("Loading " + filePath + "... ");
            trips = new ArrayList<>();
            File f = new File(filePath);
            if (!f.exists()) throw new RuntimeException("File doesn't exist: " + filePath);
            Scanner scanner = new Scanner(f);
            while (scanner.hasNext()) {
                String[] line = scanner.nextLine().split(";");
                Trip trip = new Trip();
                try {
                    for (int i = 0; i < line.length; i++) {
                        String element = line[i].replace(" ", "");
                        int id = Integer.parseInt(element);
                        trip.addStop(id);
                    }
                    trips.add(trip);
                } catch (Exception ex) {
                    System.out.println("Exception: " + ex.toString());
                    return null;
                }
            }
            // These allow flexibility to later modifications
            trips.add(new Trip());
            trips.add(new Trip());
            trips.add(new Trip());
            if (!isSolutionValid(trips)) return null;
            double val = calcScore(trips);
            bestSavedVal = Math.min(bestSavedVal, val);
            System.out.println("Solution value " + formatAnsValue(val));
            return val;

        }
        void preCalcAllDistances() throws Exception {
            System.out.println("Calculating distances...");
            dist = new double[endId][endId];
            for (int id1=1; id1<endId; id1++) {
                for (int id2 = id1 + 1; id2 < endId; id2++) {
                    double distance = calcDistance(id1, id2);
                    dist[id1][id2] = distance;
                    dist[id2][id1] = distance;
                }
            }

        }

        double calcDistance(int id1, int id2) {
            // Adapted from https://www.movable-type.co.uk/scripts/latlong.html
            double R = 6378000; // assumed radius of Earth in meters
            double lat1 = c[id1][0];
            double lat2 = c[id2][0];
            double lon1 = c[id1][1];
            double lon2 = c[id2][1];
            double phi1 = Math.toRadians(lat1);
            double phi2 = Math.toRadians(lat2);
            double deltaPhi = Math.toRadians(lat2-lat1);
            double deltaLambda = Math.toRadians(lon2-lon1);
            double sinPhi = Math.sin(deltaPhi/2);
            double sinLambda = Math.sin(deltaLambda/2);
            double a =  sinPhi*sinPhi + Math.cos(phi1) * Math.cos(phi2) * sinLambda*sinLambda;
            double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
            return R * c;
        }

        double[] getLoneliness() {
            double[] loneliness = new double[endId];
            for (int id=2; id<endId; id++) {
                // Sort everyone according to dist to id
                List<IDval> neighbors = new ArrayList<>();
                for (int neighbor=2; neighbor<endId; neighbor++) {
                    if (neighbor==id) continue;
                    neighbors.add(new IDval(neighbor, dist[id][neighbor]));
                }
                Collections.sort(neighbors);

                // Loneliness of node == radius needed to cluster this node so that sum of weight >= MAX_TRIP_WEIGHT
                int i = 0;
                for (int sumOfWeight=0; sumOfWeight < MAX_TRIP_WEIGHT; i++) {
                    sumOfWeight += neighbors.get(i).val;
                }
                int furthestNeighborInCluster = neighbors.get(i).id;
                loneliness[id] = dist[id][furthestNeighborInCluster];
            }
            return loneliness;
        }

        double calcScore(List<Trip> trips) {
            double meters = 0;
            for (Trip trip : trips) {
                meters += trip.updateMeters();
            }
            return meters;
        }

        double utz(Trip trip) {
            return 1.0 * trip.weightSum / MAX_TRIP_WEIGHT;
        }



        void writeAnsToFile(List<Trip> trips) {
            if (trips.isEmpty()) throw new RuntimeException("Empty ans given in writeOutput call");
            double val = calcScore(trips);
            if (val+0.00001 >= bestSavedVal) {
                return;
            }
            bestSavedVal = val;

            // Choose name for output file
            int nextFreeId = 1;
            String fileNameStub = "santamap";
            String fileName;
            while (true) {
                fileName = "outputs" + File.separator + fileNameStub + nextFreeId + ".txt";
                if (new File(fileName).exists()) nextFreeId++;
                else break;
            }

            System.out.println("Writing solution of value " + formatAnsValue(val) + " to " + fileName);
            // Write solution to file
            try (Writer writer = new BufferedWriter(new OutputStreamWriter(
                    new FileOutputStream(fileName), "utf-8"))) {
                for (Trip trip : trips) {
                    if (trip.isEmpty()) continue;
                    writer.write(tripToString(trip) + "\n");
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        String tripToString(Trip trip) {
            String line = "";
            for (int id : trip.ids) {
                line += id +"; ";
            }
            line = line.substring(0, line.length()-2);
            return line;
        }

        String formatAnsValue(double val) {
            NumberFormat formatter = new DecimalFormat("#0.000");
            return formatter.format(val/1e9);
        }

        class Trip {

            List<Integer> ids;
            double meters;
            boolean[] used;
            long weightSum;

            public Trip() {
                ids = new ArrayList<>();
                meters = 0;
                used = new boolean[endId];
                weightSum = 0;
            }

            // Note: meters need to be updated separately.
            void addStop(int id) {
                addStop(ids.size(), id);
            }
            void addStop(int i, int id) {
                used[id] = true;
                weightSum += w[id];
                ids.add(i, id);
            }
            void removeIndex(int i) {
                int id = ids.remove(i);
                used[id] = false;
                weightSum -= w[id];
            }
            void removeId(int id) {
                for (int i=0; i<ids.size(); i++) {
                    if (ids.get(i) == id) {
                        removeIndex(i);
                        return;
                    }
                }
            }

            int firstEntry() {
                return ids.get(0);
            }
            int lastEntry() {
                return ids.get(ids.size()-1);
            }

            double updateMeters() {
                meters = 0;
                if (ids.isEmpty()) return 0;
                int prevId = 1;
                for (int id : ids) {
                    meters += dist[prevId][id];
                    prevId = id;
                }
                meters += dist[prevId][1];
                return meters;
            }

            boolean isEmpty() {
                return ids.isEmpty();
            }

            @Override
            public boolean equals(Object o) {
                return this == o;
            }

            @Override
            public int hashCode() {
                return (int) (ids.size() + meters + weightSum*79);
            }
        }

        void periodicals() {
            periodicallyReportScore();
            periodicallySave();
        }

        void periodicallyReportScore() {
            long time = System.currentTimeMillis();
            if (time > lastScorePrintTime + printIntervalSeconds * 1000) {
                lastScorePrintTime = time;
                printScore();
            }
        }

        void periodicallySave() {
            long time = System.currentTimeMillis();
            if (time > lastScoreSaveTime + saveIntervalSeconds * 1000) {
                lastScoreSaveTime = time;
                writeAnsToFile(trips);
            }
        }

        void printScore() {
            assertSolutionValid();
            double curr = calcScore(trips);
            double prev = lastPrintedScore;
            double diff = prev-curr;
            String s = (diff > 0 ? "+" : ""); // indicate change by + or -, no symbol means no change
            lastPrintedScore = curr;
            lowestKnownScore = Math.min(curr, lowestKnownScore);
            int timeDiff = (int) (System.currentTimeMillis() - startTime) / 1000;
            int expendedSeconds = timeDiff % 60;
            int expendedMinutes = (timeDiff/60) % 60;
            int expendedHours = (timeDiff/60/60);
            String c = formatAnsValue(curr);
            String d = formatAnsValue(diff);
            String b = formatAnsValue(lowestKnownScore);
            String t = expendedSeconds+"";
            if (expendedSeconds < 10) t = "0"+t;
            t = expendedMinutes + ":" + t;
            if (expendedMinutes < 10) t = "0"+t;
            if (expendedHours > 0) {
                t = expendedHours + ":" + t;
                if (expendedHours < 10) t = "0"+t;
            }
            System.out.println(c + " (" + s + d + " diff) (" + b + " best) (" + t + " elapsed)");
        }

        void assertSolutionValid() {
            if (!isSolutionValid(trips)) throw new RuntimeException("Solution invalid!");
        }

        boolean isSolutionValid(List<Trip> trips) {
            HashSet<Integer> validityCheck = new HashSet<>();
            for (Trip trip : trips) {
                for (int id : trip.ids) {
                    if (!validityCheck.add(id)) {
                        System.out.println("Invalid solution! Id " + id + " appears twice!");
                        return false;
                    }
                }
                if (trip.weightSum > MAX_TRIP_WEIGHT) {
                    System.out.println("Invalid solution! Sleigh can not carry " + trip.weightSum);
                    return false;
                }
            }
            if (validityCheck.size() != endId-2) {
                System.out.println("Invalid solution! Expected " + (endId-2) + " stops, but actual was " + validityCheck.size());
                return false;
            }
            return true;
        }

        /************************** UTILITY CODE BELOW THIS LINE **************************/

        long MOD = (long)1e9 + 7;

        boolean closeToZero(double v) {
            // Check if double is close to zero, considering precision issues.
            return Math.abs(v) <= 0.0000000001;
        }

        class DrawGrid {

            void draw(boolean[][] d) {
                System.out.print("  ");
                for (int x=0; x<d[0].length; x++) {
                    System.out.print(" " + x + " ");
                }
                System.out.println("");
                for (int y=0; y<d.length; y++) {
                    System.out.print(y + " ");
                    for (int x=0; x<d[0].length; x++) {
                        System.out.print((d[y][x] ? "[x]" : "[ ]"));
                    }
                    System.out.println("");
                }
            }

            void draw(int[][] d) {
                int max = 1;
                for (int y=0; y<d.length; y++) {
                    for (int x=0; x<d[0].length; x++) {
                        max = Math.max(max, ("" + d[y][x]).length());
                    }
                }
                System.out.print("  ");
                String format = "%" + (max+2) + "s";
                for (int x=0; x<d[0].length; x++) {
                    System.out.print(String.format(format, x) + " ");
                }
                format = "%" + (max) + "s";
                System.out.println("");
                for (int y=0; y<d.length; y++) {
                    System.out.print(y + " ");
                    for (int x=0; x<d[0].length; x++) {
                        System.out.print(" [" + String.format(format, (d[y][x])) + "]");
                    }
                    System.out.println("");
                }
            }

        }

        class IDval implements Comparable<IDval> {
            int id;
            double val;

            public IDval(int id, double val) {
                this.val = val;
                this.id = id;
            }

            @Override
            public int compareTo(IDval o) {
                if (this.val < o.val) return -1;
                if (this.val > o.val) return 1;
                return this.id - o.id;
            }
        }

        private class ElementCounter {
            private HashMap<Long, Integer> elements;

            public ElementCounter() {
                elements = new HashMap<>();
            }

            public void add(long element) {
                int count = 1;
                Integer prev = elements.get(element);
                if (prev != null) count += prev;
                elements.put(element, count);
            }

            public void remove(long element) {
                int count = elements.remove(element);
                count--;
                if (count > 0) elements.put(element, count);
            }

            public int get(long element) {
                Integer val = elements.get(element);
                if (val == null) return 0;
                return val;
            }

            public int size() {
                return elements.size();
            }
        }

        class StringCounter {
            HashMap<String, Integer> elements;

            public StringCounter() {
                elements = new HashMap<>();
            }

            public void add(String identifier) {
                int count = 1;
                Integer prev = elements.get(identifier);
                if (prev != null) count += prev;
                elements.put(identifier, count);
            }

            public void remove(String identifier) {
                int count = elements.remove(identifier);
                count--;
                if (count > 0) elements.put(identifier, count);
            }

            public long get(String identifier) {
                Integer val = elements.get(identifier);
                if (val == null) return 0;
                return val;
            }

            public int size() {
                return elements.size();
            }
        }

        class DisjointSet {
            /** Union Find / Disjoint Set data structure. */
            int[] size;
            int[] parent;
            int componentCount;

            public DisjointSet(int n) {
                componentCount = n;
                size = new int[n];
                parent = new int[n];
                for (int i=0; i<n; i++) parent[i] = i;
                for (int i=0; i<n; i++) size[i] = 1;
            }

            public void join(int a, int b) {
                /* Find roots */
                int rootA = parent[a];
                int rootB = parent[b];
                while (rootA != parent[rootA]) rootA = parent[rootA];
                while (rootB != parent[rootB]) rootB = parent[rootB];

                if (rootA == rootB) {
                    /* Already in the same set */
                    return;
                }

                /* Merge smaller set into larger set. */
                if (size[rootA] > size[rootB]) {
                    size[rootA] += size[rootB];
                    parent[rootB] = rootA;
                } else {
                    size[rootB] += size[rootA];
                    parent[rootA] = rootB;
                }
                componentCount--;
            }

        }

        class Trie {
            int N;
            int Z;
            int nextFreeId;
            int[][] pointers;
            boolean[] end;

            /** maxLenSum = maximum possible sum of length of words */
            public Trie(int maxLenSum, int alphabetSize) {
                this.N = maxLenSum;
                this.Z = alphabetSize;
                this.nextFreeId = 1;
                pointers = new int[N+1][alphabetSize];
                end = new boolean[N+1];
            }

            public void addWord(String word) {
                int curr = 0;
                for (int j=0; j<word.length(); j++) {
                    int c = word.charAt(j) - 'a';
                    int next = pointers[curr][c];
                    if (next == 0) {
                        next = nextFreeId++;
                        pointers[curr][c] = next;
                    }
                    curr = next;
                }
                end[curr] = true;
            }

            public boolean hasWord(String word) {
                int curr = 0;
                for (int j=0; j<word.length(); j++) {
                    int c = word.charAt(j) - 'a';
                    int next = pointers[curr][c];
                    if (next == 0) return false;
                    curr = next;
                }
                return end[curr];
            }

        }

        private static class Prob {

            /** For heavy calculations on probabilities, this class
             *  provides more accuracy & efficiency than doubles.
             *  Math explained: https://en.wikipedia.org/wiki/Log_probability
             *  Quick start:
             *      - Instantiate probabilities, eg. Prob a = new Prob(0.75)
             *      - add(), multiply() return new objects, can perform on nulls & NaNs.
             *      - get() returns probability as a readable double */

            /** Logarithmized probability. Note: 0% represented by logP NaN. */
            private double logP;

            /** Construct instance with real probability. */
            public Prob(double real) {
                if (real > 0) this.logP = Math.log(real);
                else this.logP = Double.NaN;
            }

            /** Construct instance with already logarithmized value. */
            static boolean dontLogAgain = true;
            public Prob(double logP, boolean anyBooleanHereToChooseThisConstructor) {
                this.logP = logP;
            }

            /** Returns real probability as a double. */
            public double get() {
                return Math.exp(logP);
            }

            @Override
            public String toString() {
                return ""+get();
            }

            /***************** STATIC METHODS BELOW ********************/

            /** Note: returns NaN only when a && b are both NaN/null. */
            public static Prob add(Prob a, Prob b) {
                if (nullOrNaN(a) && nullOrNaN(b)) return new Prob(Double.NaN, dontLogAgain);
                if (nullOrNaN(a)) return copy(b);
                if (nullOrNaN(b)) return copy(a);

                double x = Math.max(a.logP, b.logP);
                double y = Math.min(a.logP, b.logP);
                double sum = x + Math.log(1 + Math.exp(y - x));
                return new Prob(sum, dontLogAgain);
            }

            /** Note: multiplying by null or NaN produces NaN (repping 0% real prob). */
            public static Prob multiply(Prob a, Prob b) {
                if (nullOrNaN(a) || nullOrNaN(b)) return new Prob(Double.NaN, dontLogAgain);
                return new Prob(a.logP + b.logP, dontLogAgain);
            }

            /** Returns true if p is null or NaN. */
            private static boolean nullOrNaN(Prob p) {
                return (p == null || Double.isNaN(p.logP));
            }

            /** Returns a new instance with the same value as original. */
            private static Prob copy(Prob original) {
                return new Prob(original.logP, dontLogAgain);
            }
        }

        class Binary implements Comparable<Binary> {

            /**
             * Use example: Binary b = new Binary(Long.toBinaryString(53249834L));
             *
             * When manipulating small binary strings, instantiate new Binary(string)
             * When just reading large binary strings, instantiate new Binary(string,true)
             * get(int i) returns a character '1' or '0', not an int.
             */

            private boolean[] d;
            private int first; // Starting from left, the first (most remarkable) '1'
            public int length;


            public Binary(String binaryString) {
                this(binaryString, false);
            }
            public Binary(String binaryString, boolean initWithMinArraySize) {
                length = binaryString.length();
                int size = Math.max(2*length, 1);
                first = length/4;
                if (initWithMinArraySize) {
                    first = 0;
                    size = Math.max(length, 1);
                }
                d = new boolean[size];
                for (int i=0; i<length; i++) {
                    if (binaryString.charAt(i) == '1') d[i+first] = true;
                }
            }

            public void addFirst(char c) {
                if (first-1 < 0) doubleArraySize();
                first--;
                d[first] = (c == '1' ? true : false);
                length++;
            }

            public void addLast(char c) {
                if (first+length >= d.length) doubleArraySize();
                d[first+length] = (c == '1' ? true : false);
                length++;
            }

            private void doubleArraySize() {
                boolean[] bigArray = new boolean[(d.length+1) * 2];
                int newFirst = bigArray.length / 4;
                for (int i=0; i<length; i++) {
                    bigArray[i + newFirst] = d[i + first];
                }
                first = newFirst;
                d = bigArray;
            }

            public boolean flip(int i) {
                boolean value = (this.d[first+i] ? false : true);
                this.d[first+i] = value;
                return value;
            }

            public void set(int i, char c) {
                boolean value = (c == '1' ? true : false);
                this.d[first+i] = value;
            }

            public char get(int i) {
                return (this.d[first+i] ? '1' : '0');
            }

            @Override
            public int compareTo(Binary o) {
                if (this.length != o.length) return this.length - o.length;
                int len = this.length;
                for (int i=0; i<len; i++) {
                    int diff = this.get(i) - o.get(i);
                    if (diff != 0) return diff;
                }
                return 0;
            }

            @Override
            public String toString() {
                StringBuilder sb = new StringBuilder();
                for (int i=0; i<length; i++) {
                    sb.append(d[i+first] ? '1' : '0');
                }
                return sb.toString();
            }


        }

        /************************** Range queries **************************/

        class FenwickMin {
            long n;
            long[] original;
            long[] bottomUp;
            long[] topDown;

            public FenwickMin(int n) {
                this.n = n;
                original = new long[n+2];
                bottomUp = new long[n+2];
                topDown = new long[n+2];
            }

            public void set(int modifiedNode, long value) {
                long replaced = original[modifiedNode];
                original[modifiedNode] = value;
                // Update left tree
                int i = modifiedNode;
                long v = value;
                while (i <= n) {
                    if (v > bottomUp[i]) {
                        if (replaced == bottomUp[i]) {
                            v = Math.min(v, original[i]);
                            for (int r=1 ;; r++) {
                                int x = (i&-i)>>>r;
                                if (x == 0) break;
                                int child = i-x;
                                v = Math.min(v, bottomUp[child]);
                            }
                        } else break;
                    }
                    if (v == bottomUp[i]) break;
                    bottomUp[i] = v;
                    i += (i&-i);
                }
                // Update right tree
                i = modifiedNode;
                v = value;
                while (i > 0) {
                    if (v > topDown[i]) {
                        if (replaced == topDown[i]) {
                            v = Math.min(v, original[i]);
                            for (int r=1 ;; r++) {
                                int x = (i&-i)>>>r;
                                if (x == 0) break;
                                int child = i+x;
                                if (child > n+1) break;
                                v = Math.min(v, topDown[child]);
                            }
                        } else break;
                    }
                    if (v == topDown[i]) break;
                    topDown[i] = v;
                    i -= (i&-i);
                }
            }

            public long getMin(int a, int b) {
                long min = original[a];
                int prev = a;
                int curr = prev + (prev&-prev); // parent right hand side
                while (curr <= b) {
                    min = Math.min(min, topDown[prev]); // value from the other tree
                    prev = curr;
                    curr = prev + (prev&-prev);;
                }
                min = Math.min(min, original[prev]);
                prev = b;
                curr = prev - (prev&-prev); // parent left hand side
                while (curr >= a) {
                    min = Math.min(min,bottomUp[prev]); // value from the other tree
                    prev = curr;
                    curr = prev - (prev&-prev);
                }
                return min;
            }

        }

        class FenwickSum {
            public long[] d;

            public FenwickSum(int n) {
                d=new long[n+1];
            }

            /** a[0] must be unused. */
            public FenwickSum(long[] a) {
                d=new long[a.length];
                for (int i=1; i<a.length; i++) {
                    modify(i, a[i]);
                }
            }

            /** Do not modify i=0. */
            void modify(int i, long v) {
                while (i<d.length) {
                    d[i] += v;
                    // Move to next uplink on the RIGHT side of i
                    i += (i&-i);
                }
            }

            /** Returns sum from a to b, *BOTH* inclusive. */
            long getSum(int a, int b) {
                return getSum(b) - getSum(a-1);
            }

            private long getSum(int i) {
                long sum = 0;
                while (i>0) {
                    sum += d[i];
                    // Move to next uplink on the LEFT side of i
                    i -= (i&-i);
                }
                return sum;
            }
        }

        class SegmentTree {

            /* Provides log(n) operations for:
             * - Range query (sum, min or max)
             * - Range update ("+8 to all values between indexes 4 and 94")
             */

            int N;
            long[] lazy;
            long[] sum;
            long[] min;
            long[] max;
            boolean supportSum;
            boolean supportMin;
            boolean supportMax;

            public SegmentTree(int n) {
                this(n, true, true, true);
            }

            public SegmentTree(int n, boolean supportSum, boolean supportMin, boolean supportMax) {
                for (N=2; N<n;) N*=2;
                this.lazy = new long[2*N];
                this.supportSum = supportSum;
                this.supportMin = supportMin;
                this.supportMax = supportMax;
                if (this.supportSum) this.sum = new long[2*N];
                if (this.supportMin) this.min = new long[2*N];
                if (this.supportMax) this.max = new long[2*N];
            }

            void modifyRange(long x, int a, int b) {
                modifyRec(a, b, 1, 0, N-1, x);
            }

            void setRange() {
                //TODO
            }

            long getSum(int a, int b) {
                return querySum(a, b, 1, 0, N-1);
            }

            long getMin(int a, int b) {
                return queryMin(a, b, 1, 0, N-1);
            }

            long getMax(int a, int b) {
                return queryMax(a, b, 1, 0, N-1);
            }

            private long querySum(int wantedLeft, int wantedRight, int i, int actualLeft, int actualRight) {
                if (wantedLeft > actualRight || wantedRight < actualLeft) {
                    return 0;
                }
                if (wantedLeft == actualLeft && wantedRight == actualRight) {
                    int count = wantedRight - wantedLeft + 1;
                    return sum[i] + count * lazy[i];
                }
                if (lazy[i] != 0) propagate(i, actualLeft, actualRight);
                int d = (actualRight - actualLeft + 1) / 2;
                long left = querySum(wantedLeft, min(actualLeft+d-1, wantedRight), 2*i, actualLeft, actualLeft+d-1);
                long right = querySum(max(actualLeft+d, wantedLeft), wantedRight, 2*i+1, actualLeft+d, actualRight);
                return left + right;
            }

            private long queryMin(int wantedLeft, int wantedRight, int i, int actualLeft, int actualRight) {
                if (wantedLeft > actualRight || wantedRight < actualLeft) {
                    return Long.MAX_VALUE;
                }
                if (wantedLeft == actualLeft && wantedRight == actualRight) {
                    return min[i] + lazy[i];
                }
                if (lazy[i] != 0) propagate(i, actualLeft, actualRight);
                int d = (actualRight - actualLeft + 1) / 2;
                long left = queryMin(wantedLeft, min(actualLeft+d-1, wantedRight), 2*i, actualLeft, actualLeft+d-1);
                long right = queryMin(max(actualLeft+d, wantedLeft), wantedRight, 2*i+1, actualLeft+d, actualRight);
                return min(left, right);
            }

            private long queryMax(int wantedLeft, int wantedRight, int i, int actualLeft, int actualRight) {
                if (wantedLeft > actualRight || wantedRight < actualLeft) {
                    return Long.MIN_VALUE;
                }
                if (wantedLeft == actualLeft && wantedRight == actualRight) {
                    return max[i] + lazy[i];
                }
                if (lazy[i] != 0) propagate(i, actualLeft, actualRight);
                int d = (actualRight - actualLeft + 1) / 2;
                long left = queryMax(wantedLeft, min(actualLeft+d-1, wantedRight), 2*i, actualLeft, actualLeft+d-1);
                long right = queryMax(max(actualLeft+d, wantedLeft), wantedRight, 2*i+1, actualLeft+d, actualRight);
                return max(left, right);
            }

            private void modifyRec(int wantedLeft, int wantedRight, int i, int actualLeft, int actualRight, long value) {
                if (wantedLeft > actualRight || wantedRight < actualLeft) {
                    return;
                }
                if (wantedLeft == actualLeft && wantedRight == actualRight) {
                    lazy[i] += value;
                    return;
                }
                if (lazy[i] != 0) propagate(i, actualLeft, actualRight);
                int d = (actualRight - actualLeft + 1) / 2;
                modifyRec(wantedLeft, min(actualLeft+d-1, wantedRight), 2*i, actualLeft, actualLeft+d-1, value);
                modifyRec(max(actualLeft+d, wantedLeft), wantedRight, 2*i+1, actualLeft+d, actualRight, value);
                if (supportSum) sum[i] += value * (min(actualRight, wantedRight) - max(actualLeft, wantedLeft) + 1);
                if (supportMin) min[i] = min(min[2*i] + lazy[2*i], min[2*i+1] + lazy[2*i+1]);
                if (supportMax) max[i] = max(max[2*i] + lazy[2*i], max[2*i+1] + lazy[2*i+1]);
            }

            private void propagate(int i, int actualLeft, int actualRight) {
                lazy[2*i] += lazy[i];
                lazy[2*i+1] += lazy[i];
                if (supportSum) sum[i] += lazy[i] * (actualRight - actualLeft + 1);
                if (supportMin) min[i] += lazy[i];
                if (supportMax) max[i] += lazy[i];
                lazy[i] = 0;
            }

        }

        /***************************** Graphs *****************************/

        List<Integer>[] toGraph(IO io, int n) {
            /* Trees only. */
            List<Integer>[] g = new ArrayList[n+1];
            for (int i=1; i<=n; i++) g[i] = new ArrayList<>();
            for (int i=1; i<=n-1; i++) {
                int a = io.nextInt();
                int b = io.nextInt();
                g[a].add(b);
                g[b].add(a);
            }
            return g;
        }

        class Graph {

            int n;
            List<Integer>[] edges;

            public Graph(int n) {
                this.n = n;
                edges = new ArrayList[n+1];
                for (int i=1; i<=n; i++) edges[i] = new ArrayList<>();
            }

            void addBiEdge(int a, int b) {
                addEdge(a, b);
                addEdge(b, a);
            }

            void addEdge(int from, int to) {
                edges[from].add(to);
            }

            /*********** Strongly Connected Components (Kosaraju) ****************/

            ArrayList<Integer>[] bacw;

            public int getCount() {
                bacw = new ArrayList[n+1];
                for (int i=1; i<=n; i++) {
                    bacw[i] = new ArrayList<Integer>();
                }
                for (int a=1; a<=n; a++) {
                    for (int b : edges[a]) {
                        bacw[b].add(a);
                    }
                }
                int count = 0;
                List<Integer> list = new ArrayList<Integer>();
                boolean[] visited = new boolean[n+1];
                for (int i=1; i<=n; i++) {
                    dfsForward(i, visited, list);
                }
                visited = new boolean[n+1];
                for (int i=n-1; i>=0; i--) {
                    int node = list.get(i);
                    if (visited[node]) continue;
                    count++;
                    dfsBackward(node, visited);
                }
                return count;
            }

            void dfsForward(int i, boolean[] visited, List<Integer> list) {
                if (visited[i]) return;
                visited[i] = true;
                for (int neighbor : edges[i]) {
                    dfsForward(neighbor, visited, list);
                }
                list.add(i);
            }

            void dfsBackward(int i, boolean[] visited) {
                if (visited[i]) return;
                visited[i] = true;
                for (int neighbor : bacw[i]) {
                    dfsBackward(neighbor, visited);
                }
            }

            /************************* Topological Order **********************/

            int UNTOUCHED = 0;
            int FINISHED = 2;
            int INPROGRESS = 1;
            int[] vis;
            List<Integer> topoAns;

            // Returns nodes in topological order or null if cycle was found
            public List<Integer> topoSort() {
                topoAns = new ArrayList<>();
                vis = new int[n+1];
                for (int i=1; i<=n; i++) {
                    if (!topoDFS(i)) return null;
                }
                Collections.reverse(topoAns);
                return topoAns;
            }

            boolean topoDFS(int curr) {
                Integer status = vis[curr];
                if (status == null) status = UNTOUCHED;
                if (status == FINISHED) return true;
                if (status == INPROGRESS) {
                    return false;
                }
                vis[curr] = INPROGRESS;
                for (int next : edges[curr]) {
                    if (!topoDFS(next)) return false;
                }
                vis[curr] = FINISHED;
                topoAns.add(curr);
                return true;
            }

        }

        class LCAFinder {

            /* O(n log n) Initialize: new LCAFinder(graph)
             * O(log n) Queries: find(a,b) returns lowest common ancestor for nodes a and b */

            int[] nodes;
            int[] depths;
            int[] entries;
            int pointer;
            FenwickMin fenwick;

            public LCAFinder(List<Integer>[] graph) {
                this.nodes = new int[(int)10e6];
                this.depths = new int[(int)10e6];
                this.entries = new int[graph.length];
                this.pointer = 1;
                boolean[] visited = new boolean[graph.length+1];
                dfs(1, 0, graph, visited);
                fenwick = new FenwickMin(pointer-1);
                for (int i=1; i<pointer; i++) {
                    fenwick.set(i, depths[i] * 1000000L + i);
                }
            }

            private void dfs(int node, int depth, List<Integer>[] graph, boolean[] visited) {
                visited[node] = true;
                entries[node] = pointer;
                nodes[pointer] = node;
                depths[pointer] = depth;
                pointer++;
                for (int neighbor : graph[node]) {
                    if (visited[neighbor]) continue;
                    dfs(neighbor, depth+1, graph, visited);
                    nodes[pointer] = node;
                    depths[pointer] = depth;
                    pointer++;
                }
            }

            public int find(int a, int b) {
                int left = entries[a];
                int right = entries[b];
                if (left > right) {
                    int temp = left;
                    left = right;
                    right = temp;
                }
                long mixedBag = fenwick.getMin(left, right);
                int index = (int) (mixedBag % 1000000L);
                return nodes[index];
            }
        }

        /**************************** Geometry ****************************/

        class Point {
            int y;
            int x;

            public Point(int y, int x) {
                this.y = y;
                this.x = x;
            }
        }

        boolean segmentsIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
            // Returns true if segment 1-2 intersects segment 3-4

            if (x1 == x2 && x3 == x4) {
                // Both segments are vertical
                if (x1 != x3) return false;
                if (min(y1,y2) < min(y3,y4)) {
                    return max(y1,y2) >= min(y3,y4);
                } else {
                    return max(y3,y4) >= min(y1,y2);
                }
            }
            if (x1 == x2) {
                // Only segment 1-2 is vertical. Does segment 3-4 cross it? y = a*x + b
                double a34 = (y4-y3)/(x4-x3);
                double b34 = y3 - a34*x3;
                double y = a34 * x1 + b34;
                return y >= min(y1,y2) && y <= max(y1,y2) && x1 >= min(x3,x4) && x1 <= max(x3,x4);
            }
            if (x3 == x4) {
                // Only segment 3-4 is vertical. Does segment 1-2 cross it? y = a*x + b
                double a12 = (y2-y1)/(x2-x1);
                double b12 = y1 - a12*x1;
                double y = a12 * x3 + b12;
                return y >= min(y3,y4) && y <= max(y3,y4) && x3 >= min(x1,x2) && x3 <= max(x1,x2);
            }
            double a12 = (y2-y1)/(x2-x1);
            double b12 = y1 - a12*x1;
            double a34 = (y4-y3)/(x4-x3);
            double b34 = y3 - a34*x3;
            if (closeToZero(a12 - a34)) {
                // Parallel lines
                return closeToZero(b12 - b34);
            }
            // Non parallel non vertical lines intersect at x. Is x part of both segments?
            double x = -(b12-b34)/(a12-a34);
            return x >= min(x1,x2) && x <= max(x1,x2) && x >= min(x3,x4) && x <= max(x3,x4);
        }

        boolean pointInsideRectangle(Point p, List<Point> r, boolean countBorderAsInside) {
            Point a = r.get(0);
            Point b = r.get(1);
            Point c = r.get(2);
            Point d = r.get(3);
            double apd = areaOfTriangle(a, p, d);
            double dpc = areaOfTriangle(d, p, c);
            double cpb = areaOfTriangle(c, p, b);
            double pba = areaOfTriangle(p, b, a);
            double sumOfAreas = apd + dpc + cpb + pba;
            if (closeToZero(sumOfAreas - areaOfRectangle(r))) {
                if (closeToZero(apd) || closeToZero(dpc) || closeToZero(cpb) || closeToZero(pba)) {
                    return countBorderAsInside;
                }
                return true;
            }
            return false;
        }

        double areaOfTriangle(Point a, Point b, Point c) {
            return 0.5 * Math.abs((a.x-c.x)*(b.y-a.y)-(a.x-b.x)*(c.y-a.y));
        }

        double areaOfRectangle(List<Point> r) {
            double side1xDiff = r.get(0).x - r.get(1).x;
            double side1yDiff = r.get(0).y - r.get(1).y;
            double side2xDiff = r.get(1).x - r.get(2).x;
            double side2yDiff = r.get(1).y - r.get(2).y;
            double side1 = Math.sqrt(side1xDiff * side1xDiff + side1yDiff * side1yDiff);
            double side2 = Math.sqrt(side2xDiff * side2xDiff + side2yDiff * side2yDiff);
            return side1 * side2;
        }

        boolean pointsOnSameLine(double x1, double y1, double x2, double y2, double x3, double y3) {
            double areaTimes2 = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
            return (closeToZero(areaTimes2));
        }

        class PointToLineSegmentDistanceCalculator {

            // Just call this
            double minDistFromPointToLineSegment(double point_x, double point_y, double x1, double y1, double x2, double y2) {
                return Math.sqrt(distToSegmentSquared(point_x, point_y, x1, y1, x2, y2));
            }

            private double distToSegmentSquared(double point_x, double point_y, double x1, double y1, double x2, double y2) {
                double l2 = dist2(x1,y1,x2,y2);
                if (l2 == 0) return dist2(point_x, point_y, x1, y1);
                double t = ((point_x - x1) * (x2 - x1) + (point_y - y1) * (y2 - y1)) / l2;
                if (t < 0) return dist2(point_x, point_y, x1, y1);
                if (t > 1) return dist2(point_x, point_y, x2, y2);

                double com_x = x1 + t * (x2 - x1);
                double com_y = y1 + t * (y2 - y1);
                return dist2(point_x, point_y, com_x, com_y);
            }

            private double dist2(double x1, double y1, double x2, double y2) {
                return Math.pow((x1 - x2), 2) + Math.pow((y1 - y2), 2);
            }

        }

        /****************************** Math ******************************/

        long pow(long base, int exp) {
            if (exp == 0) return 1L;
            long x = pow(base, exp/2);
            long ans = x * x;
            if (exp % 2 != 0) ans *= base;
            return ans;
        }

        long gcd(long... v) {
            /** Chained calls to Euclidean algorithm. */
            if (v.length == 1) return v[0];
            long ans = gcd(v[1], v[0]);
            for (int i=2; i<v.length; i++) {
                ans = gcd(ans, v[i]);
            }
            return ans;
        }

        long gcd(long a, long b) {
            /** Euclidean algorithm. */
            if (b == 0) return a;
            return gcd(b, a%b);
        }

        int[] generatePrimesUpTo(int last) {
            /* Sieve of Eratosthenes. Practically O(n). Values of 0 indicate primes. */
            int[] div = new int[last+1];
            for (int x=2; x<=last; x++) {
                if (div[x] > 0) continue;
                for (int u=2*x; u<=last; u+=x) {
                    div[u] = x;
                }
            }
            return div;
        }

        long lcm(long a, long b) {
            /** Least common multiple */
            return a * b / gcd(a,b);
        }

        class BaseConverter {

            /* Palauttaa luvun esityksen kannassa base */
            public String convert(Long number, int base) {
                return Long.toString(number, base);
            }

            /* Palauttaa luvun esityksen kannassa baseTo, kun annetaan luku Stringinä kannassa baseFrom */
            public String convert(String number, int baseFrom, int baseTo) {
                return Long.toString(Long.parseLong(number, baseFrom), baseTo);
            }

            /* Tulkitsee kannassa base esitetyn luvun longiksi (kannassa 10) */
            public long longify(String number, int baseFrom) {
                return Long.parseLong(number, baseFrom);
            }
        }

        class BinomialCoefficients {
            /** Total number of K sized unique combinations from pool of size N (unordered)
             N! / ( K! (N - K)! )   */

            /** For simple queries where output fits in long. */
            public long biCo(long n, long k) {
                long r = 1;
                if (k > n) return 0;
                for (long d = 1; d <= k; d++) {
                    r *= n--;
                    r /= d;
                }
                return r;
            }

            /** For multiple queries with same n, different k. */
            public long[] precalcBinomialCoefficientsK(int n, int maxK) {
                long v[] = new long[maxK+1];
                v[0] = 1; // nC0 == 1
                for (int i=1; i<=n; i++) {
                    for (int j=Math.min(i,maxK); j>0; j--) {
                        v[j] = v[j] + v[j-1]; // Pascal's triangle
                    }
                }
                return v;
            }

            /** When output needs % MOD. */
            public long[] precalcBinomialCoefficientsK(int n, int k, long M) {
                long v[] = new long[k+1];
                v[0] = 1; // nC0 == 1
                for (int i=1; i<=n; i++) {
                    for (int j=Math.min(i,k); j>0; j--) {
                        v[j] = v[j] + v[j-1]; // Pascal's triangle
                        v[j] %= M;
                    }
                }
                return v;
            }
        }

        int invertNumber(int a, int k) {
            // Inverts the binary representation of a, using only k last bits e.g. 01101 -> 10010
            int inv32k = ~a;
            int mask = 1;
            for (int i = 1; i < k; ++i) mask |= mask << 1;
            return inv32k & mask;
        }

        /**************************** Strings ****************************/

        class Zalgo {

            public int pisinEsiintyma(String haku, String kohde) {
                char[] s = new char[haku.length() + 1 + kohde.length()];
                for (int i=0; i<haku.length(); i++) {
                    s[i] = haku.charAt(i);
                }
                int j = haku.length();
                s[j++] = '#';
                for (int i=0; i<kohde.length(); i++) {
                    s[j++] = kohde.charAt(i);
                }
                int[] z = toZarray(s);
                int max = 0;
                for (int i=haku.length(); i<z.length; i++) {
                    max = Math.max(max, z[i]);
                }
                return max;
            }

            public int[] toZarray(char[] s) {
                int n = s.length;
                int[] z = new int[n];
                int a = 0, b = 0;
                for (int i = 1; i < n; i++) {
                    if (i > b) {
                        for (int j = i; j < n && s[j - i] == s[j]; j++)  z[i]++;
                    }
                    else {
                        z[i] = z[i - a];
                        if (i + z[i - a] > b) {
                            for (int j = b + 1; j < n && s[j - i] == s[j]; j++) z[i]++;
                            a = i;
                            b = i + z[i] - 1;
                        }
                    }
                }
                return z;
            }

            public List<Integer> getStartIndexesWhereWordIsFound(String haku, String kohde) {
                // this is alternative use case
                char[] s = new char[haku.length() + 1 + kohde.length()];
                for (int i=0; i<haku.length(); i++) {
                    s[i] = haku.charAt(i);
                }
                int j = haku.length();
                s[j++] = '#';
                for (int i=0; i<kohde.length(); i++) {
                    s[j++] = kohde.charAt(i);
                }
                int[] z = toZarray(s);
                List<Integer> indexes = new ArrayList<>();
                for (int i=haku.length(); i<z.length; i++) {
                    if (z[i] < haku.length()) continue;
                    indexes.add(i);
                }
                return indexes;
            }

        }

        class StringHasher {

            class HashedString {
                long[] hashes;
                long[] modifiers;

                public HashedString(long[] hashes, long[] modifiers) {
                    this.hashes = hashes;
                    this.modifiers = modifiers;
                }
            }

            long P;
            long M;

            public StringHasher() {
                initializePandM();
            }

            HashedString hashString(String s) {
                int n = s.length();
                long[] hashes = new long[n];
                long[] modifiers = new long[n];
                hashes[0] = s.charAt(0);
                modifiers[0] = 1;
                for (int i=1; i<n; i++) {
                    hashes[i] = (hashes[i-1] * P + s.charAt(i)) % M;
                    modifiers[i] = (modifiers[i-1] * P) % M;
                }
                return new HashedString(hashes, modifiers);
            }

            /**
             * Indices are inclusive.
             */
            long getHash(HashedString hashedString, int startIndex, int endIndex) {
                long[] hashes = hashedString.hashes;
                long[] modifiers = hashedString.modifiers;
                long result = hashes[endIndex];
                if (startIndex > 0) result -= (hashes[startIndex-1] * modifiers[endIndex-startIndex+1]) % M;
                if (result < 0) result += M;
                return result;
            }

            // Less interesting methods below



            /**
             * Efficient for 2 input parameter strings in particular.
             */
            HashedString[] hashString(String first, String second) {
                HashedString[] array = new HashedString[2];
                int n = first.length();
                long[] modifiers = new long[n];
                modifiers[0] = 1;

                long[] firstHashes = new long[n];
                firstHashes[0] = first.charAt(0);
                array[0] = new HashedString(firstHashes, modifiers);

                long[] secondHashes = new long[n];
                secondHashes[0] = second.charAt(0);
                array[1] = new HashedString(secondHashes, modifiers);

                for (int i=1; i<n; i++) {
                    modifiers[i] = (modifiers[i-1] * P) % M;
                    firstHashes[i] = (firstHashes[i-1] * P + first.charAt(i)) % M;
                    secondHashes[i] = (secondHashes[i-1] * P + second.charAt(i)) % M;
                }
                return array;
            }

            /**
             * Efficient for 3+ strings
             * More efficient than multiple hashString calls IF strings are same length.
             */
            HashedString[] hashString(String... strings) {
                HashedString[] array = new HashedString[strings.length];
                int n = strings[0].length();
                long[] modifiers = new long[n];
                modifiers[0] = 1;
                for (int j=0; j<strings.length; j++) {
                    // if all strings are not same length, defer work to another method
                    if (strings[j].length() != n) {
                        for (int i=0; i<n; i++) {
                            array[i] = hashString(strings[i]);
                        }
                        return array;
                    }

                    // otherwise initialize stuff
                    long[] hashes = new long[n];
                    hashes[0] = strings[j].charAt(0);
                    array[j] = new HashedString(hashes, modifiers);
                }
                for (int i=1; i<n; i++) {
                    modifiers[i] = (modifiers[i-1] * P) % M;
                    for (int j=0; j<strings.length; j++) {
                        String s = strings[j];
                        long[] hashes = array[j].hashes;
                        hashes[i] = (hashes[i-1] * P + s.charAt(i)) % M;
                    }
                }
                return array;
            }

            void initializePandM() {
                ArrayList<Long> modOptions = new ArrayList<>(20);
                modOptions.add(353873237L);
                modOptions.add(353875897L);
                modOptions.add(353878703L);
                modOptions.add(353882671L);
                modOptions.add(353885303L);
                modOptions.add(353888377L);
                modOptions.add(353893457L);
                P = modOptions.get(new Random().nextInt(modOptions.size()));

                modOptions.clear();
                modOptions.add(452940277L);
                modOptions.add(452947687L);
                modOptions.add(464478431L);
                modOptions.add(468098221L);
                modOptions.add(470374601L);
                modOptions.add(472879717L);
                modOptions.add(472881973L);
                M = modOptions.get(new Random().nextInt(modOptions.size()));

            }
        }

        int editDistance(String a, String b) {
            a = "#"+a;
            b = "#"+b;
            int n = a.length();
            int m = b.length();
            int[][] dp = new int[n+1][m+1];
            for (int y=0; y<=n; y++) {
                for (int x=0; x<=m; x++) {
                    if (y == 0) dp[y][x] = x;
                    else if (x == 0) dp[y][x] = y;
                    else {
                        int e1 = dp[y-1][x] + 1;
                        int e2 = dp[y][x-1] + 1;
                        int e3 = dp[y-1][x-1] + (a.charAt(y-1) != b.charAt(x-1) ? 1 : 0);
                        dp[y][x] = min(e1, e2, e3);
                    }
                }
            }
            return dp[n][m];
        }

        /*************************** Technical ***************************/

        private class IO extends PrintWriter {
            private InputStreamReader r;
            private static final int BUFSIZE = 1 << 15;
            private char[] buf;
            private int bufc;
            private int bufi;
            private StringBuilder sb;

            public IO() {
                super(new BufferedOutputStream(System.out));
                r = new InputStreamReader(System.in);

                buf = new char[BUFSIZE];
                bufc = 0;
                bufi = 0;
                sb = new StringBuilder();
            }

            /** Print, flush, return nextInt. */
            private int queryInt(String s) {
                io.println(s);
                io.flush();
                return nextInt();
            }

            /** Print, flush, return nextLong. */
            private long queryLong(String s) {
                io.println(s);
                io.flush();
                return nextLong();
            }

            /** Print, flush, return next word. */
            private String queryNext(String s) {
                io.println(s);
                io.flush();
                return next();
            }

            private void fillBuf() throws IOException {
                bufi = 0;
                bufc = 0;
                while(bufc == 0) {
                    bufc = r.read(buf, 0, BUFSIZE);
                    if(bufc == -1) {
                        bufc = 0;
                        return;
                    }
                }
            }

            private boolean pumpBuf() throws IOException {
                if(bufi == bufc) {
                    fillBuf();
                }
                return bufc != 0;
            }

            private boolean isDelimiter(char c) {
                return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f';
            }

            private void eatDelimiters() throws IOException {
                while(true) {
                    if(bufi == bufc) {
                        fillBuf();
                        if(bufc == 0) throw new RuntimeException("IO: Out of input.");
                    }

                    if(!isDelimiter(buf[bufi])) break;
                    ++bufi;
                }
            }

            public String next() {
                try {
                    sb.setLength(0);

                    eatDelimiters();
                    int start = bufi;

                    while(true) {
                        if(bufi == bufc) {
                            sb.append(buf, start, bufi - start);
                            fillBuf();
                            start = 0;
                            if(bufc == 0) break;
                        }

                        if(isDelimiter(buf[bufi])) break;
                        ++bufi;
                    }

                    sb.append(buf, start, bufi - start);

                    return sb.toString();
                } catch(IOException e) {
                    throw new RuntimeException("IO.next: Caught IOException.");
                }
            }

            public int nextInt() {
                try {
                    int ret = 0;

                    eatDelimiters();

                    boolean positive = true;
                    if(buf[bufi] == '-') {
                        ++bufi;
                        if(!pumpBuf()) throw new RuntimeException("IO.nextInt: Invalid int.");
                        positive = false;
                    }

                    boolean first = true;
                    while(true) {
                        if(!pumpBuf()) break;
                        if(isDelimiter(buf[bufi])) {
                            if(first) throw new RuntimeException("IO.nextInt: Invalid int.");
                            break;
                        }
                        first = false;

                        if(buf[bufi] >= '0' && buf[bufi] <= '9') {
                            if(ret < -214748364) throw new RuntimeException("IO.nextInt: Invalid int.");
                            ret *= 10;
                            ret -= (int)(buf[bufi] - '0');
                            if(ret > 0) throw new RuntimeException("IO.nextInt: Invalid int.");
                        } else {
                            throw new RuntimeException("IO.nextInt: Invalid int.");
                        }

                        ++bufi;
                    }

                    if(positive) {
                        if(ret == -2147483648) throw new RuntimeException("IO.nextInt: Invalid int.");
                        ret = -ret;
                    }

                    return ret;
                } catch(IOException e) {
                    throw new RuntimeException("IO.nextInt: Caught IOException.");
                }
            }

            public long nextLong() {
                try {
                    long ret = 0;

                    eatDelimiters();

                    boolean positive = true;
                    if(buf[bufi] == '-') {
                        ++bufi;
                        if(!pumpBuf()) throw new RuntimeException("IO.nextLong: Invalid long.");
                        positive = false;
                    }

                    boolean first = true;
                    while(true) {
                        if(!pumpBuf()) break;
                        if(isDelimiter(buf[bufi])) {
                            if(first) throw new RuntimeException("IO.nextLong: Invalid long.");
                            break;
                        }
                        first = false;

                        if(buf[bufi] >= '0' && buf[bufi] <= '9') {
                            if(ret < -922337203685477580L) throw new RuntimeException("IO.nextLong: Invalid long.");
                            ret *= 10;
                            ret -= (long)(buf[bufi] - '0');
                            if(ret > 0) throw new RuntimeException("IO.nextLong: Invalid long.");
                        } else {
                            throw new RuntimeException("IO.nextLong: Invalid long.");
                        }

                        ++bufi;
                    }

                    if(positive) {
                        if(ret == -9223372036854775808L) throw new RuntimeException("IO.nextLong: Invalid long.");
                        ret = -ret;
                    }

                    return ret;
                } catch(IOException e) {
                    throw new RuntimeException("IO.nextLong: Caught IOException.");
                }
            }

            public double nextDouble() {
                return Double.parseDouble(next());
            }

        }

        void print(Object output) {
            io.println(output);
        }

        void done(Object output) {
            print(output);
            done();
        }

        void done() {
            io.close();
            throw new RuntimeException("Clean exit");
        }

        long min(long... v) {
            long ans = v[0];
            for (int i=1; i<v.length; i++) {
                ans = Math.min(ans, v[i]);
            }
            return ans;
        }

        double min(double... v) {
            double ans = v[0];
            for (int i=1; i<v.length; i++) {
                ans = Math.min(ans, v[i]);
            }
            return ans;
        }

        int min(int... v) {
            int ans = v[0];
            for (int i=1; i<v.length; i++) {
                ans = Math.min(ans, v[i]);
            }
            return ans;
        }

        long max(long... v) {
            long ans = v[0];
            for (int i=1; i<v.length; i++) {
                ans = Math.max(ans, v[i]);
            }
            return ans;
        }

        double max(double... v) {
            double ans = v[0];
            for (int i=1; i<v.length; i++) {
                ans = Math.max(ans, v[i]);
            }
            return ans;
        }

        int max(int... v) {
            int ans = v[0];
            for (int i=1; i<v.length; i++) {
                ans = Math.max(ans, v[i]);
            }
            return ans;
        }

    }

}



