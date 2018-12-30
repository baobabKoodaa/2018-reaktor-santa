package baobab;

import java.io.*;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.*;

public class SantaSolver {

    // Constants from problem statement
    static double SANTA_HOME_LATITUDE = 68.073611;
    static double SANTA_HOME_LONGITUDE = 29.315278;
    static int MAX_TRIP_WEIGHT = 10000000;

    // Input will be read into these variables
    int endId;
    double[][] c; // coordinate values for destinations. [id][0/1] where 0==latitude, 1==longitude
    int[] w; // weights
    long sumOfAllWeights;
    double[][] dist;

    // Current solution state
    List<Trip> trips;
    int nextFreeTripId;

    // Route creation variables
    List<Integer> candidates;
    long weightRemaining;
    Trip currTrip;
    List<Integer> indicesForCurrTrip;
    double targetDist;
    Set<Integer> challengingNodes;

    // Hyperparameters for route creation
    double randomSize = 0.58; // Optimal around 0.58
    double multiplierToSlightlyExpandMaxSparsity = 1.1; // Optimal between 1.0 - 1.1
    double UTZ_CLUSTER_GOAL = 0.92; // Optimal between 0.92-0.96 (if UTZ_TRIP_GOAL is locked to 0.98)
    double UTZ_TRIP_GOAL = 0.98; // 0.989 would be slightly better, but it's much slower and sometimes stuck at infinite loop
    boolean enableLocalWalk = true;

    // Simulated annealing
    boolean SA_IN_USE = false;
    double temperatureAtRandomStart = 400000.0;
    double temperatureAtJumpStart = 8000.0;
    double temperatureAtOldValley = 1000;
    double temperature = temperatureAtRandomStart;
    double coolingRate = 0.9999999;
    double coolingReduction = 0.001;

    // Freeze condition / shaking
    long freezeCheckLastTime = 0;
    long freezeCheckIntervalSeconds = 65;
    double minScoreSinceLastFreezeCheck = Double.POSITIVE_INFINITY;
    double maxScoreSinceLastFreezeCheck = Double.NEGATIVE_INFINITY;
    double freezeCheckMinimumDiff = 500000;
    double freezeConditionTemperatureMultiplier = 10;
    boolean REORDER_EXPERIMENT = true;

    // Reduce print spam
    boolean verbose = false;
    double lastPrintedScore = Double.POSITIVE_INFINITY;
    double lowestKnownScore = Double.POSITIVE_INFINITY;
    long startTime = System.currentTimeMillis();
    long timeWhenBestScoreReached = startTime;
    long lastScorePrintTime = 0;
    double lastPval;
    double lastPvalProposalVal;
    int[] SAcount = new int[2];
    int countMoves = 0;

    // Reduce file spam
    String OUTPUT_DIR = "outputs";
    String saveFolderPath;
    double bestSavedScore = 7900000000.0;
    double lastScoreSaveTime = 0;
    double saveIntervalSeconds = 120;
    String filePathToBestSavedSolution;
    double bestLoadedScore;

    // Auxiliary
    Random rng;

    public void solve(String[] args) throws Exception {
        createSaveFolder();
        readInput();
        preCalcAllDistances();

        long seed = System.currentTimeMillis();
        rng = new Random(seed);
        System.out.println("Seeding random with " + seed);

        int actionType = 7;
        if (args.length > 0) actionType = Integer.parseInt(args[0]);

             if (actionType == 1) loadAndComparePreviousSolutions();
        else if (actionType == 3) createRoutesFromScratch();
        else if (actionType == 4) digAHoleInOldValley();
        else if (actionType == 5) hillClimbRandomRoute();
        else if (actionType == 6) simulatedAnnealingRandomRoute();
        else if (actionType == 7) simulatedAnnealingJumpStart();
        else if (actionType == 8) livenessExperiment();
    }

    // brutettu up-to-11 kokoset, parannus: +0.13% (eli jos exact solvataan yksittäisiä reittejä ni ehkä isommista enemmän parannusta vielä, 0.3% vois olla odotettu?

    // TODO pakota esikäsittelyl et reittiä luodessa tosi lähekkäin olevat nodet päätyy samaan trippiin

    void livenessExperiment() {
        SA_IN_USE = true;
        double goalMeters = 30000000000L;
        while (true) {
            temperature = temperatureAtRandomStart;
            createBadRouteRandomly();
            double meters = calcScore(trips);
            printStatus(meters);

            while (meters > goalMeters) {
                for (int i=0; i<50000; i++) {
                    tryToExecuteRandomSteal();
                    tryToExecuteRandomSwap();
                }
                meters = calcScore(trips);
                //randomWalk(100);
                //reorderingExperiment();
            }
            printStatus(meters);
            calcLiveness(trips);
        }
    }

    int calcLiveness(List<Trip> trips) {
        boolean helper = SA_IN_USE;
        SA_IN_USE = false;
        int accMoveCount = 0;
        double positiveValSums = 0;
        for (int trip1Index=0; trip1Index<trips.size(); trip1Index++) {
            Trip trip1 = trips.get(trip1Index);
            for (int stop1Index=0; stop1Index<trip1.size(); stop1Index++) {
                for (int trip2Index=trip1Index+1; trip2Index<trips.size(); trip2Index++) { // Only check each pair of trips once
                    Trip trip2 = trips.get(trip2Index);
                    for (int stop2Index=0; stop2Index<trip2.size(); stop2Index++) {
                        double swapVal = getSwapValue(trip1, trip2, stop1Index, stop2Index);
                        if (swapVal >= 0) {
                            accMoveCount++;
                            positiveValSums += swapVal;
                        }
                    }
                }
            }
        }
        SA_IN_USE = helper;
        System.out.println("Liveness: " + accMoveCount + " non negative moves available with total value " + positiveValSums);
        return accMoveCount;
    }

    void hillClimbRandomRoute() {
        createBadRouteRandomly();
        while (true) {
            periodicals();
            tryToExecuteRandomSwap();
            tryToExecuteRandomSteal();
        }
    }

    void simulatedAnnealingRandomRoute() {
        createBadRouteRandomly();
        temperature = temperatureAtRandomStart;
        SA_IN_USE = true;
        while (true) {
            periodicals();
            tryToExecuteRandomSwap();
            tryToExecuteRandomSteal();
        }
    }

    void simulatedAnnealingJumpStart() {
        createRouteFromScratch();
        temperature = temperatureAtJumpStart;
        SA_IN_USE = true;
        while (true) {
            periodicals();
            tryToExecuteRandomSwap();
            tryToExecuteRandomSteal();
        }
    }

    void periodicallyShakeIfNeeded() {
        if (!SA_IN_USE) return;
        long now = System.currentTimeMillis();
        if (freezeCheckLastTime == 0) {
            // No shake at the first call of this method
            freezeCheckLastTime = now;
            return;
        }
        if (now < freezeCheckLastTime + freezeCheckIntervalSeconds * 1000) {
            // Freeze check interval not yet complete.
            return;
        }
        //if (shakingNeeded()) {
        //    System.out.print("Freeze condition detected. ");
        //    //shakeByTemperatureJump();
        //}
        if (REORDER_EXPERIMENT) reorderingExperiment();

        double scoreNow = calcScore(trips);
        minScoreSinceLastFreezeCheck = scoreNow;
        maxScoreSinceLastFreezeCheck = scoreNow;
        freezeCheckLastTime = System.currentTimeMillis();;
    }

    boolean shakingNeeded() {
        double min = minScoreSinceLastFreezeCheck;
        double max = maxScoreSinceLastFreezeCheck;
        double diff = max - min;
        System.out.println("Checking for freeze condition... diff " + formatAnsValue(diff));
        return (diff < freezeCheckMinimumDiff);
    }

    void shakeByTemperatureJump() {
        System.out.println("Shaking by jumping temperature upwards...");
        temperature *= freezeConditionTemperatureMultiplier;
        //probabilisticDetachment();
    }

    void digAHoleInOldValley() throws FileNotFoundException {
        SA_IN_USE = true;
        temperature = temperatureAtOldValley;
        Double val = loadSolution(getFilePath("run224\\santamap33"));
        if (val == null) return;

        //temp8();
        REORDER_EXPERIMENT = true;
        periodicals();
        reorderingExperiment();
        localSearchOrderOfIndividualTrips(trips);
        optimalStealsAsLongAsPossible();
        reorderingExperiment();

        while (true) {
            periodicals();
            //shuffleAndSortIndividualTrips();
            tryToExecuteRandomSwap();
            tryToExecuteRandomSteal();
        }


    }

    void reorderingExperiment() {
        System.out.println("Trying to shuffle individual trip orders with reordering experiment...");
        for (Trip trip : trips) {
            if (trip.isEmpty()) continue;
            trip.updateMeters();
            tryToReorder(trip);
        }
    }

    void tryToReorder(Trip trip) {
        // Precalc move options based on distances
        List<IDval>[] closest = new ArrayList[endId];
        for (int i=0; i<=trip.size(); i++) {
            int from = (i<trip.size()? trip.ids.get(i) : 1);
            closest[from] = new ArrayList<>();
            for (int j=0; j<trip.size(); j++) {
                int to = trip.ids.get(j);
                closest[from].add(new IDval(to, dist[from][to]));
            }
            Collections.sort(closest[from]);
        }
        // Try different orders probabilistically
        List<Integer> bestOrder = trip.ids;
        double bestMeters = trip.meters;
        boolean[] used = new boolean[endId];
        for (int i=0; i<10000; i++) {

            // Create order
            int prevId = 1;
            double meters = - rng.nextInt(50000); // Slightly randomize and favor new solutions
            List<Integer> order = new ArrayList<>();
            for (int j=0; j<trip.size(); j++) {
                // Create likelihoods for picking any of top candidates
                double sum = 0;
                List<IDval> topCandidates = new ArrayList<>(4);
                for (int k=0; k<trip.size(); k++) {
                    IDval candidate = closest[prevId].get(k);
                    if (used[candidate.id]) continue;
                    topCandidates.add(candidate);
                    sum += candidate.val;
                    if (topCandidates.size() == 4) break;
                }
                double adjustedSum = 0;
                for (IDval candidate : topCandidates) {
                    double oppositeVal = sum - candidate.val;
                    adjustedSum += oppositeVal;
                }
                // Move to 1 of top candidates
                double decisionThreshold = rng.nextDouble() * adjustedSum * 0.999999999;
                for (int k=0; k<topCandidates.size(); k++) {
                    IDval candidate = topCandidates.get(k);
                    double oppositeVal = sum-candidate.val;
                    if (decisionThreshold <= oppositeVal) {
                        used[candidate.id] = true;
                        meters += dist[prevId][candidate.id];
                        order.add(candidate.id);
                        prevId = candidate.id;
                        break;
                    }
                    decisionThreshold -= oppositeVal;
                }
            }
            meters += dist[order.get(order.size()-1)][1];

            // Revert "used"
            for (int k=0; k<trip.size(); k++) used[order.get(k)] = false;

            // Is this the best order?
            if (meters < bestMeters) {
                bestMeters = meters;
                bestOrder = order;
            }
        }
        trip.ids = bestOrder;
    }

    double heldKarp(Trip trip) {
        // https://stackoverflow.com/a/40311520/4490400
        HashMap<Integer, Integer> idMapping = new HashMap<>();
        int nextFreeId = 0;
        for (int realId : trip.ids) {
            int shortId = nextFreeId++;
            idMapping.put(shortId, realId);
        }
        int n = trip.size();
        double[][] distance = new double[n][n];
        for (int i = 0; i < n; i++) {
            int realIdI = idMapping.get(i);
            for (int j = i + 1; j < n; j++) {
                int realIdJ = idMapping.get(j);
                double d = dist[realIdI][realIdJ];
                distance[i][j] = d;
                distance[j][i] = d;
            }
        }
        double[][] dp = new double[n][1 << n];
        for (int i = 0; i < dp.length; i++) Arrays.fill(dp[i], Double.POSITIVE_INFINITY);
        dp[0][1] = 0.0;
        for (int mask = 1; mask < (1 << n); mask++) {
            for (int last = 0; last < n; last++) {
                if ((mask & (1 << last)) == 0) continue;
                for (int next = 0; next < n; next++) {
                    if ((mask & (1 << next)) != 0) continue;
                    dp[next][mask | (1 << next)] = Math.min(
                            dp[next][mask | (1 << next)],
                            dp[last][mask] + distance[last][next]);
                }
            }
        }
        double res = Double.POSITIVE_INFINITY;
        for (int lastNode = 0; lastNode < n; lastNode++) {
            res = Math.min(res, dist[lastNode][0] + dp[lastNode][(1 << n) - 1]);
        }
        System.out.println(res);
        temp9 = res;
        return res;
    }

    void temp8() {
        double meters = 0;
        List<Trip> smallTrips = new ArrayList<>();
        for (Trip trip : trips) {
            if (trip.isEmpty()) continue;
            if (trip.size() <= 11) {
                System.out.println("size " + trip.size());
                smallTrips.add(trip);
                meters += trip.meters;
            }
        }
        System.out.println("Alle 11 count " + smallTrips.size());
        System.out.println("Non optimized meters: " + meters);
        double optimized = tempBruteTSP(smallTrips);
        System.out.println("Optimized meters: " + optimized);
        System.out.println("Improvement: " + meters/optimized);
    }
    double tempBruteTSP(List<Trip> trips) {
        double meters = 0;
        for (Trip trip : trips) {
            temp9 = Double.POSITIVE_INFINITY;;
            //tempBruteTSP(trip);
            heldKarp(trip);
            meters += temp9;
        }
        return meters;
    }

    double temp9;
    void tempBruteTSP(Trip trip) {
        int[] order = new int[trip.size()];
        rec(order, trip, 0);
    }
    void rec(int[] order, Trip original, int i) {
        if (i >= original.size()) {
            double meters = 0;
            int prevId = 1;
            for (int j=0; j<order.length; j++) {
                int id = order[j];
                meters += dist[prevId][id];
                prevId = id;
            }
            meters += dist[prevId][1];
            temp9 = Math.min(temp9, meters);
            return;
        }
        int id = original.getIdFromIndex(i);
        for (int j=0; j<order.length; j++) {
            if (order[j] != 0) continue;
            order[j] = id;
            rec(order, original, i+1);
            order[j] = 0;
        }
    }

    void tryToExecuteRandomSwap() {
        Trip trip1 = trips.get(rng.nextInt(trips.size()));
        Trip trip2 = trips.get(rng.nextInt(trips.size()));
        if (trip1.isEmpty() || trip2.isEmpty()) return;
        int stop1index = rng.nextInt(trip1.size());
        int stop2index = rng.nextInt(trip2.size());
        int stop1id = trip1.getIdFromIndex(stop1index);
        int stop2id = trip2.getIdFromIndex(stop2index);
        if (stop1id == stop2id) return;
        double swapVal = getSwapValue(trip1, trip2, stop1index, stop2index);
        if (acceptProposal(swapVal)) {
            if (trip1 != trip2) {
                trip1.addStop(stop1index, stop2id);
                trip2.addStop(stop2index, stop1id);
                trip1.removeId(stop1id);
                trip2.removeId(stop2id);
            } else {
                Trip trip = trip1;
                trip.removeIndex(stop1index);
                trip.addStop(stop1index, stop2id);
                trip.removeIndex(stop2index);
                trip.addStop(stop2index, stop1id);
            }
        }
    }

    double getSwapValue(Trip trip1, Trip trip2, int stop1index, int stop2index) {
        int stop1id = trip1.getIdFromIndex(stop1index);
        int stop2id = trip2.getIdFromIndex(stop2index);
        if (stop1id == stop2id) return 0;
        if (trip1 != trip2) {
            if (trip1.weightSum + w[stop2id] - w[stop1id] > MAX_TRIP_WEIGHT) return Double.NEGATIVE_INFINITY;
            if (trip2.weightSum + w[stop1id] - w[stop2id] > MAX_TRIP_WEIGHT) return Double.NEGATIVE_INFINITY;
            double replacementVal1 = trip2.getReplacementVal(stop1id, stop2index);
            double replacementVal2 = trip1.getReplacementVal(stop2id, stop1index);
            return replacementVal1 + replacementVal2;
        } else {
            return trip1.getSwapVal(stop1index, stop2index);
        }
    }

    void tryToExecuteRandomSteal() {
        Trip giver = trips.get(rng.nextInt(trips.size()));
        Trip taker = trips.get(rng.nextInt(trips.size()));
        if (giver.isEmpty()) return;
        if (giver == taker) return;
        int giverIndex = rng.nextInt(giver.size());
        int takerIndex = rng.nextInt(taker.size() + 1);
        int stealId = giver.getIdFromIndex(giverIndex);
        if (taker.weightSum + w[stealId] > MAX_TRIP_WEIGHT) return;
        double removalVal = giver.getRemovalVal(giverIndex);
        double insertionVal = taker.getInsertionVal(stealId, takerIndex);
        double proposalVal = removalVal + insertionVal;
        if (acceptProposal(proposalVal)) {
            taker.addStop(takerIndex, stealId);
            giver.removeIndex(giverIndex);
        }
    }

    boolean acceptProposal(double proposalVal) {
        if (proposalVal < -999999999999.9) return false; // just a speedup
        if (SA_IN_USE) {
            if (proposalVal >= 0) return true;
            double P = Math.exp(proposalVal / temperature);
            lastPval = P;
            lastPvalProposalVal = proposalVal;
            if (rng.nextDouble() < coolingReduction) temperature *= coolingRate;
            boolean accepted = (P >= rng.nextDouble());
            if (accepted) SAcount[1]++;
            else SAcount[0]++;
            return accepted;
        } else {
            // Default: hill climb.
            return (proposalVal >= 0);
        }
    }


    void createBadRouteRandomly() {
        System.out.println("Generating a random solution from scratch...");
        resetTrips();
        List<Integer> candidates = new ArrayList<>();
        for (int candidate = 2; candidate < endId; candidate++) {
            candidates.add(candidate);
        }
        Collections.shuffle(candidates);
        for (int stop : candidates) {
            Collections.shuffle(trips);
            for (int tripIndex = 0; ; tripIndex++) {
                if (tripIndex >= trips.size()) trips.add(new Trip());
                Trip trip = trips.get(tripIndex);
                if (trip.weightSum + w[stop] <= MAX_TRIP_WEIGHT) {
                    trip.addStop(stop);
                    break;
                }
            }
        }
        addEmptyTrips();
    }

    void nnTSP(Trip trip) {
        List<Integer> newOrder = new ArrayList<>();
        boolean[] newUsed = new boolean[endId];
        int prevId = 1;
        while (newOrder.size() < trip.size()) {
            int closestId = -1;
            Double minDist = Double.POSITIVE_INFINITY;
            for (int id : trip.ids) {
                if (newUsed[id])
                    continue;
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
            for (int i = 0; i < trip.size(); i++) {
                int currId = trip.getIdFromIndex(i);
                if (w[currId] > 50000)
                    continue; // Only detach small items
                int prevId = trip.getIdFromIndex(i - 1);
                int nextId = trip.getIdFromIndex(i + 1);
                double val = dist[prevId][currId] + dist[currId][nextId] - dist[prevId][nextId];
                val *= (1 + 2 * rng.nextDouble()); // probabilistically so some random nodes get detached at the same time, too
                d.add(new DetachmentCandidate(trip, i, val));
            }
        }
        Collections.sort(d, Collections.reverseOrder());
        List<Integer> detachedIds = new ArrayList<>();
        HashMap<Trip, List<Integer>> removeIndices = new HashMap<>();
        for (int i = 0; i < d.size() && d.get(i).val > 20000; i++) {
            DetachmentCandidate detach = d.get(i);
            int id = detach.trip.getIdFromIndex(detach.index);
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
                if (taker.weightSum + w[currId] > MAX_TRIP_WEIGHT)
                    continue;
                int takerBestPos = -1;
                double takerBestPosInsertionVal = Integer.MIN_VALUE;
                for (int newPos = 0; newPos <= taker.size(); newPos++) {
                    int prevId = taker.getIdFromIndex(newPos - 1);
                    int shiftedNextId = taker.getIdFromIndex(newPos);
                    double insertionVal = dist[prevId][shiftedNextId] - (dist[prevId][currId] + dist[currId][shiftedNextId]);
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
                System.out.println("Unable to find placement for id=" + currId + ", creating new trip for it.");
                bestCandidate = new Trip();
                bestCandidateInsertionPos = 0;
                trips.add(bestCandidate);
            }
            bestCandidate.addStop(bestCandidateInsertionPos, currId); // No need to update meters yet
        }
        double valAfterDetachment = calcScore(trips);
        double diff = valBeforeDetachment - valAfterDetachment;
        System.out.println(formatAnsValue(valAfterDetachment) + " Detached " + detachedIds.size() + " destinations (" + Math.round(100.0 * detachedIds.size() / (endId - 2)) + "%) (diff " + diff + ")");
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
            if (this.val - o.val < 0)
                return -1;
            if (this.val - o.val > 0)
                return 1;
            return 0;
        }
    }

    void optimalStealsAsLongAsPossible() {
        while (true) {
            boolean alive = false;
            for (Trip taker : trips) {
                while (true) {
                    boolean takerImproved = false;
                    for (Trip giver : trips) {
                        if (taker == giver) continue;
                        for (int i = 0; i < giver.size(); i++) {
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

            if (!alive) return;
        }
    }

    boolean transferIndex(int index, Trip giver, Trip taker) {
        int currId = giver.getIdFromIndex(index);
        if (taker.weightSum + w[currId] > MAX_TRIP_WEIGHT) return false;
        int prevId = giver.getIdFromIndex(index - 1);
        int nextId = giver.getIdFromIndex(index + 1);
        double removalVal = (dist[prevId][currId] + dist[currId][nextId]) - dist[prevId][nextId];

        int bestPos = -1;
        double bestPosInsertionVal = Integer.MIN_VALUE;
        for (int newPos = 0; newPos <= taker.size(); newPos++) {
            prevId = taker.getIdFromIndex(newPos - 1);
            nextId = taker.getIdFromIndex(newPos);
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
            return true;
        }
    }


    void createRoutesFromScratch() {
        while (true) {
            createRouteFromScratch();
        }
    }

    void createRouteFromScratch() {
        System.out.print("Creating route from scratch... " + (verbose ? "\n" : ""));

        // Init
        resetTrips();
        weightRemaining = sumOfAllWeights;
        candidates = new ArrayList<>();

        // Order candidates mainly based on distance from Santa
        double[] loneliness = getLoneliness();
        List<IDval> sortHelper = new ArrayList<>();
        for (int id = 2; id < endId; id++) {
            double heuristicVal = dist[1][id] + rng.nextInt(100000);
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

        System.out.println("Route value " + formatAnsValue(calcScore(trips)));
        addEmptyTrips();
        writeAnsToFile(trips);
    }

    double getSmallestPossibleClusterSparsity() {
        double min = 0;
        double max = 10000000.0;
        if (possibleToQuicklyFillClusterOfSize(300000))
            max = 299999; // minor speedup
        else
            min = 300000;
        while (min + 5 < max) {
            double mid = min + (max - min) / 2;
            if (possibleToQuicklyFillClusterOfSize(mid)) {
                max = mid;
            } else {
                min = mid + 5;
            }
        }
        return min + 5;
    }

    // This method intentionally overfills (due to use case where this is used to set a lower bound)
    boolean possibleToQuicklyFillClusterOfSize(double maxSparsity) {

        currTrip = new Trip();

        // Lock down the furthest target
        int currId = candidates.get(candidates.size() - 1);
        currTrip.addStop(currId);

        double goal = UTZ_CLUSTER_GOAL * MAX_TRIP_WEIGHT;
        while (true) {
            boolean change = false;
            for (int candidateId : candidates) {
                if (currTrip.used[candidateId])
                    continue;

                // Is candidate within maxSparsity?
                double sparsity = Double.POSITIVE_INFINITY;
                for (int id : currTrip.ids) {
                    sparsity = Math.min(sparsity, dist[id][candidateId]);
                }
                if (sparsity > maxSparsity)
                    continue;

                // Add candidate to cluster
                currTrip.addStop(candidateId);
                if (currTrip.weightSum >= goal)
                    return true;
                change = true;
            }
            if (!change)
                return false;
        }
    }


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
        int clusterTarget = candidates.get(candidates.size() - 1);
        targetDist = dist[1][clusterTarget];

        // these variables will hold the best trip we find
        Trip bestTrip = null;
        List<Integer> bestIndicesForRemoval = null;

        // Create several (greedy, slightly randomized) options for trip to target
        double sparsity = multiplierToSlightlyExpandMaxSparsity * sparsityMin;
        for (; bestTrip == null; sparsity *= 1.04) { // Usually only 1 iteration. In rare cases the initial sparsity is not enough (because we allowed overfill when calculating lower bound for sparsity)
            double detourModifier = 0.01;
            for (int tripOption = 1; tripOption <= 50; tripOption++) {

                currTrip = new Trip();
                indicesForCurrTrip = new ArrayList<>();

                // Lock down the furthest target
                currTrip.used[clusterTarget] = true;
                currTrip.addStop(clusterTarget);
                indicesForCurrTrip.add(candidates.size() - 1);

                // collect cluster (at least) up to UTZ_CLUSTER_GOAL and then zigzag to/from cluster to fill remaining capacity.
                collectClusterAroundTarget(sparsity);

                if (sparsity > 50 * sparsityMin) {
                    // We need this to avoid infinite loop in 2 special cases:
                    // Case 1: last trip may sometimes have low UTZ because there simply isn't enough weight available
                    // Case 2: some late trip may end up impossible due to weight restrictions (e.g. pick up up to 0.97 UTZ, but there's only large items available after that)
                    localSearchOrderOfIndividualTrip(currTrip);
                    bestTrip = currTrip;
                    bestIndicesForRemoval = indicesForCurrTrip;
                    bestTrip.updateMeters();
                    break;
                }

                if (utz(currTrip) < UTZ_CLUSTER_GOAL) {
                    if (tripOption < 2) break; // to speedup
                    continue;
                }

                // Add detours on the way to/from cluster (for this we need to know the first/last entry to cluster, so sort the order first)
                localSearchOrderOfIndividualTrip(currTrip);
                collectDetours(currTrip.firstId(), currTrip.lastId(), sparsity, detourModifier);

                // Is this best tripOption for current target?
                if (utz(currTrip) >= UTZ_TRIP_GOAL) {
                    localSearchOrderOfIndividualTrip(currTrip);
                    currTrip.updateMeters();
                    if (bestTrip == null || currTrip.meters < bestTrip.meters) {
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
                            bestTrip.size() + " stops, " +
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
                    heuristicVal *= (1 + randomSize * rng.nextDouble());
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
                } else {
                    // Case: add to end ("on the way from cluster")
                    closest2 = candidateId;
                    currTrip.addStop(candidateId);
                    indicesForCurrTrip.add(candidateIndex);
                }
            }

        }
    }

    void localSearchOrderOfIndividualTrips(List<Trip> trips) {
        for (Trip trip : trips) {
            localSearchOrderOfIndividualTrip(trip);
        }
    }

    // This method will local walk the order within a single trip to a local optima.
    void localSearchOrderOfIndividualTrip(Trip trip) {
        if (!enableLocalWalk) return;
        if (trip.isEmpty()) return;
        int n = trip.size();
        int fromIndex = 0;
        for (int countWithoutUpdate = 0; countWithoutUpdate <= n; ) {
            int id = trip.getIdFromIndex(fromIndex);
            double removalVal = trip.getRemovalVal(fromIndex);
            int bestIndex = fromIndex;
            double bestPosVal = 0;
            for (int toIndex = 0; toIndex <= n; toIndex++) {
                if (toIndex == fromIndex)
                    continue; // displaces itself and causes bugs
                if (toIndex == fromIndex + 1)
                    continue; // this would also displace itself and cause bugs

                double insertionVal = trip.getInsertionVal(id, toIndex);
                double val = insertionVal + removalVal;
                if (val > bestPosVal) {
                    bestPosVal = val;
                    bestIndex = toIndex;
                }
            }
            if (bestIndex == fromIndex) {
                countWithoutUpdate++;
            } else {
                countWithoutUpdate = 0;
                // Note: atypical way of modifying a trip. Careful with introducing bugs here!
                trip.ids.add(bestIndex, id);
                if (bestIndex > fromIndex) trip.ids.remove(fromIndex);
                else trip.ids.remove(fromIndex + 1);
            }

            fromIndex = (bestIndex + 1) % n;
        }
    }


    void addEmptyTrips() {
        // These provide some flexibility when moving between solutions
        trips.add(new Trip());
        trips.add(new Trip());
    }

    void resetTrips() {
        nextFreeTripId = 0;
        trips = new ArrayList<>();
    }

    class Trip {

        int tripId;
        List<Integer> ids;
        double meters;
        boolean[] used;
        long weightSum;

        public Trip() {
            tripId = nextFreeTripId++;
            ids = new ArrayList<>();
            meters = 0;
            used = new boolean[endId];
            weightSum = 0;
        }

        int getIdFromIndex(int index) {
            if (index == -1 || index == ids.size())
                return 1;
            return ids.get(index);
        }

        // Note: meters need to be updated separately.
        void addStop(int id) {
            addStop(ids.size(), id);
        }

        void addStop(int i, int id) {
            used[id] = true;
            weightSum += w[id];
            ids.add(i, id);
            countMoves++;
        }

        void removeIndex(int i) {
            int id = ids.remove(i);
            used[id] = false;
            weightSum -= w[id];
        }

        void removeId(int id) {
            for (int i = 0; i < ids.size(); i++) {
                if (ids.get(i) == id) {
                    removeIndex(i);
                    return;
                }
            }
        }

        int firstId() {
            return ids.get(0);
        }

        int lastId() {
            return ids.get(ids.size() - 1);
        }

        double updateMeters() {
            meters = 0;
            if (ids.isEmpty())
                return 0;
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

        int size() {
            return ids.size();
        }

        double getRemovalVal(int index) {
            int prevId = getIdFromIndex(index - 1);
            int removeId = getIdFromIndex(index);
            int nextId = getIdFromIndex(index + 1);
            return (dist[prevId][removeId] + dist[removeId][nextId]) - dist[prevId][nextId];
        }

        double getReplacementVal(int newId, int index) {
            int prevId = getIdFromIndex(index - 1);
            int oldId = getIdFromIndex(index);
            int nextId = getIdFromIndex(index + 1);
            return dist[prevId][oldId] + dist[oldId][nextId] - (dist[prevId][newId] + dist[newId][nextId]);
        }

        double getInsertionVal(int newId, int index) {
            int prevId = getIdFromIndex(index - 1);
            int shiftedNextId = getIdFromIndex(index);
            return dist[prevId][shiftedNextId] - (dist[prevId][newId] + dist[newId][shiftedNextId]);
        }

        double getSwapVal(int index1, int index2) {
            if (index1 > index2) {
                int helper = index1;
                index1 = index2;
                index2 = helper;
            }
            int id1 = getIdFromIndex(index1);
            int id2 = getIdFromIndex(index2);
            if (index1 + 1 == index2) {
                int prev = getIdFromIndex(index1 - 1);
                int next = getIdFromIndex(index2 + 1);
                return dist[prev][id1] + dist[id2][next] - (dist[prev][id2] + dist[id1][next]);
            } else {
                double rep1 = getReplacementVal(id2, index1);
                double rep2 = getReplacementVal(id1, index2);
                return rep1 + rep2;
            }
        }

        @Override
        public boolean equals(Object o) {
            return this == o;
        }

        @Override
        public int hashCode() {
            return this.tripId;
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            for (int id : ids) {
                sb.append(id);
                sb.append("; ");
            }
            sb.deleteCharAt(sb.length() - 1);
            sb.deleteCharAt(sb.length() - 1);
            return sb.toString();
        }
    }



    void periodicals() {
        oncePerSecondUpdates();
        periodicallyShakeIfNeeded();
        periodicallySave();
    }

    void oncePerSecondUpdates() {
        long timeNow = System.currentTimeMillis();
        if (timeNow > lastScorePrintTime + 1000) {
            double scoreNow = calcScore(trips);
            updateFreezeCheck(scoreNow);
            printStatus(scoreNow);
        }
    }

    void updateFreezeCheck(double scoreNow) {
        minScoreSinceLastFreezeCheck = Math.min(scoreNow, minScoreSinceLastFreezeCheck);
        maxScoreSinceLastFreezeCheck = Math.max(scoreNow, maxScoreSinceLastFreezeCheck);
    }

    void printStatus(double curr) {
        long now = System.currentTimeMillis();
        assertSolutionValid();
        double prev = lastPrintedScore;
        double diff = prev - curr;
        String s = (diff > 0 ? "+" : ""); // indicate change by + or -, no symbol means no change
        lastPrintedScore = curr;
        if (curr < lowestKnownScore) {
            timeWhenBestScoreReached = now;
            lowestKnownScore = curr;
        }
        String c = formatAnsValue(curr);
        String d = formatAnsValue(diff);
        String b = formatAnsValue(lowestKnownScore);
        int moves = countMoves;
        countMoves = 0;

        String timeFromStart = formatElapsedTime(now - startTime);
        String timeFromBest = formatElapsedTime(now - timeWhenBestScoreReached);

        int sumSA = SAcount[0] + SAcount[1];
        String extras = "";
        if (SA_IN_USE) {
            extras += "SA acceptance: " + SAcount[1] + " of " + (sumSA + " ("
                    + formatPercent(SAcount[1] * 1.0 / sumSA) + ")")
                    + " (Temperature " + Math.round(temperature) + ")"
            ;
            SAcount = new int[2];
        }
        System.out.println(c + " (" + s + d + " diff) (" + b + " best " + timeFromBest + " ago) (" + timeFromStart + " from start) (" + moves + " moves) | " + extras);
        lastScorePrintTime = now;
    }

    /********************************** FILE OPERATIONS ***************************************/

    void readInput() throws FileNotFoundException {
        System.out.println("Reading input...");

        // Find max id
        endId = Integer.MIN_VALUE;
        Scanner scanner = new Scanner(new File("input.txt"));
        while (scanner.hasNext()) {
            String[] line = scanner.nextLine().split(";");
            int id = Integer.parseInt(line[0]);
            endId = Math.max(endId, id + 1);
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

    void readChallengingNodes() throws FileNotFoundException {
        System.out.println("Reading challenging nodes...");
        challengingNodes = new HashSet<>();
        Scanner scanner = new Scanner(new File("challenging_nodes.txt"));
        while (scanner.hasNext()) {
            String line = scanner.nextLine();
            String[] sep = line.split(" ");
            try {
                int id = Integer.parseInt(sep[sep.length - 1]);
                int tripNum = Integer.parseInt(sep[1].substring(1));
                if (tripNum < 500)
                    challengingNodes.add(id);
            } catch (Exception ex) {
                //System.out.println("    Skipping row " + line);
            }
        }
    }

    void loadAndComparePreviousSolutions() throws FileNotFoundException {
        // Trowls through subfolders too
        bestLoadedScore = Double.POSITIVE_INFINITY;
        loadAndComparePreviousSolutions(OUTPUT_DIR);
        File rootDir = new File(OUTPUT_DIR);
        for (File file : rootDir.listFiles()) {
            if (file.isDirectory())
                loadAndComparePreviousSolutions(file.getPath());
        }
        System.out.println("Best solution i=" + filePathToBestSavedSolution + " val=" + formatAnsValue(bestLoadedScore));
    }

    void loadAndComparePreviousSolutions(String folderPath) throws FileNotFoundException {
        for (int i = 1; ; i++) {
            String filePath = folderPath + File.separator + "santamap" + i + ".txt";
            if (!new File(filePath).exists())
                break;
            Double val = loadSolution(filePath);
            if (val != null && val < bestLoadedScore) {
                bestLoadedScore = val;
                filePathToBestSavedSolution = filePath;
            }
        }
    }

    String getFilePath(String fileName) {
        return OUTPUT_DIR + File.separator + fileName + ".txt";
    }

    Double loadSolution(String filePath) throws FileNotFoundException {
        System.out.print("Loading " + filePath + "... ");
        resetTrips();
        File f = new File(filePath);
        if (!f.exists())
            throw new RuntimeException("File doesn't exist: " + filePath);
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
        addEmptyTrips();
        if (!isSolutionValid(trips))
            return null;
        double val = calcScore(trips);
        bestSavedScore = Math.min(bestSavedScore, val);
        System.out.println("Solution value " + formatAnsValue(val));
        return val;

    }

    /* SAVE */

    void periodicallySave() {
        long time = System.currentTimeMillis();
        if (time > lastScoreSaveTime + saveIntervalSeconds * 1000) {
            lastScoreSaveTime = time;
            writeAnsToFile(trips);
        }
    }

    void createSaveFolder() {
        String folderNameStub = "run";
        for (int nextFreeId = 1; ; nextFreeId++) {
            saveFolderPath = OUTPUT_DIR + File.separator + folderNameStub + nextFreeId;
            File saveFolder = new File(saveFolderPath);
            if (!saveFolder.exists()) {
                saveFolder.mkdir();
                break;
            }
        }
    }

    void writeAnsToFile(List<Trip> trips) {
        if (saveFolderPath == null || saveFolderPath.isEmpty()) {
            throw new RuntimeException("Save folder not defined!");
        }
        if (trips.isEmpty()) throw new RuntimeException("Empty ans given in writeOutput call");
        if (!isSolutionValid(trips)) throw new RuntimeException("Attempted to save invalid solution.");
        double val = calcScore(trips);
        if (val + 0.0001 >= bestSavedScore) {
            return;
        }
        bestSavedScore = val;

        // Choose name for output file
        int nextFreeId = 1;
        String fileNameStub = "santamap";
        String fileName;
        while (true) {
            fileName = saveFolderPath + File.separator + fileNameStub + nextFreeId + ".txt";
            if (new File(fileName).exists()) nextFreeId++;
            else break;
        }

        System.out.println("Writing solution of value " + formatAnsValue(val) + " to " + fileName);
        // Write solution to file
        try (Writer writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(fileName), "utf-8"))) {
            for (Trip trip : trips) {
                if (trip.isEmpty()) continue;
                writer.write(trip.toString() + "\n");
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /********************************** UTILITIES ***************************************/

    void assertSolutionValid() {
        if (!isSolutionValid(trips))
            throw new RuntimeException("Solution invalid!");
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
        if (validityCheck.size() != endId - 2) {
            System.out.println("Invalid solution! Expected " + (endId - 2) + " stops, but actual was " + validityCheck.size());
            return false;
        }
        return true;
    }

    void preCalcAllDistances() throws Exception {
        System.out.println("Calculating distances...");
        dist = new double[endId][endId];
        for (int id1 = 1; id1 < endId; id1++) {
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
        double deltaPhi = Math.toRadians(lat2 - lat1);
        double deltaLambda = Math.toRadians(lon2 - lon1);
        double sinPhi = Math.sin(deltaPhi / 2);
        double sinLambda = Math.sin(deltaLambda / 2);
        double a = sinPhi * sinPhi + Math.cos(phi1) * Math.cos(phi2) * sinLambda * sinLambda;
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c;
    }

    double[] getLoneliness() {
        double[] loneliness = new double[endId];
        for (int id = 2; id < endId; id++) {
            // Sort everyone according to dist to id
            List<IDval> neighbors = new ArrayList<>();
            for (int neighbor = 2; neighbor < endId; neighbor++) {
                if (neighbor == id)
                    continue;
                neighbors.add(new IDval(neighbor, dist[id][neighbor]));
            }
            Collections.sort(neighbors);

            // Loneliness of node == radius needed to cluster this node so that sum of weight >= maxTripWeight
            int i = 0;
            for (int sumOfWeight = 0; sumOfWeight < MAX_TRIP_WEIGHT; i++) {
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

    void printTrips(List<Trip> trips) {
        List<Helper> ordered = new ArrayList<>();
        for (Trip trip : trips) {
            ordered.add(new Helper(trip, utz(trip)));
        }
        Collections.sort(ordered);
        for (Helper h : ordered) {
            Trip trip = h.trip;
            double furthestDist = 0;
            for (int id : trip.ids) furthestDist = Math.max(furthestDist, dist[1][id]);
            System.out.println(
                    "Trip #" + trip.tripId +
                            " overall " + Math.round(trip.meters / 1000) + "km, " +
                            "target " + Math.round(furthestDist / 1000) + "km, " +
                            "detours " + Math.round((trip.meters - 2 * furthestDist) / 1000) + "km, " +
                            trip.size() + " stops, " +
                            "utz " + utz(trip)
            );
        }
    }

    class Helper implements Comparable {

        Trip trip;
        double v;

        public Helper(Trip trip, double v) {
            this.trip = trip;
            this.v = v;
        }

        @Override
        public int compareTo(Object o) {
            Helper other = (Helper) o;
            if (this.v < other.v) return 1;
            if (other.v < this.v) return -1;
            return 0;
        }
    }

    String formatAnsValue(double val) {
        NumberFormat formatter = new DecimalFormat("#0.0000");
        return formatter.format(val / 1e9);
    }

    String formatElapsedTime(long diff) {
        int timeDiff = (int) diff / 1000;
        int expendedSeconds = timeDiff % 60;
        int expendedMinutes = (timeDiff / 60) % 60;
        int expendedHours = (timeDiff / 60 / 60);
        String t = expendedSeconds + "";
        if (expendedSeconds < 10)
            t = "0" + t;
        t = expendedMinutes + ":" + t;
        if (expendedMinutes < 10)
            t = "0" + t;
        if (expendedHours > 0) {
            t = expendedHours + ":" + t;
            if (expendedHours < 10)
                t = "0" + t;
        }
        return t;
    }

    String formatPercent(double val) {
        return "" + Math.round(1000000.0 * val) / 1000000.0;
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
            if (this.val < o.val)
                return -1;
            if (this.val > o.val)
                return 1;
            return this.id - o.id;
        }
    }

    void canWeRedistribute() {
        PriorityQueue<Helper> q = new PriorityQueue<>(Collections.reverseOrder());
        for (Trip trip : trips) {
            if (trip.isEmpty()) continue;
            q.add(new Helper(trip, trip.weightSum));
        }
        Trip giver = q.poll().trip;
        System.out.println("Givers weight " + giver.weightSum);
        PriorityQueue<IDval> ids = new PriorityQueue<>();
        for (int id : giver.ids) {
            ids.add(new IDval(id, -w[id]));
        }
        while (!ids.isEmpty()) {
            int id = ids.poll().id;
            Helper taker = q.poll();
            if (taker.v + w[id] > MAX_TRIP_WEIGHT) {
                System.out.println("Can't put id="+id+" (weight " + w[id] + " into taker " + taker.trip.tripId + " (weightSum " + taker.v + "), because weight sum would be " + (taker.v + w[id]));
                return;
            }
            System.out.println("Can put thing with weight " + w[id] + " into taker with weight " + taker.v);
            taker.v += w[id];
            q.add(taker);
        }
        System.out.println("SUCCESS!");
    }

    double averageFill(List<Trip> trips) {
        long sumOfWeights = 0;
        int countOfNonEmptyTrips = 0;
        for (Trip trip : trips) {
            if (trip.isEmpty()) continue;
            sumOfWeights += trip.weightSum;
            countOfNonEmptyTrips++;
        }
        double avgWeight = sumOfWeights * 1.0 / countOfNonEmptyTrips;
        double avgFill = avgWeight / MAX_TRIP_WEIGHT;
        return avgFill;
    }

}
