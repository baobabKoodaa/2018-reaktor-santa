package baobab;

import java.util.*;

import static baobab.SantaSolver.*;

public class Solution {

    List<Trip> trips;
    int nextFreeId;
    int[] SAcount;
    int countMoves;

    public Solution() {
        reset();
    }

    void reset() {
        SAcount = new int[2];
        countMoves = 0;
        trips = new ArrayList<>();
        nextFreeId = 0;
        addEmptyTrip();
        addEmptyTrip();
    }

    /** The intent here is to provide some flexibility when optimizing a solution. */
    Trip addEmptyTrip() {
        Trip trip = new Trip(nextFreeId++);
        trips.add(trip);
        return trip;
    }

    void addExistingTripAndFixItsId(Trip trip) {
        trip.tripId = nextFreeId++;
        trips.add(trip);
    }

    void shuffleTrips() {
        Collections.shuffle(trips);
    }

    Solution getCopy() {
        Solution solutionCopy = new Solution();
        solutionCopy.countMoves = this.countMoves;
        solutionCopy.nextFreeId = this.nextFreeId;
        solutionCopy.SAcount = new int[2];
        solutionCopy.SAcount[0] = this.SAcount[0];
        solutionCopy.SAcount[1] = this.SAcount[1];
        solutionCopy.trips = new ArrayList<>();
        for (Trip trip : this.trips) {
            Trip copy = new Trip(trip.tripId);
            copy.ids = new ArrayList<>();
            for (int id : trip.ids) {
                copy.ids.add(id);
            }
            copy.meters = trip.meters;
            copy.used = new boolean[trip.used.length];
            for (int i=0; i<trip.used.length; i++) {
                copy.used[i] = trip.used[i];
            }
            copy.weightSum = trip.weightSum;
            solutionCopy.trips.add(copy);
        }
        return solutionCopy;
    }

    double calcScore() {
        double meters = 0;
        for (Trip trip : trips) {
            meters += trip.updateMeters();
        }
        return meters;
    }

    int calcLiveness() {
        boolean helper = SA_IN_USE;
        SA_IN_USE = false;
        int accMoveCount = 0;
        double positiveValSums = 0;
        for (int trip1Index=0; trip1Index<trips.size(); trip1Index++) {
            Trip trip1 = trips.get(trip1Index);
            for (int node1Index=0; node1Index<trip1.size(); node1Index++) {
                for (int trip2Index=trip1Index+1; trip2Index<trips.size(); trip2Index++) { // Only check each pair of trips once
                    Trip trip2 = trips.get(trip2Index);
                    for (int node2Index=0; node2Index<trip2.size(); node2Index++) {
                        double swapVal = getSwapValue(trip1, trip2, node1Index, node2Index);
                        if (swapVal >= 0) {
                            accMoveCount++;
                            positiveValSums += swapVal;
                        }
                    }
                }
            }
        }
        SA_IN_USE = helper;
        System.out.println("        Liveness: " + accMoveCount + " non negative moves available with total value " + positiveValSums);
        return accMoveCount;
    }

    boolean isValid() {
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

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        List<Helper> ordered = new ArrayList<>();
        for (Trip trip : trips) {
            ordered.add(new Helper(trip, utz(trip)));
        }
        Collections.sort(ordered);
        for (Helper h : ordered) {
            Trip trip = h.trip;
            double furthestDist = 0;
            for (int id : trip.ids) furthestDist = Math.max(furthestDist, dist[1][id]);
            sb.append("Trip #" + trip.tripId +
                    " overall " + Math.round(trip.meters / 1000) + "km, " +
                    "target " + Math.round(furthestDist / 1000) + "km, " +
                    "detours " + Math.round((trip.meters - 2 * furthestDist) / 1000) + "km, " +
                    trip.size() + " stops, " +
                    "utz " + utz(trip)
                    + "\n"
            );
        }
        return sb.toString();
    }

    void canWeRedistribute() {
        PriorityQueue<Helper> q = new PriorityQueue<>(Collections.reverseOrder());
        for (Trip trip : sol.trips) {
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
}

