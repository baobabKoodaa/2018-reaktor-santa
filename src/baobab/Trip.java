package baobab;

import java.util.*;

import static baobab.SantaSolver.*;

public class Trip {

    int tripId;
    List<Integer> ids;
    double meters;
    boolean[] used;
    long weightSum;

    public Trip(int tripId) {
        this.tripId = tripId;
        ids = new ArrayList<>();
        meters = 0;
        used = new boolean[endId];
        weightSum = 0;
    }

    /** Trip starts and ends at 1, ids contains those in between. */
    int getIdFromIndex(int index) {
        if (index == -1 || index == ids.size()) return 1;
        return ids.get(index);
    }

    /** Note: meters need to be updated separately (because of cascading double rounding errors). */
    void addId(int id) {
        addId(ids.size(), id);
    }

    void addId(int i, int id) {
        used[id] = true;
        weightSum += w[id];
        ids.add(i, id);
        sol.countMoves++;
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