// LAB 6 PART 1 SUBMITTED BY TONY NAJJAR (adnnk2)



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <float.h>
#include "route_planner.h"

#define MAX_NODES 10000
#define MAX_EDGES 50000
#define EARTH_RADIUS 6371.0

// Node structure
typedef struct { 
    int id;
    double lat;
    double lon;
} Node;

// Edge structure
typedef struct Edge { 
    int to;
    double weight;
    struct Edge* next;
} Edge;

// Graph structure
typedef struct {
    Node nodes[MAX_NODES];
    Edge* adj_list[MAX_NODES];
    int node_count;
    int node_ids[MAX_NODES];
} Graph;

// Priority queue node for Dijkstra and A*
typedef struct {
    int node;
    double priority;
} PQNode;

// Priority queue
typedef struct {
    PQNode heap[MAX_NODES];
    int size;
} PriorityQueue;




// Helper function to calculate haversine distance
double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    
    double a = sin(dlat/2) * sin(dlat/2) + 
               cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS * c;
}






// Priority queue functions
void pq_init(PriorityQueue* pq) {
    pq->size = 0;
}

void pq_push(PriorityQueue* pq, int node, double priority) {
    int i = pq->size++;
    pq->heap[i].node = node;
    pq->heap[i].priority = priority;
    
    // Bubble up
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (pq->heap[parent].priority <= pq->heap[i].priority) break;
        PQNode temp = pq->heap[parent];
        pq->heap[parent] = pq->heap[i];
        pq->heap[i] = temp;
        i = parent;
    }
}

PQNode pq_pop(PriorityQueue* pq) {
    PQNode result = pq->heap[0];
    pq->heap[0] = pq->heap[--pq->size];
    
    // Bubble down
    int i = 0;
    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;
        
        if (left < pq->size && pq->heap[left].priority < pq->heap[smallest].priority)
            smallest = left;
        if (right < pq->size && pq->heap[right].priority < pq->heap[smallest].priority)
            smallest = right;
        
        if (smallest == i) break;
        
        PQNode temp = pq->heap[i];
        pq->heap[i] = pq->heap[smallest];
        pq->heap[smallest] = temp;
        i = smallest;
    }
    
    return result;
}

int pq_empty(PriorityQueue* pq) {
    return pq->size == 0;
}





// Graph functions
void graph_init(Graph* g) {
    g->node_count = 0;
    for (int i = 0; i < MAX_NODES; i++) {
        g->adj_list[i] = NULL;
        g->node_ids[i] = -1;
    }
}

int find_node_index(Graph* g, int node_id) {
    for (int i = 0; i < g->node_count; i++) {
        if (g->node_ids[i] == node_id) return i;
    }
    return -1;
}

void add_edge(Graph* g, int from, int to, double weight) {
    Edge* edge = (Edge*)malloc(sizeof(Edge));
    edge->to = to;
    edge->weight = weight;
    edge->next = g->adj_list[from];
    g->adj_list[from] = edge;
}




// Dijkstra's algorithm
void dijkstra(Graph* g, int start_idx, int* node_priorities, int* end_idx, double* dist, int* prev, int* nodes_explored) {
    PriorityQueue pq;
    pq_init(&pq);
    
    for (int i = 0; i < g->node_count; i++) {
        dist[i] = DBL_MAX;
        prev[i] = -1;
    }
    
    dist[start_idx] = 0;
    pq_push(&pq, start_idx, 0);
    *nodes_explored = 0;
    
    while (!pq_empty(&pq)) {
        PQNode current = pq_pop(&pq);
        int u = current.node;
        (*nodes_explored)++;
        
        if (u == end_idx) break; //need to change this- not just one end point now
        
        if (current.priority > dist[u]) continue;
        
        Edge* edge = g->adj_list[u];
        while (edge != NULL) {
            int v = edge->to;
            double alt = dist[u] + edge->weight;
            
            if (alt < dist[v]) {
                dist[v] = alt;
                prev[v] = u;
                pq_push(&pq, v, alt);
            }
            edge = edge->next;
        }
    }
}

// A* algorithm

//going to use the A* algorithm but replace the distance from the haversine formula with a multiplier based on priority
//the idea being a destination with high priority gets no distance penalty,
//medium gets a little bit extra
//low becomes even more unfavorable
//and no priority should never be considered if something with priority is available
//NO PRIORITY: 2.50x
//LOW PRIORITY: 1.66x
//MEDIUM PRIORITY: 1.33x
//HIGH PRIORITY: 1.00x
//tldr penalize pathing towards lower priority nodes, meaning high priority nodes *should* be chosen always when viable, as long as there isnt a major distance difference.
//multipliers may require readjustment

//at the time of my extremely late turning in, I cannot say for certain if everything works 100% as intended.
//  further testing and bug finding should be done when I do my analysis

void astar(Graph* g, int start_idx, int* node_priorities, int* end_idx, double* dist, int* prev, int* nodes_explored) {
    PriorityQueue pq;
    pq_init(&pq);

    double priority_multipliers[] = {2.50, 1.66, 1.33, 1.00};
    
    for (int i = 0; i < g->node_count; i++) { //sets all distances and previous nodes to default values
        dist[i] = DBL_MAX;
        prev[i] = -1;
    }
    
    int has_been_visited[g->node_count] = {};
    dist[start_idx] = 0; //set the first node's distance to 0
    //double h = haversine(g->nodes[start_idx].lat, g->nodes[start_idx].lon,
                        //g->nodes[*end_idx].lat, g->nodes[*end_idx].lon); //calculate expected distance between point A and B
    pq_push(&pq, start_idx, 0); //add first node to the pq 
    *nodes_explored = 0;
    
    while (!pq_empty(&pq)) {
        PQNode current = pq_pop(&pq); //retrieve node from pq
        int u = current.node; //get current node id as u
        (*nodes_explored)++;
        has_been_visited[u] = 1; //register node U as having been visited at least once
        if(node_priorities[u] >= 1){
            *end_idx = u; //set the end idx or "end of the path" to be the last visited destination node
            printf("registered new end_idx: %d\n", *end_idx + 1);
        }

        printf("U: %d, DIST[U]: %lf, PRIORITY: %d\n", u+1, dist[u], node_priorities[u]);

        int is_all_destinations_visited = 1; //before nodes are checked, set this value to 1. if a destination is not visited, it becomes 0
        for(int i=1; i<=g->node_count; i++){
            //printf("NODE PRIO[%d]: %d\n", i, node_priorities[i]);
            if(node_priorities[i] >= 1){
                if(has_been_visited[i] != 1){
                    is_all_destinations_visited = 0;
                }
            }
        }
        if(is_all_destinations_visited == 1){ //if this value is still 1, break. Should only happen if all destination nodes have been visited
            *end_idx = u;  //just for good measure set the node that triggered this to be the last in the path.
            printf("All destinations reached? Ending at %d.\n", *end_idx + 1);
            break; 
        }

        //if (u == end_idx) break; //old but keeping it around as reference
        
        Edge* edge = g->adj_list[u];
        while (edge != NULL) { //while current node HAS at least one adjacency:
            int v = edge->to; //next node, v
            double alt = (dist[u] + edge->weight) * priority_multipliers[node_priorities[v]]; //calculate potential new distance to node
                                                                                                //now including a priority multiplier
                                                
            printf("V: %d, ALT[V]: %lf, PRIORITY: %d\n", v+1, alt, node_priorities[v]);

            if (alt < dist[v]) {//distance starts at infinity, so the first visit of node v will always activate this
                                //however, on subsequent visits, if a better route is found, it will be replaced.
                dist[v] = dist[u] + edge->weight; //subbed in normal alt calculation here, as we want the distance to be the real distance, not the multiplied one
                prev[v] = u; //register v's previous node or "parent" as node u
                //double h = haversine(g->nodes[v].lat, g->nodes[v].lon,
                                    //g->nodes[end_idx].lat, g->nodes[end_idx].lon);
                pq_push(&pq, v, alt);
            }
            edge = edge->next; //check other node u adjacencies, if any
        }
    }
}

// Bellman-Ford algorithm
int bellman_ford(Graph* g, int start_idx, int* node_priorities, int* end_idx, double* dist, int* prev, int* nodes_explored) {
    for (int i = 0; i < g->node_count; i++) {
        dist[i] = DBL_MAX;
        prev[i] = -1;
    }
    
    dist[start_idx] = 0;
    *nodes_explored = 0;
    
    // Relax edges |V| - 1 times
    for (int i = 0; i < g->node_count - 1; i++) {
        int updated = 0;
        for (int u = 0; u < g->node_count; u++) {
            if (dist[u] == DBL_MAX) continue;
            
            Edge* edge = g->adj_list[u];
            while (edge != NULL) {
                int v = edge->to;
                if (dist[u] + edge->weight < dist[v]) {
                    dist[v] = dist[u] + edge->weight;
                    prev[v] = u;
                    updated = 1;
                }
                edge = edge->next;
            }
        }
        (*nodes_explored)++;
        if (!updated) break;
    }
    
    // Check for negative cycles
    for (int u = 0; u < g->node_count; u++) {
        if (dist[u] == DBL_MAX) continue;
        
        Edge* edge = g->adj_list[u];
        while (edge != NULL) {
            int v = edge->to;
            if (dist[u] + edge->weight < dist[v]) {
                return 0; // Negative cycle detected
            }
            edge = edge->next;
        }
    }
    
    return 1; // No negative cycle
}





void print_path(Graph* g, int* prev, int start_idx, int end_idx, double distance) {
    if (prev[end_idx] == -1) {
        printf("No path found\n");
        return;
    }
    
    // Build path
    int path[MAX_NODES];
    int path_len = 0;
    int current = end_idx;
    
    while (current != -1) { //trace back the found path, using parent nodes starting from the end
        path[path_len++] = current;
        current = prev[current];
    }
    
    printf("Path from %d to %d: ", g->node_ids[start_idx], g->node_ids[end_idx]);
    for (int i = path_len - 1; i >= 0; i--) {
        printf("%d", g->node_ids[path[i]]);
        if (i > 0) printf(" -> ");
    }
    printf("\nTotal distance: %.2f km\n", distance);
}




int main(int argc, char* argv[]) {
    if (argc != 6) {
        printf("Usage: %s <nodes.csv> <edges.csv> <start_node> <destinations.csv> <algorithm>\n", argv[0]);
        printf("Algorithms: dijkstra, astar, bellman-ford\n");
        return 1;
    }
    
    char* nodes_file = argv[1];
    char* edges_file = argv[2];
    int start_node = atoi(argv[3]);
    char* destinations_file = argv[4];
    char* algorithm = argv[5];
    
    Graph g;
    graph_init(&g);
    int node_priorities[MAX_NODES] = {0};
    
    // Load nodes
    FILE* fp = fopen(nodes_file, "r");
    if (!fp) {
        printf("Error opening nodes file\n");
        return 1;
    }
    
    char line[256];
    fgets(line, sizeof(line), fp); // Skip header
    
    while (fgets(line, sizeof(line), fp)) {
        int id;
        double lat, lon;
        if (sscanf(line, "%d,%lf,%lf", &id, &lat, &lon) == 3) {
            g.nodes[g.node_count].id = id;
            g.nodes[g.node_count].lat = lat;
            g.nodes[g.node_count].lon = lon;
            g.node_ids[g.node_count] = id;
            g.node_count++;
        }
    }
    fclose(fp);
    
    // Load edges
    fp = fopen(edges_file, "r");
    if (!fp) {
        printf("Error opening edges file\n");
        return 1;
    }
    
    fgets(line, sizeof(line), fp); // Skip header
    
    while (fgets(line, sizeof(line), fp)) {
        int from, to;
        double distance;
        if (sscanf(line, "%d,%d,%lf", &from, &to, &distance) == 3) {
            int from_idx = find_node_index(&g, from);
            int to_idx = find_node_index(&g, to);
            if (from_idx != -1 && to_idx != -1) {
                add_edge(&g, from_idx, to_idx, distance);
            }
        }
    }
    fclose(fp);

    //load destinations file
    fp = fopen(destinations_file, "r");
    if (!fp) {
        printf("Error opening destinations file\n");
        return 1;
    }
    
    fgets(line, sizeof(line), fp); // Skip header
    
    while (fgets(line, sizeof(line), fp)) {
        int destination;
        char* priority[16];
        if (sscanf(line, "%d,%s", &destination, priority) == 2) {
            //printf("%s\n", priority);
            if(strcmp("HIGH", priority) == 0){
                node_priorities[destination-1] = 3;
            }else if(strcmp("MEDIUM", priority) == 0){
                node_priorities[destination-1] = 2;
            }else if(strcmp("LOW", priority) == 0){
                node_priorities[destination-1] = 1;
            }else{
                node_priorities[destination-1] = 0;
            }


        }
    }
    fclose(fp);
    
    // Find start and end indices
    int start_idx = find_node_index(&g, start_node);
    int end_idx = -1;
    
    if (start_idx == -1) {
        printf("Invalid start node\n");
        return 1;
    }
    
    double dist[MAX_NODES];
    int prev[MAX_NODES];
    int nodes_explored = 0;
    
    // Run selected algorithm
    if (strcmp(algorithm, "dijkstra") == 0) {
        printf("=== Dijkstra's Algorithm ===\n");
        dijkstra(&g, start_idx, node_priorities, &end_idx, dist, prev, &nodes_explored);
    } else if (strcmp(algorithm, "astar") == 0) {
        printf("=== A* Algorithm ===\n");
        astar(&g, start_idx, node_priorities, &end_idx, dist, prev, &nodes_explored); //pass the address of end_idx, because the astar func will edit it to be the last node in the path
    } else if (strcmp(algorithm, "bellman-ford") == 0) {
        printf("=== Bellman-Ford Algorithm ===\n");
        if (!bellman_ford(&g, start_idx, node_priorities, &end_idx, dist, prev, &nodes_explored)) {
            printf("Negative cycle detected!\n");
            return 1;
        }
    } else {
        printf("Unknown algorithm: %s\n", algorithm);
        printf("Available algorithms: dijkstra, astar, bellman-ford\n");
        return 1;
    }
    
    print_path(&g, prev, start_idx, end_idx, dist[end_idx]);
    printf("Nodes explored: %d\n", nodes_explored);
    
    // Cleanup
    for (int i = 0; i < g.node_count; i++) {
        Edge* edge = g.adj_list[i];
        while (edge != NULL) {
            Edge* next = edge->next;
            free(edge);
            edge = next;
        }
    }
    
    return 0;
}