#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>

#define MAX_ROUTERS 26          // Maximum number of routers
#define MAX_NAME_LEN 10         // Maximum length of router name
#define INF INT_MAX             // Represents infinity

typedef struct {
    char name[MAX_NAME_LEN];
    int neighbors[MAX_ROUTERS];                 // Array of neighbor router indices
    int neighbor_costs[MAX_ROUTERS];            // Direct link costs to each neighbor
    int neighbor_count;                         // Number of direct neighbors
    // Distance table: distance_table[dest][via] = cost to reach dest via via
    int distance_table[MAX_ROUTERS][MAX_ROUTERS];
    
    // Routing table: [next_hop_index, total_cost] for each destination
    int routing_table[MAX_ROUTERS][2];          
    
    int min_costs[MAX_ROUTERS];                 // Minimum cost to each destination (DV)
    
    bool has_route[MAX_ROUTERS];                // Whether a route exists to each destination
} Router;

//Link structure - represents a network link between two routers
typedef struct {
    char router1[MAX_NAME_LEN];                 // First router name
    char router2[MAX_NAME_LEN];                 // Second router name
    int cost;                                   // Link cost (-1 means remove link)
} Link;

// Global variables
Router routers[MAX_ROUTERS];                    // Array of all routers
char router_names[MAX_ROUTERS][MAX_NAME_LEN];   // Router names in order
int router_count = 0;                           // Total number of routers

Link initial_links[100];                        // Initial topology links
int initial_link_count = 0;                     // Number of initial links

Link updates[100];                              // Topology update links
int update_count = 0;                           // Number of update links

// Function declarations
int find_router_index(const char* name);
void initialize_distance_table(int router_idx);
void get_distance_vector(int router_idx, int* dv);
bool update_distance_table(int router_idx, int neighbor_idx, int* neighbor_dv);
void compute_routing_table(int router_idx);
void print_distance_table(int router_idx, int step);
void print_routing_table(int router_idx);
bool states_equal(int prev_state[MAX_ROUTERS][MAX_ROUTERS][MAX_ROUTERS], 
                  int curr_state[MAX_ROUTERS][MAX_ROUTERS][MAX_ROUTERS]);
void copy_state(int src[MAX_ROUTERS][MAX_ROUTERS][MAX_ROUTERS], 
                int dst[MAX_ROUTERS][MAX_ROUTERS][MAX_ROUTERS]);

/*
 * Find the index of a router by its name
 * 
 * @param name: Router name to search for
 * @return: Router index, or -1 if not found
 */
int find_router_index(const char* name) {
    for (int i = 0; i < router_count; i++) {
        if (strcmp(router_names[i], name) == 0) {
            return i;
        }
    }
    return -1;  // Router not found
}

/*
 * Initialize distance table for a router according to DV algorithm
 * 
 * Sets up initial distance table where:
 * - Direct neighbors have their link cost as distance
 * - All other destinations are set to infinity
 * - Computes initial minimum costs (distance vector)
 * 
 * @param router_idx: Index of router to initialize
 */
void initialize_distance_table(int router_idx) {
    Router* router = &routers[router_idx];
    
    // Initialize all distances to infinity for all destinations except self
    for (int dest = 0; dest < router_count; dest++) {
        if (dest != router_idx) {  // Skip self
            // Initialize all possible paths to infinity
            for (int via = 0; via < router_count; via++) {
                router->distance_table[dest][via] = INF;
            }
            
            // Set direct neighbor costs
            // This implements: D_x(y) = c(x,y) for direct neighbors
            for (int n = 0; n < router->neighbor_count; n++) {
                int neighbor_idx = router->neighbors[n];
                if (dest == neighbor_idx) {
                    router->distance_table[dest][neighbor_idx] = router->neighbor_costs[n];
                }
            }
            
            // Compute minimum cost to this destination
            // This is the initial distance vector entry
            int min_cost = INF;
            for (int via = 0; via < router_count; via++) {
                if (router->distance_table[dest][via] < min_cost) {
                    min_cost = router->distance_table[dest][via];
                }
            }
            router->min_costs[dest] = min_cost;
        }
    }
}

/*
 * Extract distance vector from a router's current state
 * 
 * @param router_idx: Index of the router
 * @param dv: Output array to store distance vector
 */
void get_distance_vector(int router_idx, int* dv) {
    for (int i = 0; i < router_count; i++) {
        if (i != router_idx) {
            // Use minimum cost as distance vector entry
            dv[i] = routers[router_idx].min_costs[i];
        } else {
            // Distance to self is always 0
            dv[i] = 0;
        }
    }
}

/*
 * Update distance table based on received distance vector from neighbor
 * Implements the Bellman-Ford equation: D_x(y) = min_v{c(x,v) + D_v(y)}
 * 
 * @param router_idx: Index of router updating its table / 
 * @param neighbor_idx: Index of neighbor sending the update / 
 * @param neighbor_dv: Distance vector received from neighbor / 
 * @return: true if any distance changed, false otherwise / 
 */
bool update_distance_table(int router_idx, int neighbor_idx, int* neighbor_dv) {
    Router* router = &routers[router_idx];
    bool changed = false;
    
    // Verify this is actually a neighbor (security check)
    bool is_neighbor = false;
    int neighbor_cost = 0;
    for (int n = 0; n < router->neighbor_count; n++) {
        if (router->neighbors[n] == neighbor_idx) {
            is_neighbor = true;
            neighbor_cost = router->neighbor_costs[n];
            break;
        }
    }
    
    if (!is_neighbor) return false;  // Ignore updates from non-neighbors
    
    // Update distance table using Bellman-Ford equation
    // D_x(y) = min_v{c(x,v) + D_v(y)}
    for (int dest = 0; dest < router_count; dest++) {
        if (dest != router_idx) {  // Skip self
            // Calculate new cost: c(x,neighbor) + D_neighbor(dest)
            int new_cost = (neighbor_dv[dest] == INF) ? INF : neighbor_cost + neighbor_dv[dest];
            int old_cost = router->distance_table[dest][neighbor_idx];
            
            // Update the distance table entry
            router->distance_table[dest][neighbor_idx] = new_cost;
            
            if (new_cost != old_cost) {
                changed = true;  // Mark that I changed something
            }
            
            // Recompute minimum cost (distance vector entry)
            int old_min = router->min_costs[dest];
            int new_min = INF;
            for (int via = 0; via < router_count; via++) {
                if (router->distance_table[dest][via] < new_min) {
                    new_min = router->distance_table[dest][via];
                }
            }
            router->min_costs[dest] = new_min;
            
            // If minimum changed, we need to notify neighbors
            if (old_min != new_min) {
                changed = true;
            }
        }
    }
    
    return changed;  // Return whether any changes occurred
}

/*
 * Compute routing table from distance table
 * For each destination, find the best next hop (alphabetically first if tie)
 * @param router_idx: Index of router to compute routing table for
 */
void compute_routing_table(int router_idx) {
    Router* router = &routers[router_idx];
    
    // For each destination, find the best route
    for (int dest = 0; dest < router_count; dest++) {
        if (dest != router_idx) {  // Skip self
            int min_cost = INF;
            int best_via = -1;
            
            // Create sorted list of neighbors for alphabetical tie-breaking
            int sorted_neighbors[MAX_ROUTERS];
            int sorted_count = 0;
            
            // Copy neighbor indices
            for (int n = 0; n < router->neighbor_count; n++) {
                sorted_neighbors[sorted_count++] = router->neighbors[n];
            }
            
            // Sort neighbors alphabetically by name
            for (int i = 0; i < sorted_count - 1; i++) {
                for (int j = 0; j < sorted_count - i - 1; j++) {
                    if (strcmp(router_names[sorted_neighbors[j]], 
                              router_names[sorted_neighbors[j + 1]]) > 0) {
                        int temp = sorted_neighbors[j];
                        sorted_neighbors[j] = sorted_neighbors[j + 1];
                        sorted_neighbors[j + 1] = temp;
                    }
                }
            }
            
            // Find best route (first in alphabetical order if tie)
            for (int n = 0; n < sorted_count; n++) {
                int via = sorted_neighbors[n];
                if (router->distance_table[dest][via] < min_cost) {
                    min_cost = router->distance_table[dest][via];
                    best_via = via;
                }
            }
            
            // Store routing information
            if (best_via != -1 && min_cost < INF) {
                router->routing_table[dest][0] = best_via;      // Next hop
                router->routing_table[dest][1] = min_cost;      // Total cost
                router->has_route[dest] = true;
            } else {
                router->has_route[dest] = false;                // No route available
            }
        }
    }
}

/*
 * Print distance table for a router at a specific time step
 * @param router_idx: Index of router
 * @param step: Current time step
 */
void print_distance_table(int router_idx, int step) {
    Router* router = &routers[router_idx];
    printf("Distance Table of router %s at t=%d:\n", router->name, step);
    
    // Collect all destinations (exclude self)
    int dest_count = 0;
    int destinations[MAX_ROUTERS];
    for (int i = 0; i < router_count; i++) {
        if (i != router_idx) {
            destinations[dest_count++] = i;
        }
    }
    
    if (dest_count > 0) {
        // Print header row (destination names)
        printf("     ");
        for (int d = 0; d < dest_count; d++) {
            printf("%s    ", router_names[destinations[d]]);
        }
        printf("\n");
        
        // Print each row (via router -> costs to destinations)
        for (int d = 0; d < dest_count; d++) {
            int dest = destinations[d];
            printf("%s    ", router_names[dest]);
            
            // Print costs via each possible next hop
            for (int other = 0; other < dest_count; other++) {
                int other_idx = destinations[other];
                int cost = router->distance_table[dest][other_idx];
                if (cost == INF) {
                    printf("INF  ");
                } else {
                    printf("%-5d", cost);
                }
            }
            printf("\n");
        }
    }
    printf("\n");  // Blank line separator
}

/*
 * Print final routing table for a router
 * @param router_idx: Index of router
 */
void print_routing_table(int router_idx) {
    Router* router = &routers[router_idx];
    printf("Routing Table of router %s:\n", router->name);
    
    // Print routes to all reachable destinations in alphabetical order
    for (int dest = 0; dest < router_count; dest++) {
        if (dest != router_idx && router->has_route[dest]) {
            printf("%s,%s,%d\n", 
                   router_names[dest],                          // Destination
                   router_names[router->routing_table[dest][0]], // Next hop
                   router->routing_table[dest][1]);             // Total cost
        }
    }
}

/*
 * Check if two network states are equal (for convergence detection)
 * @param prev_state: Previous network state
 * @param curr_state: Current network state
 * @return: true if states are equal, false otherwise
 */
bool states_equal(int prev_state[MAX_ROUTERS][MAX_ROUTERS][MAX_ROUTERS], 
                  int curr_state[MAX_ROUTERS][MAX_ROUTERS][MAX_ROUTERS]) {
    for (int r = 0; r < router_count; r++) {
        for (int d = 0; d < router_count; d++) {
            for (int v = 0; v < router_count; v++) {
                if (prev_state[r][d][v] != curr_state[r][d][v]) {
                    return false;  // Found a difference
                }
            }
        }
    }
    return true;  // All states are identical
}

/*
 * Copy network state from source to destination
 * @param src: Source state array
 * @param dst: Destination state array
 */
void copy_state(int src[MAX_ROUTERS][MAX_ROUTERS][MAX_ROUTERS], 
                int dst[MAX_ROUTERS][MAX_ROUTERS][MAX_ROUTERS]) {
    for (int r = 0; r < router_count; r++) {
        for (int d = 0; d < router_count; d++) {
            for (int v = 0; v < router_count; v++) {
                dst[r][d][v] = src[r][d][v];
            }
        }
    }
}


//Main function - Entry point of the program
int main(int argc, char* argv[]) {
    char line[256];
    FILE* input = stdin;
    
    // Handle file input if provided
    if (argc > 1) {
        input = fopen(argv[1], "r");
        if (!input) {
            fprintf(stderr, "Error opening file: %s\n", argv[1]);
            return 1;
        }
    }
    
    /*
     * Phase 1: Read router names
     * Read router names until "START" keyword
     */
    while (fgets(line, sizeof(line), input)) {
        line[strcspn(line, "\n")] = 0;  // Remove newline
        if (strcmp(line, "START") == 0) break;
        if (strlen(line) > 0) {
            // Store router name and initialize router structure
            strcpy(router_names[router_count], line);
            strcpy(routers[router_count].name, line);
            routers[router_count].neighbor_count = 0;
            router_count++;
        }
    }
    
    /*
     * Phase 2: Read initial topology links
     * Read links until "UPDATE" keyword
     */
    while (fgets(line, sizeof(line), input)) {
        line[strcspn(line, "\n")] = 0;  // Remove newline
        if (strcmp(line, "UPDATE") == 0) break;
        
        char r1[MAX_NAME_LEN], r2[MAX_NAME_LEN];
        int cost;
        if (sscanf(line, "%s %s %d", r1, r2, &cost) == 3) {
            // Store link information
            strcpy(initial_links[initial_link_count].router1, r1);
            strcpy(initial_links[initial_link_count].router2, r2);
            initial_links[initial_link_count].cost = cost;
            initial_link_count++;
        }
    }
    
    /*
     * Phase 3: Read topology updates
     * Read updates until "END" keyword
     */
    while (fgets(line, sizeof(line), input)) {
        line[strcspn(line, "\n")] = 0;  // Remove newline
        if (strcmp(line, "END") == 0) break;
        
        char r1[MAX_NAME_LEN], r2[MAX_NAME_LEN];
        int cost;
        if (sscanf(line, "%s %s %d", r1, r2, &cost) == 3) {
            // Store update information
            strcpy(updates[update_count].router1, r1);
            strcpy(updates[update_count].router2, r2);
            updates[update_count].cost = cost;
            update_count++;
        }
    }
    
    // Close file if it was opened
    if (input != stdin) {
        fclose(input);
    }
    
    /*
     * Phase 4: Build initial topology
     * Process initial links to build neighbor relationships
     */
    for (int i = 0; i < initial_link_count; i++) {
        int r1_idx = find_router_index(initial_links[i].router1);
        int r2_idx = find_router_index(initial_links[i].router2);
        int cost = initial_links[i].cost;
        
        // Links are bidirectional - add both directions
        
        // Add r2 as neighbor of r1
        routers[r1_idx].neighbors[routers[r1_idx].neighbor_count] = r2_idx;
        routers[r1_idx].neighbor_costs[routers[r1_idx].neighbor_count] = cost;
        routers[r1_idx].neighbor_count++;
        
        // Add r1 as neighbor of r2
        routers[r2_idx].neighbors[routers[r2_idx].neighbor_count] = r1_idx;
        routers[r2_idx].neighbor_costs[routers[r2_idx].neighbor_count] = cost;
        routers[r2_idx].neighbor_count++;
    }
    
    /*
     * Phase 5: Initialize DV algorithm
     * Each router sets up its initial distance table
     */
    for (int i = 0; i < router_count; i++) {
        initialize_distance_table(i);
    }
    
    /*
     * Phase 6: Run DV algorithm to convergence
     * This implements the main DV algorithm loop
     */
    int step = 0;
    
    // Print initial state (t=0)
    for (int i = 0; i < router_count; i++) {
        print_distance_table(i, step);
    }
    
    // Initialize state tracking for convergence detection
    int prev_state[MAX_ROUTERS][MAX_ROUTERS][MAX_ROUTERS];
    int curr_state[MAX_ROUTERS][MAX_ROUTERS][MAX_ROUTERS];
    bool first_iteration = true;
    
    // Main DV algorithm loop
    while (true) {
        // Save current state for convergence detection
        for (int r = 0; r < router_count; r++) {
            for (int d = 0; d < router_count; d++) {
                for (int v = 0; v < router_count; v++) {
                    curr_state[r][d][v] = routers[r].distance_table[d][v];
                }
            }
        }
        
        // Exchange distance vectors (synchronous update)
        int dvs[MAX_ROUTERS][MAX_ROUTERS];
        for (int i = 0; i < router_count; i++) {
            get_distance_vector(i, dvs[i]);  // Get current DV
        }
        
        // Each router processes received distance vectors
        bool changed = false;
        for (int i = 0; i < router_count; i++) {
            for (int n = 0; n < routers[i].neighbor_count; n++) {
                int neighbor_idx = routers[i].neighbors[n];
                // Update distance table based on neighbor's DV
                if (update_distance_table(i, neighbor_idx, dvs[neighbor_idx])) {
                    changed = true;
                }
            }
        }
        
        // Check for convergence
        if (!first_iteration && (states_equal(prev_state, curr_state) || !changed)) {
            break;  // Algorithm has converged
        }
        
        // Print distance tables for this iteration
        step++;
        for (int i = 0; i < router_count; i++) {
            compute_routing_table(i);  // Update routing table
        }
        for (int i = 0; i < router_count; i++) {
            print_distance_table(i, step);
        }
        
        // Prepare for next iteration
        copy_state(curr_state, prev_state);
        first_iteration = false;
    }
    
    //Phase 7: Print final routing tables
    for (int i = 0; i < router_count; i++) {
        print_routing_table(i);
        printf("\n");
    }
    /*
     * Phase 8: Process topology updates (if any)
     * This section handles the UPDATE phase of the input
     */
    if (update_count > 0) {
        // Save current distance vectors before applying updates
        // This is needed for proper propagation after topology changes
        int last_dvs[MAX_ROUTERS][MAX_ROUTERS];
        for (int i = 0; i < router_count; i++) {
            get_distance_vector(i, last_dvs[i]);
        }
        
        /*
         * Apply all topology updates
         * Process each update in the updates array
         */
        for (int u = 0; u < update_count; u++) {
            int r1_idx = find_router_index(updates[u].router1);
            int r2_idx = find_router_index(updates[u].router2);
            int cost = updates[u].cost;
            
            /*
             * Handle link removal (cost = -1)
             * Remove bidirectional link between r1 and r2
             */
            if (cost == -1) {
                // Remove r2 from r1's neighbor list
                for (int n = 0; n < routers[r1_idx].neighbor_count; n++) {
                    if (routers[r1_idx].neighbors[n] == r2_idx) {
                        // Shift remaining neighbors left to fill the gap
                        for (int j = n; j < routers[r1_idx].neighbor_count - 1; j++) {
                            routers[r1_idx].neighbors[j] = routers[r1_idx].neighbors[j + 1];
                            routers[r1_idx].neighbor_costs[j] = routers[r1_idx].neighbor_costs[j + 1];
                        }
                        routers[r1_idx].neighbor_count--;
                        break;
                    }
                }
                
                // Remove r1 from r2's neighbor list (bidirectional)
                for (int n = 0; n < routers[r2_idx].neighbor_count; n++) {
                    if (routers[r2_idx].neighbors[n] == r1_idx) {
                        // Shift remaining neighbors left to fill the gap
                        for (int j = n; j < routers[r2_idx].neighbor_count - 1; j++) {
                            routers[r2_idx].neighbors[j] = routers[r2_idx].neighbors[j + 1];
                            routers[r2_idx].neighbor_costs[j] = routers[r2_idx].neighbor_costs[j + 1];
                        }
                        routers[r2_idx].neighbor_count--;
                        break;
                    }
                }
                
                // Set distance table entries to infinity (no direct path)
                for (int dest = 0; dest < router_count; dest++) {
                    routers[r1_idx].distance_table[dest][r2_idx] = INF;
                    routers[r2_idx].distance_table[dest][r1_idx] = INF;
                }
            } 
            /*
             * Handle link addition/update (cost >= 0)
             * Add or update bidirectional link between r1 and r2
             */
            else {
                bool found_r1_to_r2 = false, found_r2_to_r1 = false;
                
                // Update r1 -> r2 link cost (or add if new)
                for (int n = 0; n < routers[r1_idx].neighbor_count; n++) {
                    if (routers[r1_idx].neighbors[n] == r2_idx) {
                        routers[r1_idx].neighbor_costs[n] = cost;
                        found_r1_to_r2 = true;
                        break;
                    }
                }
                // Add new neighbor if not found
                if (!found_r1_to_r2) {
                    routers[r1_idx].neighbors[routers[r1_idx].neighbor_count] = r2_idx;
                    routers[r1_idx].neighbor_costs[routers[r1_idx].neighbor_count] = cost;
                    routers[r1_idx].neighbor_count++;
                }
                
                // Update r2 -> r1 link cost (bidirectional)
                for (int n = 0; n < routers[r2_idx].neighbor_count; n++) {
                    if (routers[r2_idx].neighbors[n] == r1_idx) {
                        routers[r2_idx].neighbor_costs[n] = cost;
                        found_r2_to_r1 = true;
                        break;
                    }
                }
                // Add new neighbor if not found
                if (!found_r2_to_r1) {
                    routers[r2_idx].neighbors[routers[r2_idx].neighbor_count] = r1_idx;
                    routers[r2_idx].neighbor_costs[routers[r2_idx].neighbor_count] = cost;
                    routers[r2_idx].neighbor_count++;
                }
                
                // Update distance table entries for direct paths
                routers[r1_idx].distance_table[r2_idx][r2_idx] = cost;
                routers[r2_idx].distance_table[r1_idx][r1_idx] = cost;
            }
        }
        
        /*
         * Propagate the topology changes
         * Use the saved distance vectors to update all routers
         */
        for (int i = 0; i < router_count; i++) {
            for (int n = 0; n < routers[i].neighbor_count; n++) {
                int neighbor_idx = routers[i].neighbors[n];
                update_distance_table(i, neighbor_idx, last_dvs[neighbor_idx]);
            }
        }
        
        // Recompute routing tables after topology changes
        for (int i = 0; i < router_count; i++) {
            compute_routing_table(i);
        }
        
        // Print distance tables after initial update propagation
        step++;
        for (int i = 0; i < router_count; i++) {
            print_distance_table(i, step);
        }
        
        /*
         * Run DV algorithm to convergence after topology update
         * Continue until no more changes occur
         */
        while (true) {
            // Get current distance vectors from all routers
            int dvs[MAX_ROUTERS][MAX_ROUTERS];
            for (int i = 0; i < router_count; i++) {
                get_distance_vector(i, dvs[i]);
            }
            
            // Exchange distance vectors and check for changes
            bool changed = false;
            for (int i = 0; i < router_count; i++) {
                for (int n = 0; n < routers[i].neighbor_count; n++) {
                    int neighbor_idx = routers[i].neighbors[n];
                    if (update_distance_table(i, neighbor_idx, dvs[neighbor_idx])) {
                        changed = true;
                    }
                }
            }
            
            // If no changes occurred, algorithm has converged
            if (!changed) break;
            
            // Print distance tables for this iteration
            step++;
            for (int i = 0; i < router_count; i++) {
                compute_routing_table(i);
            }
            for (int i = 0; i < router_count; i++) {
                print_distance_table(i, step);
            }
        }
        
        /*
         * Phase 9: Print final routing tables after convergence
         * Output the final routing tables for all routers
         */
        for (int i = 0; i < router_count; i++) {
            print_routing_table(i);
            printf("\n");  // Blank line separator between routing tables
        }
    }
    
    return 0;  // Program completed successfully
}